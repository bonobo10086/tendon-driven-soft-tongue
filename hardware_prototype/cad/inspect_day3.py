"""Read existing STL files and make day-3 inspection illustrations.

No CAD, STL, BOM, or previous instructions are modified. The bracket cable
exit check uses rays through the actual delivered STL, not a rebuilt mesh.
The PNG files are technical illustrations rendered from the existing STL.
"""

from pathlib import Path
import json
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "tools" / "vendor"))

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont


FONT_PATH = Path("C:/Windows/Fonts/msyh.ttc")


def font(size):
    return ImageFont.truetype(str(FONT_PATH), size)


def read_mesh(name):
    return trimesh.load_mesh(ROOT / "stl" / name, force="mesh", process=True)


def ray_hits(mesh, origin, direction):
    triangles = mesh.triangles
    e1 = triangles[:, 1] - triangles[:, 0]
    e2 = triangles[:, 2] - triangles[:, 0]
    h = np.cross(np.broadcast_to(direction, e2.shape), e2)
    a = np.einsum("ij,ij->i", e1, h)
    f = np.divide(1.0, a, out=np.zeros_like(a), where=np.abs(a) > 1e-10)
    s = origin - triangles[:, 0]
    u = f * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, e1)
    v = f * (q @ direction)
    t = f * np.einsum("ij,ij->i", e2, q)
    hit = (
        (np.abs(a) > 1e-10)
        & (u >= -1e-8)
        & (v >= -1e-8)
        & (u + v <= 1 + 1e-8)
        & (t > 1e-8)
    )
    return np.unique(np.round(t[hit], 5)).tolist()


def projection(mesh, direction, width, height):
    forward = np.asarray(direction, dtype=float)
    forward /= np.linalg.norm(forward)
    reference = np.array([0.0, 1.0, 0.0])
    if abs(reference @ forward) > 0.92:
        reference = np.array([0.0, 0.0, 1.0])
    right = np.cross(reference, forward)
    right /= np.linalg.norm(right)
    up = np.cross(forward, right)
    centre = mesh.bounds.mean(axis=0)
    vertices = mesh.vertices - centre
    xyz = np.column_stack((vertices @ right, vertices @ up, vertices @ forward))
    scale = min((width - 60) / np.ptp(xyz[:, 0]), (height - 60) / np.ptp(xyz[:, 1]))

    def pixels(points):
        points = np.atleast_2d(points) - centre
        return np.column_stack((width / 2 + points @ right * scale, height / 2 - points @ up * scale))

    return pixels(mesh.vertices), xyz[:, 2], pixels


def render(mesh, size=(500, 430), direction=(0.35, -0.25, 1.0)):
    image = Image.new("RGB", size, "#eef3f8")
    draw = ImageDraw.Draw(image)
    xy, depths, pixels = projection(mesh, direction, *size)
    light = np.array([0.2, -0.4, 1.0])
    light /= np.linalg.norm(light)
    for index in np.argsort(depths[mesh.faces].mean(axis=1)):
        normal = mesh.face_normals[index]
        strength = 0.38 + 0.62 * max(0.0, float(normal @ light))
        color = tuple(int(base * strength) for base in (150, 185, 214))
        draw.polygon([tuple(point) for point in xy[mesh.faces[index]]], fill=color)
    return image, pixels


def parts_sheet():
    parts = [
        ("tongue_mold_lower_v2.stl", "下模：4个凸起定位销", "68.16×95.89×29.27 mm"),
        ("tongue_mold_upper_v2.stl", "上模：定位孔、浇口和排气孔", "68.16×95.89×27.27 mm"),
        ("tongue_root_insert_v2.stl", "舌根嵌件：大法兰＋小埋入板", "38.00×19.00×11.50 mm"),
        ("tongue_tip_anchor_v2.stl", "舌尖锚板：4个阶梯小孔", "13.00×7.50×2.40 mm"),
        ("tongue_root_bracket_v2.stl", "舌根支架：L形底座", "56.00×36.00×34.00 mm"),
    ]
    sheet = Image.new("RGB", (1530, 1150), "white")
    draw = ImageDraw.Draw(sheet)
    draw.text((24, 12), "第3天 · 五种打印件识别（来自当前STL，不是实物照片）", fill="#17324d", font=font(28))
    for index, (name, label, dimension) in enumerate(parts):
        column, row = index % 3, index // 3
        x, y = column * 510 + 5, row * 535 + 60
        picture, _ = render(read_mesh(name))
        sheet.paste(picture, (x, y))
        draw.text((x + 12, y + 435), label, fill="#17324d", font=font(21))
        draw.text((x + 12, y + 467), dimension, fill="#17324d", font=font(18))
        draw.text((x + 12, y + 493), name, fill="#526779", font=font(15))
    draw.text((1040, 670), "先用形状认件，再核对尺寸。", fill="#17324d", font=font(22))
    draw.text((1040, 715), "下模总高度包括定位销；", fill="#17324d", font=font(21))
    draw.text((1040, 750), "测高度时不要漏掉销顶。", fill="#17324d", font=font(21))
    draw.text((1040, 805), "图中摆放仅用于展示。", fill="#17324d", font=font(21))
    draw.text((1040, 840), "实际打印朝向需看切片截图。", fill="#17324d", font=font(21))
    return sheet


def bracket_sheet(mesh):
    picture, pixels = render(mesh, size=(900, 550), direction=(0.0, -1.0, 0.0))
    sheet = Image.new("RGB", (1000, 750), "white")
    sheet.paste(picture, (50, 70))
    draw = ImageDraw.Draw(sheet)
    draw.text((25, 15), "必须核对：舌根贴合后，4条拉索的出口不能被支架挡住", fill="#17324d", font=font(25))
    paths = {"LEFT": (-6.0, 25.0), "RIGHT": (6.0, 25.0), "DOWN": (0.0, 21.5), "UP": (0.0, 28.5)}
    for name, (x, y) in paths.items():
        point = np.array([x, 5.0, y - 6.0])
        px, py = pixels(point)[0] + np.array([50, 70])
        draw.ellipse((px - 14, py - 14, px + 14, py + 14), outline="#dc3545", width=4)
        draw.text((px + 18, py - 15), name, fill="#a01325", font=font(17))
    draw.text((30, 640), "红圈只是标注计划出口的位置，不是当前STL已有的孔。", fill="#a01325", font=font(23))
    draw.text((30, 678), "当前文件这里是实心壁。若实物相同，停止放行，先处理设计缺口。", fill="#a01325", font=font(23))
    return sheet


def socket_sheet():
    sheet = Image.new("RGB", (1150, 560), "white")
    draw = ImageDraw.Draw(sheet)
    draw.text((25, 15), "止口怎么试：软管只坐入浅窝；钢棒可以继续穿过小孔", fill="#17324d", font=font(27))
    for row, (label, depth, thickness) in enumerate([("舌根埋入板", 1.5, 3.0), ("舌尖锚板", 1.2, 2.4)]):
        cy = 180 + row * 200
        left, top, bottom, scale = 300, cy - 75, cy + 75, 100
        shoulder, end = left + depth * scale, left + thickness * scale
        draw.rectangle((left, top, end, bottom), fill="#a8c9e3", outline="#17324d", width=2)
        draw.rectangle((left, cy - 58, shoulder, cy + 58), fill="white")
        draw.rectangle((shoulder, cy - 33, end, cy + 33), fill="white")
        draw.rectangle((90, cy - 50, shoulder, cy + 50), fill="#d0eddd", outline="#24865e", width=2)
        draw.rectangle((90, cy - 25, end + 35, cy + 25), fill="#b9c1c9", outline="#586470", width=2)
        draw.text((22, cy - 80), label, fill="#17324d", font=font(23))
        draw.text((25, cy + 85), "剖面示意，径向比例夸张；不要按图测量", fill="#526779", font=font(17))
        draw.text((660, cy - 55), f"软管座：直径2.3 mm，深{depth:.1f} mm", fill="#17324d", font=font(21))
        draw.text((660, cy - 18), "贯通小孔：直径1.3 mm", fill="#17324d", font=font(21))
        draw.text((660, cy + 19), "软管外径2 mm；钢棒直径1 mm", fill="#17324d", font=font(21))
    return sheet


def main():
    out = ROOT / "preview" / "day3"
    out.mkdir(parents=True, exist_ok=True)
    bracket = read_mesh("tongue_root_bracket_v2.stl")
    result = {"scope": "Existing digital STL only; physical print not inspected", "parts": {}, "bracket_cable_exit_rays": {}}
    for name in ["tongue_mold_lower_v2.stl", "tongue_mold_upper_v2.stl", "tongue_root_insert_v2.stl", "tongue_tip_anchor_v2.stl", "tongue_root_bracket_v2.stl"]:
        mesh = read_mesh(name)
        result["parts"][name] = {"size_mm": mesh.extents.round(3).tolist(), "z_min": float(mesh.bounds[0, 2]), "watertight": bool(mesh.is_watertight)}
    for name, (x, y) in {"LEFT": (-6.0, 25.0), "RIGHT": (6.0, 25.0), "DOWN": (0.0, 21.5), "UP": (0.0, 28.5)}.items():
        # CAD to bracket STL: x'=x, y'=-z, z'=y-6.
        hits = ray_hits(bracket, np.array([x, 4.8, y - 6.0]), np.array([0.0, 1.0, 0.0]))
        result["bracket_cable_exit_rays"][name] = {"intersections_mm": hits, "blocked_by_4mm_wall": hits == [0.2, 4.2]}
    parts_sheet().save(out / "parts_identification.png")
    bracket_sheet(bracket).save(out / "bracket_cable_exit_check.png")
    socket_sheet().save(out / "socket_stop_check.png")
    (out / "digital_check.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
