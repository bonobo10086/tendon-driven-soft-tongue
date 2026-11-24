"""Read the actual STL files and render assembly-reference figures.

Only new explanatory images and a read-only audit are written. No STL, BOM,
existing template, CAD model or purchased-part specification is changed.
The drawings describe final positions, not a validated insertion trajectory.
"""

from pathlib import Path
import hashlib
import json
import math
import struct
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "assembly_guide_v3"
sys.path.insert(0, str(ROOT / "tools" / "vendor"))
import build_printables as old
from inspect_day3 import ray_hits

FONT = "C:/Windows/Fonts/msyh.ttc"
INK = "#193348"
MUTED = "#516371"
UPPER = (174, 191, 202)
ROOT_COLOR = (224, 173, 70)
TIP_COLOR = (232, 128, 79)
ROUTE_COLORS = {"LEFT":"#20824c", "RIGHT":"#7e40af", "DOWN":"#207bc1", "UP":"#c33131"}


class Mesh:
    """Minimal triangle-mesh reader for figures; no CAD dependency or repairs."""
    def __init__(self, triangles):
        self.vertices = np.asarray(triangles, dtype=float).reshape(-1, 3)
        self.faces = np.arange(len(self.vertices)).reshape(-1, 3)

    @property
    def triangles(self):
        return self.vertices[self.faces]

    @property
    def bounds(self):
        return np.array([self.vertices.min(axis=0), self.vertices.max(axis=0)])

    @property
    def extents(self):
        return self.bounds[1] - self.bounds[0]

    @property
    def face_normals(self):
        triangles = self.triangles
        normals = np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
        lengths = np.linalg.norm(normals, axis=1)
        return np.divide(normals, lengths[:,None], out=np.zeros_like(normals), where=lengths[:,None]>1e-12)

    def copy(self):
        return Mesh(self.triangles.copy())

    def apply_translation(self, offset):
        self.vertices += np.asarray(offset)


def read_mesh(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + count * 50, "Expected binary STL: " + str(path)
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3,3)), ("attribute", "<u2")])
    records = np.frombuffer(data, dtype=dtype, count=count, offset=84)
    return Mesh(records["vertices"])


def font(size):
    return ImageFont.truetype(FONT, size)


def label(draw, xy, text, size=27, color=INK):
    draw.text(xy, text, font=font(size), fill=color, spacing=9)


def arrow(draw, points, color=INK, width=4, head=13):
    points = np.asarray(points, dtype=float)
    draw.line([tuple(p) for p in points], fill=color, width=width)
    direction = points[-1] - points[-2]
    direction /= np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]])
    base = points[-1] - head * direction
    draw.polygon([tuple(points[-1]), tuple(base + .45 * head * normal), tuple(base - .45 * head * normal)], fill=color)


def scene(items, size=(1380, 830), forward=(0, -1, 0), horizontal=(0, 0, 1), bounds_points=None):
    """Orthographic engineering rendering, with a shared depth sort."""
    forward = np.asarray(forward, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.asarray(horizontal, dtype=float)
    right -= np.dot(right, forward) * forward
    right /= np.linalg.norm(right)
    up = np.cross(forward, right)
    vertices = np.vstack([mesh.vertices for mesh, _ in items]) if bounds_points is None else bounds_points
    uv = np.column_stack((vertices @ right, vertices @ up))
    centre = (uv.min(axis=0) + uv.max(axis=0)) / 2
    scale = min((size[0]-110) / np.ptp(uv[:, 0]), (size[1]-110) / np.ptp(uv[:, 1]))

    def pixels(points):
        points = np.atleast_2d(points)
        uv = np.column_stack((points @ right, points @ up)) - centre
        return np.column_stack((size[0] / 2 + uv[:, 0] * scale, size[1] / 2 - uv[:, 1] * scale))

    image = Image.new("RGB", size, "#f2f5f7")
    draw = ImageDraw.Draw(image)
    faces = []
    light = forward + np.array([.3, -.1, .35])
    light /= np.linalg.norm(light)
    for mesh, color in items:
        xy = pixels(mesh.vertices)
        depths = mesh.vertices @ forward
        normals = mesh.face_normals
        for i, face in enumerate(mesh.faces):
            strength = .42 + .58 * max(0., float(normals[i] @ light))
            rgb = tuple(int(component * strength) for component in color)
            faces.append((float(depths[face].mean()), xy[face], rgb))
    for _, triangle, color in sorted(faces, key=lambda face: face[0]):
        draw.polygon([tuple(p) for p in triangle], fill=color)
    return image, pixels


def load_world(name, y_min, y_max):
    mesh = read_mesh(ROOT / "stl" / name)
    xyz = mesh.vertices.copy()
    if "mold_upper" in name:
        mesh.vertices = np.column_stack((xyz[:,0], y_max-xyz[:,2], xyz[:,1]))
    elif "mold_lower" in name:
        mesh.vertices = np.column_stack((xyz[:,0], y_min+xyz[:,2], -xyz[:,1]))
    elif "root_insert" in name:
        mesh.vertices[:,2] -= 5.
    elif "tip_anchor" in name:
        mesh.vertices = np.column_stack((xyz[:,0], -xyz[:,1], 77.6-xyz[:,2]))
    else:
        raise ValueError(name)
    return mesh


def common_footer(draw, y, extra=""):
    label(draw, (35, y), "根据现有STL绘制；颜色仅用于认件；非1:1打印图；尚未完成实物装配验证。", 22, MUTED)
    if extra:
        label(draw, (35, y + 35), extra, 22, MUTED)


def mold_positions(world):
    upper = world["tongue_mold_upper_v2.stl"]
    root = world["tongue_root_insert_v2.stl"]
    tip = world["tongue_tip_anchor_v2.stl"]
    sheet = Image.new("RGB", (1450, 1220), "white")
    draw = ImageDraw.Draw(sheet)
    label(draw, (35, 20), "图1 · 上模当托盘：根在左，尖在右", 37)
    label(draw, (35, 78), "这里是根/尖件已经落槽的俯视位置，不是管棒组的下压装配动作。", 25, MUTED)
    picture, project = scene([(upper, UPPER), (root, ROOT_COLOR), (tip, TIP_COLOR)], (1380, 850))
    sheet.paste(picture, (35, 145))
    def point(p):
        return project(p)[0] + [35, 145]
    # Leaders target visible material on the insert, not the neighbouring gate.
    rp = point([11., 18., -3.5])
    tp = point([-1.5, 21.25, 76.4])
    label(draw, (60, 117), "① 根嵌件：大板在外侧，小板朝中央", 25)
    arrow(draw, [(280, 149), (280, 205), rp], color="#9a6d14")
    label(draw, (840, 117), "② 尖锚板：薄片立进末端槽", 25)
    arrow(draw, [(1120, 149), (1120, 205), tp], color="#b45527")
    gate = point([0, 30, 8])
    vents = point([3., 25., 73.])
    label(draw, (60, 1012), "根端：靠近大浇口", 27)
    arrow(draw, [(290, 1012), (350, 977), gate], width=3)
    label(draw, (930, 1012), "尖端：靠近两个排气孔", 27)
    arrow(draw, [(1130, 1012), (1090, 965), vents], width=3)
    label(draw, (35, 1075), "先不放软管、弯棒；带4根凸定位销的下模放旁边，L形支架不进模具。", 27)
    common_footer(draw, 1138)
    sheet.save(OUT / "01_mold_positions.png")


def standing_parts(world):
    upper = world["tongue_mold_upper_v2.stl"]
    root = world["tongue_root_insert_v2.stl"]
    tip = world["tongue_tip_anchor_v2.stl"]
    root_lift = root.copy()
    tip_lift = tip.copy()
    root_lift.apply_translation([0, -23, 0])
    tip_lift.apply_translation([0, -23, 0])
    sheet = Image.new("RGB", (1450, 1160), "white")
    draw = ImageDraw.Draw(sheet)
    label(draw, (35, 20), "图2 · 两块板都像“小卡片”一样立着，不是平铺", 35)
    label(draw, (35, 76), "为显示槽位，图中把两件抬高；箭头只表示放入槽的方向。", 25, MUTED)
    picture, project = scene([(upper, UPPER), (root_lift, ROOT_COLOR), (tip_lift, TIP_COLOR)], (1380, 845), forward=(.55, -1, -.25))
    sheet.paste(picture, (35, 125))
    def point(p):
        return project(p)[0] + [35, 125]
    for x, z, color in [(.5, -3.5, "#9a6d14"), (-1.5, 76.4, "#b45527")]:
        arrow(draw, [point([x, 6., z]), point([x, 24., z])], color=color, width=5)
    label(draw, (65, 105), "根：插管面 → 中央", 27, "#9a6d14")
    label(draw, (990, 105), "中央 ← 插管面：尖", 27, "#b45527")
    label(draw, (35, 985), "尖锚板13 mm长边横跨模腔；2.4 mm厚度沿根→尖方向。", 28)
    label(draw, (35, 1030), "大孔浅窝朝两件之间；根件大法兰与小埋入板是一体件，不要拆开。", 27)
    common_footer(draw, 1092)
    sheet.save(OUT / "02_standing_parts.png")


def face_holes(world):
    root = world["tongue_root_insert_v2.stl"]
    tip = world["tongue_tip_anchor_v2.stl"]
    sheet = Image.new("RGB", (1500, 1070), "white")
    draw = ImageDraw.Draw(sheet)
    label(draw, (35, 20), "图3 · 两个插管面面对面，所以正视图左右相反", 35)
    label(draw, (35, 76), "只认中央四路阶梯孔；圈标是孔位提示，不是新增孔。", 25, MUTED)
    for mesh, color, xoff, direction, title, paths, z in [
        (root, ROOT_COLOR, 25, (0, 0, 1), "根嵌件：从舌尖方向看插管面", old.ROOT_PATHS, 6.5),
        (tip, TIP_COLOR, 770, (0, 0, -1), "尖锚板：从舌根方向看插管面", old.TIP_PATHS, 75.2),
    ]:
        image, project = scene([(mesh, color)], (700, 570), forward=direction, horizontal=(1, 0, 0) if direction[2] > 0 else (-1, 0, 0))
        sheet.paste(image, (xoff, 150))
        label(draw, (xoff+15, 115), title, 27)
        for name, (x, y) in paths.items():
            sx, sy = project([x, y, z])[0] + [xoff, 150]
            draw.ellipse((sx-13, sy-13, sx+13, sy+13), outline=ROUTE_COLORS[name], width=4)
            dx, dy = {"LEFT":(-110,-12),"RIGHT":(22,-12),"UP":(-25,-58),"DOWN":(-38,30)}[name]
            if direction[2] < 0 and name in ("LEFT", "RIGHT"):
                dx = 24 if name == "LEFT" else -122
            label(draw, (sx+dx, sy+dy), name, 24, ROUTE_COLORS[name])
    label(draw, (40, 748), "根件外侧4个大孔＝安装孔；小板四角4孔＝硅胶锁孔。", 26)
    label(draw, (40, 790), "它们都不接软管。软管只进入彩圈的4个中央阶梯孔。", 26)
    label(draw, (40, 850), "此图UP/DOWN是设计方向；上模翻作托盘后，不能按桌面上下重新命名。", 26)
    label(draw, (40, 895), "本次先在模具外接一根LEFT管：根图左孔 ↔ 尖图右孔。", 28)
    common_footer(draw, 979)
    sheet.save(OUT / "03_tube_faces.png")


def socket_and_wall():
    sheet = Image.new("RGB", (1500, 1220), "white")
    draw = ImageDraw.Draw(sheet)
    label(draw, (35, 20), "图4 · 软管停在浅窝里；只有钢棒继续穿过小孔", 35)
    label(draw, (35, 78), "纵向剖面示意；孔径和板厚为标称值，图不按比例。", 25, MUTED)
    y, half = 310, 125
    # Root: tube approaches the insert from the right.
    draw.rectangle((280, y-half, 540, y+half), fill="#f1d58b", outline="#9a6d14", width=3)
    draw.rectangle((280, y-36, 540, y+36), fill="white")
    draw.rectangle((410, y-70, 540, y+70), fill="white")
    draw.rectangle((410, y-58, 700, y+58), fill="#b8ddcc", outline="#20824c", width=3)
    # Tip: tube approaches from the left.
    draw.rectangle((960, y-half, 1220, y+half), fill="#f3bd9e", outline="#b45527", width=3)
    draw.rectangle((960, y-36, 1220, y+36), fill="white")
    draw.rectangle((960, y-70, 1090, y+70), fill="white")
    draw.rectangle((800, y-58, 1090, y+58), fill="#b8ddcc", outline="#20824c", width=3)
    draw.rectangle((155, y-24, 1325, y+24), fill="#aab4bd", outline="#566575", width=2)
    label(draw, (290, 140), "根件插管浅座", 29, "#9a6d14")
    label(draw, (970, 140), "尖件插管浅座", 29, "#b45527")
    label(draw, (480, 470), "两块插管面朝中间，软管在两件之间", 29)
    label(draw, (95, 520), "根：Ø2.3 mm座，深1.5 mm", 26)
    label(draw, (890, 520), "尖：Ø2.3 mm座，深1.2 mm", 26)
    label(draw, (95, 565), "细孔Ø1.3 mm；钢棒Ø1 mm；软管外径Ø2 mm。不能把软管顶穿。", 27)
    arrow(draw, [(655, 215), (545, 215)], color="#20824c")
    arrow(draw, [(845, 215), (955, 215)], color="#20824c")
    # End-wall topology: the access holes are closed holes, not open slots.
    y = 830
    for a, b in [(245, 355), (1130, 1240)]:
        draw.rectangle((a, y-110, b, y+110), fill="#cbd6de", outline=MUTED, width=3)
        draw.rectangle((a, y-18, b, y+18), fill="white")
    draw.line((140, y, 1360, y), fill="#697785", width=11)
    draw.line((410, y, 1080, y), fill="#20824c", width=26)
    label(draw, (95, 650), "模具两端是封闭端壁：钢棒必须从短边侧孔穿出去", 30)
    label(draw, (190, 970), "根端壁", 27)
    label(draw, (1090, 970), "尖端壁", 27)
    label(draw, (460, 970), "图中拉直只为说明穿孔关系", 25, MUTED)
    # Forbidden lowering arrow, stopped above the wall.
    arrow(draw, [(300, 700), (300, 745)], color="#bd3c35")
    draw.line((278, 750, 322, 794), fill="#bd3c35", width=6)
    draw.line((322, 750, 278, 794), fill="#bd3c35", width=6)
    label(draw, (35, 1045), "不能把已弯管棒整组从上往下压；装入动作仍需实物低力验证。", 28, "#9f2924")
    label(draw, (35, 1090), "本图不表示已验证的穿棒顺序；卡住即停止，不钻槽、不烫孔。", 26)
    common_footer(draw, 1155)
    sheet.save(OUT / "04_socket_and_closed_walls.png")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    names = ["tongue_mold_upper_v2.stl", "tongue_mold_lower_v2.stl", "tongue_root_insert_v2.stl", "tongue_tip_anchor_v2.stl"]
    paths = [ROOT / "stl" / name for name in names] + [ROOT / "BOM_CN.md", ROOT / "revision_v3" / "print" / "tongue_root_bracket_v3.stl"]
    before = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    upper_print = read_mesh(ROOT / "stl" / "tongue_mold_upper_v2.stl")
    lower_print = read_mesh(ROOT / "stl" / "tongue_mold_lower_v2.stl")
    y_max = old.MOLD_SPLIT_Y + upper_print.extents[2]
    y_min = old.MOLD_SPLIT_Y - (lower_print.extents[2] - old.ALIGN_PIN_HEIGHT)
    world = {name: load_world(name, y_min, y_max) for name in names}
    upper = world["tongue_mold_upper_v2.stl"]
    root = world["tongue_root_insert_v2.stl"]
    tip = world["tongue_tip_anchor_v2.stl"]
    checks = {
        "root_hole_positions": old.ROOT_PATHS,
        "tip_hole_positions": old.TIP_PATHS,
        "root_tube_socket_faces_tip": True,
        "tip_tube_socket_faces_root": True,
        "upper_side_is_cavity_up_when_looking_along_positive_source_y": True,
        "rod_insertion_trajectory_physically_validated": False,
        "stl_and_bom_modified": False,
    }
    # Explicitly verify the two socket shoulders through the delivered meshes.
    for name, (x, y) in old.ROOT_PATHS.items():
        off = np.array([x+.8, y, 8.])
        hits = ray_hits(root, off, np.array([0., 0., -1.]))
        assert hits and abs(hits[0] - 3.) < 1e-4, (name, hits)
    for name, (x, y) in old.TIP_PATHS.items():
        off = np.array([x+.8, y, 74.5])
        hits = ray_hits(tip, off, np.array([0., 0., 1.]))
        assert hits and abs(hits[0] - 1.9) < 1e-4, (name, hits)
    checks["actual_STL_socket_shoulders_confirmed"] = True
    checks["root_face_z_mm"] = 6.5
    checks["root_stop_z_mm"] = 5.0
    checks["tip_face_z_mm"] = 75.2
    checks["tip_stop_z_mm"] = 76.4
    mold_positions(world)
    standing_parts(world)
    face_holes(world)
    socket_and_wall()
    after = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    assert before == after
    checks["preserved_file_sha256"] = after
    (OUT / "DRAWING_CHECK.json").write_text(json.dumps(checks, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("DRAWINGS |", OUT)


if __name__ == "__main__":
    main()
