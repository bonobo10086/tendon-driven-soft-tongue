"""Generate the V2 audited printable hardware for the tendon-driven tongue.

The tongue cavity is derived from the SOFA ``assets/mesh/tongue.obj`` mesh.
Dimensions are millimetres.  The 25 mm/model-unit scale makes the mesh width
44.98 mm, matching the approximately 45 mm human-tongue width reported by
Endo (2021).  All non-source dimensions are collected near the top of this
file so they can be revised after the first physical fit check.

Generated parts are engineering prototypes, not certified medical devices.
They are intended for a desktop demonstration and must not be put in a human
or animal mouth.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
VENDOR = HERE.parent / "tools" / "vendor"
if VENDOR.exists():
    sys.path.insert(0, str(VENDOR))

import trimesh  # noqa: E402


# ---------------------------------------------------------------------------
# Design parameters (millimetres)
# ---------------------------------------------------------------------------
MODEL_SCALE = 25.0
CAVITY_SCALE = 1.004  # ~0.1 mm average release clearance around the tongue

MOLD_MARGIN_X = 11.5
MOLD_MARGIN_Z = 9.0
MOLD_BOTTOM_WALL = 8.0
MOLD_TOP_WALL = 10.0
MOLD_SPLIT_Y = 17.2

M3_CLEARANCE_D = 3.5
ALIGN_PIN_D = 4.0
ALIGN_HOLE_D = 4.4
ALIGN_PIN_HEIGHT = 4.0

POUR_PORT_D = 9.0
VENT_PORT_D = 2.0

# Endo (2021): Shore-A60 silicone tube OD 2 mm / ID 1 mm.
TUBE_OD = 2.0
TUBE_SOCKET_D = 2.3
MANDREL_D = 1.0

# TowerPro publishes a 22.8 x 12.4 mm body in its dimension table.  The ear
# hole spacing below is a common measured value rather than a guaranteed
# manufacturer tolerance, so the single-servo plate is printed only after the
# purchased servo has been measured.
MG90S_BODY_X = 22.8
MG90S_BODY_Y = 12.4
MG90S_HOLE_SPACING = 27.8

# Four paths are an engineering extension of Endo's two-path mechanism.
# Root positions are inside the measured cross-section at z=5 mm.
ROOT_PATHS = {
    "LEFT": (-6.0, 25.0),
    "RIGHT": (6.0, 25.0),
    "DOWN": (0.0, 21.5),
    "UP": (0.0, 28.5),
}

# Paths converge at the narrow tip.  The centre is asymmetric because the
# source mesh itself is slightly asymmetric.
TIP_PATHS = {
    "LEFT": (-4.5, 25.0),
    "RIGHT": (1.5, 25.0),
    "DOWN": (-1.5, 23.5),
    "UP": (-1.5, 26.5),
}

ROOT_PATH_Z = 5.0
TIP_PATH_Z = 76.0


def box(extents, center=(0.0, 0.0, 0.0)):
    mesh = trimesh.creation.box(extents=np.asarray(extents, dtype=float))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_between(p0, p1, radius, sections=48):
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    direction = p1 - p0
    length = float(np.linalg.norm(direction))
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    transform = trimesh.geometry.align_vectors([0, 0, 1], direction / length)
    mesh.apply_transform(transform)
    mesh.apply_translation((p0 + p1) / 2.0)
    return mesh


def union(*meshes):
    meshes = [m for m in meshes if m is not None]
    if len(meshes) == 1:
        return meshes[0]
    return trimesh.boolean.union(meshes, engine="manifold")


def difference(base, *cutters):
    result = base
    for cutter in cutters:
        result = trimesh.boolean.difference([result, cutter], engine="manifold")
    return result


def load_scaled_tongue():
    source = PROJECT / "assets" / "mesh" / "tongue.obj"
    # The OBJ repeats vertices at UV seams.  Merge those copies so the same
    # visually closed surface becomes a watertight solid for Boolean work.
    tongue = trimesh.load_mesh(source, force="mesh", process=False)
    tongue.merge_vertices(merge_tex=True, merge_norm=True)
    tongue.update_faces(tongue.unique_faces())
    tongue.remove_unreferenced_vertices()
    vertices = tongue.vertices.copy()

    # x: left/right, y: lower/upper, z: root/tip in the source SOFA scene.
    x_mid = (vertices[:, 0].min() + vertices[:, 0].max()) / 2.0
    z_root = vertices[:, 2].max()
    y_bottom = vertices[:, 1].min()
    vertices[:, 0] = (vertices[:, 0] - x_mid) * MODEL_SCALE
    vertices[:, 1] = (vertices[:, 1] - y_bottom) * MODEL_SCALE
    vertices[:, 2] = (z_root - vertices[:, 2]) * MODEL_SCALE
    tongue.vertices = vertices
    if tongue.volume < 0:
        tongue.invert()

    # Small uniform enlargement provides demoulding clearance without changing
    # the nominal engineering dimensions used by the inserts.
    centre = tongue.bounds.mean(axis=0)
    tongue.vertices = (tongue.vertices - centre) * CAVITY_SCALE + centre
    return tongue


def path_centre_y(z):
    coeff = np.polyfit([5.0, 40.0, 68.0], [25.0, 9.5, 23.5], 2)
    if z <= 68.0:
        return float(np.polyval(coeff, z))
    return 23.5 + (z - 68.0) * (1.5 / 8.0)


def point_inside_mesh(mesh, point):
    # Dependency-free Moller-Trumbore ray test.  The slightly tilted ray avoids
    # ambiguous intersections exactly on triangle edges.
    origin = np.asarray(point, dtype=float)
    direction = np.array([1.0, 0.00037, 0.00019])
    direction /= np.linalg.norm(direction)
    tri = mesh.vertices[mesh.faces]
    edge1 = tri[:, 1] - tri[:, 0]
    edge2 = tri[:, 2] - tri[:, 0]
    h = np.cross(np.broadcast_to(direction, edge2.shape), edge2)
    a = np.einsum("ij,ij->i", edge1, h)
    valid = np.abs(a) > 1e-10
    f = np.zeros_like(a)
    f[valid] = 1.0 / a[valid]
    s = origin - tri[:, 0]
    u = f * np.einsum("ij,ij->i", s, h)
    q = np.cross(s, edge1)
    v = f * (q @ direction)
    t = f * np.einsum("ij,ij->i", edge2, q)
    hits = valid & (u >= 0) & (v >= 0) & ((u + v) <= 1) & (t > 1e-8)
    return bool(np.count_nonzero(hits) % 2)


def validate_tube_paths(tongue):
    z_values = np.linspace(ROOT_PATH_Z, TIP_PATH_Z - 0.5, 80)
    for name in ROOT_PATHS:
        root_x, root_y = ROOT_PATHS[name]
        tip_x, tip_y = TIP_PATHS[name]
        root_offset = root_y - 25.0
        tip_offset = tip_y - 25.0
        failures = []
        for z in z_values:
            ratio = (z - ROOT_PATH_Z) / (TIP_PATH_Z - ROOT_PATH_Z)
            x = root_x + ratio * (tip_x - root_x)
            offset = root_offset + ratio * (tip_offset - root_offset)
            y = path_centre_y(float(z)) + offset
            # Check the tube centre and a conservative 1.5 mm envelope (1 mm
            # tube radius + roughly 0.5 mm minimum silicone cover).  The first
            # and last 2 mm are supported by rigid inserts and are exempt.
            envelope = [(0.0, 0.0)]
            if ROOT_PATH_Z + 2.0 <= z <= TIP_PATH_Z - 2.0:
                envelope += [(1.5, 0.0), (-1.5, 0.0), (0.0, 1.5), (0.0, -1.5)]
            if any(not point_inside_mesh(tongue, [x + dx, y + dy, z]) for dx, dy in envelope):
                failures.append((x, y, z))
        if failures:
            first = failures[0]
            raise RuntimeError(
                f"{name} tube centre leaves tongue near "
                f"x={first[0]:.2f}, y={first[1]:.2f}, z={first[2]:.2f} mm"
            )


def build_mold_halves(tongue):
    bounds = tongue.bounds
    x_min = bounds[0, 0] - MOLD_MARGIN_X
    x_max = bounds[1, 0] + MOLD_MARGIN_X
    z_min = bounds[0, 2] - MOLD_MARGIN_Z
    z_max = bounds[1, 2] + MOLD_MARGIN_Z
    y_min = bounds[0, 1] - MOLD_BOTTOM_WALL
    y_max = bounds[1, 1] + MOLD_TOP_WALL

    lower = box(
        [x_max - x_min, MOLD_SPLIT_Y - y_min, z_max - z_min],
        [(x_min + x_max) / 2, (y_min + MOLD_SPLIT_Y) / 2, (z_min + z_max) / 2],
    )
    upper = box(
        [x_max - x_min, y_max - MOLD_SPLIT_Y, z_max - z_min],
        [(x_min + x_max) / 2, (MOLD_SPLIT_Y + y_max) / 2, (z_min + z_max) / 2],
    )

    lower = difference(lower, tongue)
    upper = difference(upper, tongue)

    # Pockets for the rigid root and tip inserts.  The small scale enlargement
    # gives about 0.1-0.2 mm FDM assembly clearance; masking tape seals the root
    # slot during casting, as in Endo's published manufacturing procedure.
    root_pocket = union(
        box([24.4, 15.4, 3.4], [0.5, 25.0, 5.0]),
        box([12.4, 10.4, 6.9], [0.5, 25.0, 0.75]),
        box([38.4, 19.4, 3.4], [0.5, 25.0, -3.5]),
    )
    tip_pocket = box([13.4, 7.9, 2.8], [-1.5, 25.0, 76.4])
    for pocket in (root_pocket, tip_pocket):
        lower = difference(lower, pocket)
        upper = difference(upper, pocket)

    # Six M3 through-holes prevent the 96 mm-long mould from opening at its
    # mid-span.  Four separate registration pins control lateral alignment.
    bolt_x = x_max - 5.0
    bolt_z = z_max - 5.0
    bolt_positions = [
        (x_min + 5.0, z_min + 5.0),
        (bolt_x, z_min + 5.0),
        (x_min + 5.0, (z_min + z_max) / 2),
        (bolt_x, (z_min + z_max) / 2),
        (x_min + 5.0, bolt_z),
        (bolt_x, bolt_z),
    ]
    align_positions = [
        (-24.0, z_min + 5.0),
        (24.0, z_min + 5.0),
        (-24.0, bolt_z),
        (24.0, bolt_z),
    ]

    for x, z in bolt_positions:
        hole = cylinder_between(
            [x, y_min - 1.0, z], [x, y_max + 1.0, z], M3_CLEARANCE_D / 2.0
        )
        lower = difference(lower, hole)
        upper = difference(upper, hole)

    for x, z in align_positions:
        pin = cylinder_between(
            [x, MOLD_SPLIT_Y - 0.3, z],
            [x, MOLD_SPLIT_Y + ALIGN_PIN_HEIGHT, z],
            ALIGN_PIN_D / 2.0,
        )
        pin_hole = cylinder_between(
            [x, MOLD_SPLIT_Y - 0.5, z],
            [x, MOLD_SPLIT_Y + ALIGN_PIN_HEIGHT + 0.7, z],
            ALIGN_HOLE_D / 2.0,
        )
        lower = union(lower, pin)
        upper = difference(upper, pin_hole)

    # Pour near the high root region; two small vents near the distal tip.
    pour = cylinder_between(
        [0.0, 30.0, 8.0], [0.0, y_max + 2.0, 8.0], POUR_PORT_D / 2.0
    )
    upper = difference(upper, pour)
    for x in (-4.5, 3.0):
        vent = cylinder_between(
            [x, 25.0, 73.0], [x, y_max + 2.0, 73.0], VENT_PORT_D / 2.0
        )
        upper = difference(upper, vent)

    # Access holes let the four stainless mandrels pass through the end walls.
    for name, (x, y) in ROOT_PATHS.items():
        root_hole = cylinder_between(
            [x, y, z_min - 1.0], [x, y, ROOT_PATH_Z + 1.0], MANDREL_D / 2.0 + 0.15
        )
        upper = difference(upper, root_hole)
    for name, (x, y) in TIP_PATHS.items():
        tip_hole = cylinder_between(
            [x, y, TIP_PATH_Z - 1.0], [x, y, z_max + 1.0], MANDREL_D / 2.0 + 0.15
        )
        upper = difference(upper, tip_hole)

    return lower, upper


def build_root_insert():
    # Embedded plate + narrow neck + external mounting flange.
    embedded = box([24.0, 15.0, 3.0], [0.5, 25.0, 5.0])
    neck = box([12.0, 10.0, 6.5], [0.5, 25.0, 0.75])
    flange = box([38.0, 19.0, 3.0], [0.5, 25.0, -3.5])
    part = union(embedded, neck, flange)

    # A 1.3 mm through-hole carries the 1 mm mandrel and 0.45 mm cable.  A
    # 2.3 mm counterbore from the tongue side forms a positive stop for the
    # 2 mm OD tube at z=5 mm, preventing it from sliding during casting.
    for x, y in ROOT_PATHS.values():
        through_hole = cylinder_between(
            [x, y, -6.0], [x, y, 8.0], MANDREL_D / 2.0 + 0.15
        )
        tube_socket = cylinder_between([x, y, 5.0], [x, y, 8.0], TUBE_SOCKET_D / 2.0)
        part = difference(part, through_hole, tube_socket)

    # Mechanical-lock perforations let silicone connect through the insert.
    lock_points = [(-9, 20), (-9, 30), (10, 20), (10, 30)]
    for x, y in lock_points:
        hole = cylinder_between([x, y, 3.0], [x, y, 7.0], 1.6)
        part = difference(part, hole)

    # Four mounting holes in the external flange.
    for x, y in [(-15, 19), (16, 19), (-15, 31), (16, 31)]:
        hole = cylinder_between([x, y, -6.0], [x, y, -1.0], M3_CLEARANCE_D / 2.0)
        part = difference(part, hole)
    return part


def build_tip_anchor():
    # A small rigid cap at the distal end.  The four cables emerge through this
    # cap and are secured by crimp sleeves; a cosmetic silicone cover is added
    # only after the mechanism has been tested.
    part = box([13.0, 7.5, 2.4], [-1.5, 25.0, 76.4])
    for x, y in TIP_PATHS.values():
        through_hole = cylinder_between(
            [x, y, 74.5], [x, y, 78.0], MANDREL_D / 2.0 + 0.15
        )
        tube_socket = cylinder_between([x, y, 74.5], [x, y, 76.4], TUBE_SOCKET_D / 2.0)
        part = difference(part, through_hole, tube_socket)
    return part


def build_root_bracket():
    # L-bracket: vertical root-insert mounting wall and horizontal base plate.
    # Its front face is z=-5 mm, exactly flush with the rear face of the root
    # insert flange; V1 accidentally left a 1 mm air gap here.
    vertical = box([46.0, 30.0, 4.0], [0.5, 25.0, -7.0])
    base = box([56.0, 5.0, 36.0], [0.5, 8.5, -22.0])
    part = union(vertical, base)

    for x, y in [(-15, 19), (16, 19), (-15, 31), (16, 31)]:
        hole = cylinder_between([x, y, -11.0], [x, y, -5.0], M3_CLEARANCE_D / 2.0)
        part = difference(part, hole)
    for x, z in [(-22, -14), (23, -14), (-22, -30), (23, -30)]:
        hole = cylinder_between([x, 5.0, z], [x, 11.0, z], 2.1)
        part = difference(part, hole)
    return part


def build_mg90s_single_test_mount():
    # One-servo electronics/test plate.  A four-servo final rack is deliberately
    # deferred until the cast tongue's required pulling force selects the real
    # actuator.  This also avoids the overlapping mounting ears in the V1 rack.
    rack = box([60.0, 44.0, 4.0], [0.0, 0.0, 0.0])
    body_cut = box([MG90S_BODY_X + 0.6, MG90S_BODY_Y + 0.6, 8.0], [0.0, 0.0, 0.0])
    rack = difference(rack, body_cut)
    for x in (-MG90S_HOLE_SPACING / 2.0, MG90S_HOLE_SPACING / 2.0):
        hole = cylinder_between([x, 0.0, -3.0], [x, 0.0, 3.0], 1.2)
        rack = difference(rack, hole)
    for x, y in [(-25, -17.5), (25, -17.5), (-25, 17.5), (25, 17.5)]:
        hole = cylinder_between([x, y, -3.0], [x, y, 3.0], 2.1)
        rack = difference(rack, hole)
    return rack


def orient_for_print(filename, mesh):
    """Put every large flat mounting face on the slicer's XY build plane."""
    result = mesh.copy()
    if "mold_lower" in filename or "root_bracket" in filename:
        rotation = trimesh.transformations.rotation_matrix(math.pi / 2.0, [1, 0, 0])
        result.apply_transform(rotation)
    elif "mold_upper" in filename:
        rotation = trimesh.transformations.rotation_matrix(-math.pi / 2.0, [1, 0, 0])
        result.apply_transform(rotation)
    elif "tip_anchor" in filename:
        # Put the 2.3 mm tube counterbores on the top face instead of against
        # the first layer, where elephant-foot and bed squish could close them.
        rotation = trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0])
        result.apply_transform(rotation)
    result.apply_translation([0.0, 0.0, -result.bounds[0, 2]])
    return result


def export_mesh(mesh, path):
    mesh.remove_unreferenced_vertices()
    if not mesh.is_watertight:
        raise RuntimeError(f"Generated mesh is not watertight: {path.name}")
    parent = np.arange(len(mesh.vertices), dtype=np.int64)

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def join(a, b):
        ra, rb = find(int(a)), find(int(b))
        if ra != rb:
            parent[rb] = ra

    for a, b, c in mesh.faces:
        join(a, b)
        join(b, c)
    components = len({find(i) for i in range(len(mesh.vertices))})
    if components != 1:
        raise RuntimeError(f"Generated mesh has {components} disconnected bodies: {path.name}")
    mesh.export(path)


def write_path_template(path):
    # 1:1 SVG reference used to pre-bend the mandrels in the y-z plane.
    width, height = 95, 46
    zs = np.linspace(ROOT_PATH_Z, TIP_PATH_Z, 100)
    # Quadratic through (5,25), (40,9.5), (68,23.5), then gently approaches 25.
    coeff = np.polyfit([5.0, 40.0, 68.0], [25.0, 9.5, 23.5], 2)
    centre = np.polyval(coeff, np.minimum(zs, 68.0))
    centre[zs > 68.0] = 23.5 + (zs[zs > 68.0] - 68.0) * (1.5 / 8.0)

    def points(offset):
        return " ".join(f"{8 + z:.2f},{40 - (y + offset):.2f}" for z, y in zip(zs, centre))

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}mm" height="{height}mm" viewBox="0 0 {width} {height}">
  <rect x="0.25" y="0.25" width="{width-0.5}" height="{height-0.5}" fill="white" stroke="black" stroke-width="0.5"/>
  <polyline points="{points(3.5)}" fill="none" stroke="#d62728" stroke-width="0.45"/>
  <polyline points="{points(0.0)}" fill="none" stroke="#222" stroke-width="0.45"/>
  <polyline points="{points(-3.5)}" fill="none" stroke="#1f77b4" stroke-width="0.45"/>
  <line x1="5" y1="43" x2="25" y2="43" stroke="black" stroke-width="0.5"/>
  <text x="5" y="41.5" font-size="2.7">20 mm scale check</text>
  <text x="4" y="4" font-size="3">Mandrel bending template, print at 100% / actual size</text>
  <text x="83" y="8" font-size="2.5" fill="#d62728">UP</text>
  <text x="83" y="12" font-size="2.5">LEFT/RIGHT</text>
  <text x="83" y="16" font-size="2.5" fill="#1f77b4">DOWN</text>
</svg>'''
    path.write_text(svg, encoding="utf-8")


def write_top_path_template(path):
    width, height = 95, 50
    zs = np.linspace(ROOT_PATH_Z, TIP_PATH_Z, 100)

    def points(name):
        root_x = ROOT_PATHS[name][0]
        tip_x = TIP_PATHS[name][0]
        ratio = (zs - ROOT_PATH_Z) / (TIP_PATH_Z - ROOT_PATH_Z)
        xs = root_x + ratio * (tip_x - root_x)
        return " ".join(f"{8 + z:.2f},{25 - x:.2f}" for z, x in zip(zs, xs))

    colours = {"LEFT": "#2ca02c", "RIGHT": "#9467bd", "DOWN": "#1f77b4", "UP": "#d62728"}
    paths = "\n".join(
        f'  <polyline points="{points(name)}" fill="none" stroke="{colour}" stroke-width="0.45"/>'
        for name, colour in colours.items()
    )
    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}mm" height="{height}mm" viewBox="0 0 {width} {height}">
  <rect x="0.25" y="0.25" width="{width-0.5}" height="{height-0.5}" fill="white" stroke="black" stroke-width="0.5"/>
{paths}
  <line x1="5" y1="46" x2="25" y2="46" stroke="black" stroke-width="0.5"/>
  <text x="5" y="44.5" font-size="2.7">20 mm scale check</text>
  <text x="4" y="4" font-size="3">Mandrel top-view template, print at 100% / actual size</text>
  <text x="81" y="10" font-size="2.5" fill="#2ca02c">LEFT</text>
  <text x="81" y="14" font-size="2.5" fill="#9467bd">RIGHT</text>
  <text x="81" y="18" font-size="2.5" fill="#1f77b4">DOWN</text>
  <text x="81" y="22" font-size="2.5" fill="#d62728">UP</text>
</svg>'''
    path.write_text(svg, encoding="utf-8")


def main():
    out = HERE.parent / "stl"
    out.mkdir(parents=True, exist_ok=True)
    tongue = load_scaled_tongue()
    validate_tube_paths(tongue)
    lower, upper = build_mold_halves(tongue)
    parts = {
        "tongue_mold_lower_v2.stl": lower,
        "tongue_mold_upper_v2.stl": upper,
        "tongue_root_insert_v2.stl": build_root_insert(),
        "tongue_tip_anchor_v2.stl": build_tip_anchor(),
        "tongue_root_bracket_v2.stl": build_root_bracket(),
        "mg90s_single_test_mount_v2.stl": build_mg90s_single_test_mount(),
    }
    for filename, mesh in parts.items():
        printable = orient_for_print(filename, mesh)
        export_mesh(printable, out / filename)
        bounds = printable.bounds
        size = bounds[1] - bounds[0]
        print(
            f"{filename}: watertight={mesh.is_watertight}, "
            f"size={size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} mm"
        )
    write_path_template(HERE.parent / "tube_path_template_1to1.svg")
    write_top_path_template(HERE.parent / "tube_path_template_top_1to1.svg")


if __name__ == "__main__":
    main()
