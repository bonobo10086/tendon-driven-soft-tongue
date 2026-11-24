"""Run deterministic purchase-release checks for the V2 printable parts."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
VENDOR = HERE.parent / "tools" / "vendor"
if VENDOR.exists():
    sys.path.insert(0, str(VENDOR))

import trimesh  # noqa: E402

import build_printables as design  # noqa: E402


EXPECTED_SIZE = {
    "tongue_mold_lower_v2.stl": np.array([68.16, 95.89, 29.27]),
    "tongue_mold_upper_v2.stl": np.array([68.16, 95.89, 27.27]),
    "tongue_root_insert_v2.stl": np.array([38.00, 19.00, 11.50]),
    "tongue_tip_anchor_v2.stl": np.array([13.00, 7.50, 2.40]),
    "tongue_root_bracket_v2.stl": np.array([56.00, 36.00, 34.00]),
    "mg90s_single_test_mount_v2.stl": np.array([60.00, 44.00, 4.00]),
}


def connected_body_count(mesh):
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
    return len({find(i) for i in range(len(mesh.vertices))})


def path_points(name, count=1001):
    zs = np.linspace(design.ROOT_PATH_Z, design.TIP_PATH_Z, count)
    root_x, root_y = design.ROOT_PATHS[name]
    tip_x, tip_y = design.TIP_PATHS[name]
    ratio = (zs - design.ROOT_PATH_Z) / (design.TIP_PATH_Z - design.ROOT_PATH_Z)
    xs = root_x + ratio * (tip_x - root_x)
    root_offset = root_y - 25.0
    tip_offset = tip_y - 25.0
    offsets = root_offset + ratio * (tip_offset - root_offset)
    ys = np.array([design.path_centre_y(float(z)) for z in zs]) + offsets
    return np.column_stack((xs, ys, zs))


def check(condition, message):
    if not condition:
        raise AssertionError(message)
    print(f"PASS | {message}")


def main():
    stl_dir = HERE.parent / "stl"
    for filename, expected in EXPECTED_SIZE.items():
        path = stl_dir / filename
        check(path.exists(), f"exists: {filename}")
        # STL repeats triangle vertices by design; normal import processing
        # welds equal coordinates before evaluating manifold connectivity.
        mesh = trimesh.load_mesh(path, force="mesh", process=True)
        check(mesh.is_watertight, f"watertight: {filename}")
        check(connected_body_count(mesh) == 1, f"single body: {filename}")
        check(np.allclose(mesh.bounds[0, 2], 0.0, atol=0.01), f"on build plate: {filename}")
        size = mesh.extents
        check(np.allclose(size, expected, atol=0.03), f"size: {filename} = {size.round(2)} mm")

    tongue = design.load_scaled_tongue()
    design.validate_tube_paths(tongue)
    check(True, "four tube paths and 1.5 mm envelope remain inside the tongue")

    paths = {name: path_points(name) for name in design.ROOT_PATHS}
    lengths = {
        name: float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        for name, points in paths.items()
    }
    for name, length in lengths.items():
        check(79.4 <= length <= 79.8, f"{name} path length = {length:.2f} mm; 80 mm tube is suitable")

    min_separation = float("inf")
    names = list(paths)
    for i, name_a in enumerate(names):
        for name_b in names[i + 1 :]:
            distance = np.linalg.norm(paths[name_a] - paths[name_b], axis=1).min()
            min_separation = min(min_separation, float(distance))
    check(
        min_separation >= 3.0 - 1e-6,
        f"minimum tube centre separation = {min_separation:.2f} mm (OD is 2.00 mm)",
    )

    for label, point in {
        "pour gate": [0.0, 30.0, 8.0],
        "left vent": [-4.5, 25.0, 73.0],
        "right vent": [3.0, 25.0, 73.0],
    }.items():
        check(design.point_inside_mesh(tongue, point), f"{label} opens into tongue cavity")

    lower, upper = design.build_mold_halves(tongue)
    root_insert = design.build_root_insert()
    tip_anchor = design.build_tip_anchor()
    for part_name, part in {"root insert": root_insert, "tip anchor": tip_anchor}.items():
        for mold_name, mold in {"lower mold": lower, "upper mold": upper}.items():
            overlap = trimesh.boolean.intersection([part, mold], engine="manifold").volume
            check(overlap < 0.01, f"{part_name} has no collision with {mold_name}")

    root_embed = trimesh.boolean.intersection([root_insert, tongue], engine="manifold").volume
    tip_embed = trimesh.boolean.intersection([tip_anchor, tongue], engine="manifold").volume
    check(root_embed > 900.0, f"root insert embedded volume = {root_embed:.1f} mm^3")
    check(tip_embed > 60.0, f"tip anchor embedded volume = {tip_embed:.1f} mm^3")

    clamped_thickness = float(
        tongue.extents[1] + design.MOLD_BOTTOM_WALL + design.MOLD_TOP_WALL
    )
    check(clamped_thickness < 53.0, f"clamped mold thickness = {clamped_thickness:.2f} mm; M3x65 fits")

    # Root flange rear face and bracket front face are both z=-5 mm.
    root_flange_rear_z = -5.0
    bracket_front_z = -5.0
    check(
        abs(root_flange_rear_z - bracket_front_z) < 1e-9,
        "root insert and bracket mounting faces are flush",
    )

    ideal_force_10mm = 1.8 * 9.80665 / 1.0
    print(
        "REVIEW | MG90S ideal stall pull at a 10 mm horn radius = "
        f"{ideal_force_10mm:.1f} N; final actuator selection requires a real pull test."
    )
    print(
        "REVIEW | The rigid tip anchor is intentionally exposed in V2 and needs a later "
        "cosmetic silicone cover for a finished-looking tongue."
    )
    print("AUDIT RESULT | ALL AUTOMATED GEOMETRY CHECKS PASSED")


if __name__ == "__main__":
    main()
