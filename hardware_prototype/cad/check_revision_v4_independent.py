"""Read-only V4 verification, independent of the CAD builder's helpers.

The binary STL parsing, triangle topology, ray probes, source-to-print poses,
and sampled motion tests below do not import the V4 generator. Only the final
report is written; no input STL, project plan or primary audit is modified.
Sampled motion checks supplement (not replace) the builder's continuous sweep.
"""
from pathlib import Path
import hashlib
import json
import math
import struct
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "revision_v4"
sys.path.insert(0, str(ROOT / "tools" / "vendor"))
import manifold3d as mg

REPORT = {"method": "Independent STL topology/ray probes and sampled rigid poses",
          "limitations": ["Nominal STL geometry only, no physical assembly or seal test.",
                         "Discrete poses cannot exclude collisions between samples.",
                         "Steel extraction through the cured silicone is not verified."],
          "checks": {}, "meshes": {}}


def read(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + count * 50, "Invalid binary STL byte count: " + str(path)
    rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), offset=84)
    return rec["v"].astype(float)


def bounds(t):
    v = t.reshape(-1, 3)
    return np.array([v.min(axis=0), v.max(axis=0)])


def test(name, passed, detail):
    REPORT["checks"][name] = {"passed": bool(passed), "detail": detail}
    print(("PASS" if passed else "FAIL") + " | " + name + " | " + str(detail), flush=True)


def topology(t):
    verts, inverse = np.unique(t.reshape(-1, 3), axis=0, return_inverse=True)
    f = inverse.reshape(-1, 3)
    edges = np.vstack((f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]))
    sorted_edges = np.sort(edges, axis=1)
    unique_edges, edge_index, counts = np.unique(sorted_edges, axis=0, return_inverse=True, return_counts=True)
    signs = np.where(edges[:, 0] < edges[:, 1], 1, -1)
    balance = np.bincount(edge_index, weights=signs, minlength=len(unique_edges))
    cross = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    volumes = np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])) / 6
    return {"vertices": len(verts), "triangles": len(f), "edges": len(unique_edges),
            "unpaired_or_nonmanifold_edges": int(np.count_nonzero(counts != 2)),
            "inconsistent_directed_edges": int(np.count_nonzero(balance)),
            "degenerate_triangles": int(np.count_nonzero(np.linalg.norm(cross, axis=1) < 1e-10)),
            "signed_volume_mm3": float(volumes.sum()),
            "euler_characteristic": int(len(verts) - len(unique_edges) + len(f)),
            "bounds_mm": bounds(t).tolist()}


def rays(t, origin, direction):
    origin, direction = np.asarray(origin), np.asarray(direction)
    e1, e2 = t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]
    p = np.cross(direction, e2)
    det = np.einsum("ij,ij->i", e1, p)
    valid = np.abs(det) > 1e-10
    inv = np.divide(1., det, out=np.zeros_like(det), where=valid)
    s = origin - t[:, 0]
    u = np.einsum("ij,ij->i", s, p) * inv
    q = np.cross(s, e1)
    v = q @ direction * inv
    d = np.einsum("ij,ij->i", e2, q) * inv
    ok = valid & (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8) & (d >= 0)
    vals = np.sort(d[ok])
    return vals[np.r_[True, np.diff(vals) > 2e-5]] if len(vals) else vals


def manifold(t):
    verts, inverse = np.unique(t.reshape(-1, 3).astype(np.float32), axis=0, return_inverse=True)
    m = mg.Manifold(mg.Mesh(verts, inverse.reshape(-1, 3).astype(np.uint32)))
    assert m.status() == mg.Error.NoError, str(m.status())
    return m


def volume_overlap(a, b):
    return float(mg.Manifold.batch_boolean([a, b], mg.OpType.Intersect).volume())


def world(t, kind, ymin, ymax, zmin, zmax):
    v = t.copy()
    if kind == "upper":
        v = np.stack((t[:, :, 0], ymax - t[:, :, 2], t[:, :, 1]), axis=-1)
    elif kind == "lower":
        v = np.stack((t[:, :, 0], ymin + t[:, :, 2], -t[:, :, 1]), axis=-1)
    elif kind == "root_insert":
        v[:, :, 2] -= 5
    elif kind == "tip_anchor":
        v = np.stack((t[:, :, 0], -t[:, :, 1], 77.6 - t[:, :, 2]), axis=-1)
    elif kind == "root_end":
        v[:, :, 2] += zmin - 2
    elif kind == "tip_end":
        v = np.stack((t[:, :, 0], -t[:, :, 1], zmax + 2 - t[:, :, 2]), axis=-1)
    return v


def equal_ray_hits(a, b):
    return len(a) == len(b) and bool(np.allclose(a, b, atol=2e-4, rtol=0))


def main():
    REPORT["reused_meshes"] = {}
    reused = [ROOT / "stl" / name for name in ("tongue_mold_lower_v2.stl", "tongue_root_insert_v2.stl", "tongue_tip_anchor_v2.stl")]
    reused.append(ROOT / "revision_v3/print/tongue_root_bracket_v3.stl")
    for path in reused:
        stats = topology(read(path))
        stats["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        REPORT["reused_meshes"][path.name] = stats
        test("reused_closed_consistent_" + path.name,
             stats["unpaired_or_nonmanifold_edges"] == 0 and stats["inconsistent_directed_edges"] == 0
             and stats["degenerate_triangles"] == 0 and stats["signed_volume_mm3"] > 0,
             {"triangles": stats["triangles"], "unpaired_or_nonmanifold_edges": stats["unpaired_or_nonmanifold_edges"],
              "inconsistent_directed_edges": stats["inconsistent_directed_edges"], "degenerate_triangles": stats["degenerate_triangles"]})
    old_up_p = read(ROOT / "stl/tongue_mold_upper_v2.stl")
    old_lo_p = read(ROOT / "stl/tongue_mold_lower_v2.stl")
    ymax = 17.2 + float(np.ptp(old_up_p[:, :, 2]))
    ymin = 17.2 - (float(np.ptp(old_lo_p[:, :, 2])) - 4.)
    zmin, zmax = bounds(old_up_p)[:, 1]
    old_upper = world(old_up_p, "upper", ymin, ymax, zmin, zmax)
    old_lower = world(old_lo_p, "lower", ymin, ymax, zmin, zmax)
    meshes = {}
    for path in sorted((OUT / "print").glob("*.stl")):
        t = read(path)
        stats = topology(t)
        stats["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        REPORT["meshes"][path.name] = stats
        test("closed_consistent_" + path.name,
             stats["unpaired_or_nonmanifold_edges"] == 0 and stats["inconsistent_directed_edges"] == 0
             and stats["degenerate_triangles"] == 0 and stats["signed_volume_mm3"] > 0,
             stats)
        test("print_floor_" + path.name, abs(stats["bounds_mm"][0][2]) < 1e-4, stats["bounds_mm"])
        kind = "upper" if "upper" in path.name else "lower" if "lower" in path.name else "root_end" if "root_end" in path.name else "tip_end"
        meshes[kind] = world(t, kind, ymin, ymax, zmin, zmax)
    if "lower" not in meshes:
        meshes["lower"] = old_lower
    upper, lower = meshes["upper"], meshes["lower"]
    # Six actual legacy M3 centers; ring rays check the bearing rim was not cut.
    xs = [float(bounds(old_upper)[0, 0] + 5), float(bounds(old_upper)[1, 0] - 5)]
    zs = [zmin + 5, (zmin + zmax) / 2, zmax - 5]
    bolt_records = []
    for x in xs:
        for z in zs:
            entry = {"x": x, "z": z, "bores_open": True, "rims_unchanged": True}
            for label, old_t, new_t in (("upper", old_upper, upper), ("lower", old_lower, lower)):
                # Interior ray sampling also detects a mistakenly filled bore.
                for dx, dz in ((0, 0), (.8, 0), (0, .8), (-.8, 0), (0, -.8)):
                    hits = rays(new_t, [x + dx, ymin - 1, z + dz], [0, 1, 0])
                    entry["bores_open"] &= len(hits) == 0
                for angle in np.linspace(0, 2 * math.pi, 9)[:-1]:
                    origin = [x + 2.7 * math.cos(angle), ymin - 1, z + 2.7 * math.sin(angle)]
                    entry["rims_unchanged"] &= equal_ray_hits(rays(old_t, origin, [0, 1, 0]), rays(new_t, origin, [0, 1, 0]))
            bolt_records.append(entry)
    test("six_M3_bores_and_bearing_rims", all(r["bores_open"] and r["rims_unchanged"] for r in bolt_records), bolt_records)
    pins = []
    for x in (-24., 24.):
        for z in (zmin + 5, zmax - 5):
            entry = {"x": x, "z": z, "pin_and_socket_unchanged": True}
            for dx, dz in ((0, 0), (.8, 0), (0, .8), (2.7, 0), (-2.7, 0), (0, 2.7), (0, -2.7)):
                origin = [x + dx, ymin - 1, z + dz]
                for old_t, new_t in ((old_upper, upper), (old_lower, lower)):
                    entry["pin_and_socket_unchanged"] &= equal_ray_hits(rays(old_t, origin, [0, 1, 0]), rays(new_t, origin, [0, 1, 0]))
            pins.append(entry)
    test("four_registration_pins_and_sockets", all(p["pin_and_socket_unchanged"] for p in pins), pins)
    ports = []
    for name, x, z, radius in (("pour", 0., 8., 4.5), ("vent_1", -4.5, 73., 1.), ("vent_2", 3., 73., 1.)):
        intervals = []
        for dx, dz in ((0, 0), (.4 * radius, 0), (-.4 * radius, 0), (0, .4 * radius), (0, -.4 * radius)):
            origin = [x + dx, ymax + 1, z + dz]
            hits = rays(upper, origin, [0, -1, 0])
            first_y = None if not len(hits) else float(ymax + 1 - hits[0])
            intervals.append(first_y)
        # At least the original axial port remains open down to its declared base.
        base = 30 if name == "pour" else 25
        passed = all(y is None or y <= base + 1e-3 for y in intervals)
        ports.append({"name": name, "first_material_y_mm": intervals, "passed": passed})
    test("pour_and_two_vent_axial_passages", all(p["passed"] for p in ports), ports)
    root_paths = {"LEFT": (-6., 25.), "RIGHT": (6., 25.), "DOWN": (0., 21.5), "UP": (0., 28.5)}
    tip_paths = {"LEFT": (-4.5, 25.), "RIGHT": (1.5, 25.), "DOWN": (-1.5, 23.5), "UP": (-1.5, 26.5)}
    end_holes = []
    for which, coordinates in (("root_end", root_paths), ("tip_end", tip_paths)):
        t = meshes[which]
        for name, (x, y) in coordinates.items():
            open_all = True
            for dx, dy in ((0, 0), (.49, 0), (-.49, 0), (0, .49), (0, -.49)):
                open_all &= len(rays(t, [x + dx, y + dy, zmin - 30], [0, 0, 1])) == 0
            end_holes.append({"end": which, "route": name, "diameter_1mm_clear": open_all})
    test("eight_core_endcap_bores", all(p["diameter_1mm_clear"] for p in end_holes), end_holes)
    # Target solids come from unchanged legacy print files or released V4 prints.
    fixed = {"root_insert": world(read(ROOT / "stl/tongue_root_insert_v2.stl"), "root_insert", ymin, ymax, zmin, zmax),
             "tip_anchor": world(read(ROOT / "stl/tongue_tip_anchor_v2.stl"), "tip_anchor", ymin, ymax, zmin, zmax),
             "root_end": meshes["root_end"], "tip_end": meshes["tip_end"]}
    # Curved tubes/cores are checked from released reference surfaces, not from
    # the builder's loading cutters or any Minkowski-envelope helper.
    for path in sorted((OUT / "reference").glob("*.stl")):
        if path.stem.startswith(("tube_", "core_")):
            fixed[path.stem] = read(path)
    group_triangles = np.concatenate(list(fixed.values()))
    pour_clear = []
    # The central UP tube crosses the center of the large pour port. Verify
    # the specified off-center pouring lane with the assembly actually present.
    for angle in np.linspace(0, 2 * math.pi, 17)[:-1]:
        x, z = 2.5 + .95 * math.cos(angle), 8. + .95 * math.sin(angle)
        origin = [x, ymax + 1, z]
        hits = rays(np.concatenate((upper, group_triangles)), origin, [0, -1, 0])
        first_y = None if not len(hits) else float(ymax + 1 - hits[0])
        pour_clear.append(first_y)
    test("assembled_off_center_pour_lane", all(y is None or y <= 26.001 for y in pour_clear),
         {"center_x_z_mm": [2.5, 8.], "sample_radius_mm": .95, "first_material_y_mm": pour_clear})
    up_m, lo_m = manifold(upper), manifold(lower)
    motions = []
    shifts = np.unique(np.r_[np.linspace(-45, -10, 8), np.linspace(-10, 0, 41)])
    for name, t in fixed.items():
        part = manifold(t)
        upper_max = lower_max = 0.
        worst_up = worst_low = None
        for d in shifts:
            # Move the insert toward the fixed upper half; separately move the
            # lower mold from above the fixed insert (same world -Y direction).
            u = volume_overlap(up_m, part.translate((0, float(d), 0)))
            l = volume_overlap(lo_m.translate((0, float(d), 0)), part)
            if u > upper_max:
                upper_max, worst_up = u, float(d)
            if l > lower_max:
                lower_max, worst_low = l, float(d)
        motions.append({"part": name, "upper_max_overlap_mm3": upper_max, "lower_max_overlap_mm3": lower_max,
                        "upper_worst_shift_mm": worst_up, "lower_worst_shift_mm": worst_low})
    test("sampled_loading_and_lower_closure", all(max(p["upper_max_overlap_mm3"], p["lower_max_overlap_mm3"]) < 1e-4 for p in motions),
         {"poses_per_part": len(shifts), "results": motions})
    mold_overlaps = [volume_overlap(up_m, lo_m.translate((0, float(d), 0))) for d in shifts]
    test("sampled_mold_half_closure", max(mold_overlaps) < 1e-4,
         {"poses": len(shifts), "max_intersection_mm3": max(mold_overlaps)})
    cores = mg.Manifold.batch_boolean([manifold(t) for name, t in fixed.items() if name.startswith("core_")], mg.OpType.Add)
    tubes = mg.Manifold.batch_boolean([manifold(t) for name, t in fixed.items() if name.startswith("tube_")], mg.OpType.Add)
    sliding = []
    for name, outside in (("root_insert", -24.), ("tip_anchor", 20.), ("root_end", -12.), ("tip_end", 17.)):
        part = manifold(fixed[name])
        values = []
        for dz in np.linspace(outside, 0, 49):
            moved = part.translate((0, 0, float(dz)))
            values.append(max(volume_overlap(moved, cores), volume_overlap(moved, tubes)))
        sliding.append({"part": name, "maximum_overlap_mm3": max(values)})
    test("sampled_prethreading_from_straight_tails", max(p["maximum_overlap_mm3"] for p in sliding) < 1e-4,
         {"poses_per_part": 49, "results": sliding})
    # Actual upper access: directed rays from outside must not hit plastic before
    # reaching the top of formerly trapped rigid pockets.
    access = []
    for name, x, z, target_y in (("root_neck", .5, .75, 20.), ("tip_center", -1.5, 76.4, 21.25),
                                  ("tip_left", -7., 76.4, 21.25), ("tip_right", 4., 76.4, 21.25)):
        hits = rays(upper, [x, 0., z], [0, 1, 0])
        first = None if not len(hits) else float(hits[0])
        passed = first is None or first > target_y + .05
        access.append({"point": name, "first_material_y_mm": first, "target_top_y_mm": target_y, "passed": passed})
    test("formerly_closed_roofs_now_accessible", all(a["passed"] for a in access), access)
    REPORT["summary"] = {"passed": all(c["passed"] for c in REPORT["checks"].values()), "number_of_checks": len(REPORT["checks"])}
    target = OUT / "audit/V4_INDEPENDENT_CHECK.json"
    target.write_text(json.dumps(REPORT, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("REPORT | " + str(target), flush=True)
    if not REPORT["summary"]["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
