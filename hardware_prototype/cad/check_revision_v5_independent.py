"""Independently inspect the released inverted-pour V5 STL bytes.

The V5 generator is deliberately not imported. Binary parsing, directed-edge
topology, print/world transforms, legacy-feature probes, and protected-zone
Boolean comparisons are implemented here. Only this checker's report is
written. Passing checks do not establish bubble-free filling or real print fit.
"""
from pathlib import Path
import hashlib
import json
import math
import struct
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "revision_v5"
sys.path.insert(0, str(ROOT / "tools/vendor"))
import manifold3d as mg

REPORT = {
    "method": "Independent released-STL topology, transforms, rays, Boolean regression and fit",
    "limits": [
        "Nominal rigid geometry only; print shrinkage and silicone behavior are not measured.",
        "No flow simulation, physical fill, sealing, cure or bubble-free performance validation.",
        "Sampled assembly poses do not replace a full physical dry-fit cycle.",
        "The four long curved reference cores are historical geometry, not an instruction to reuse stuck cores.",
    ],
    "checks": {},
    "meshes": {},
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    data = path.read_bytes()
    assert len(data) >= 84, str(path)
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * count, "Invalid binary STL size: " + str(path)
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    return np.frombuffer(data, dtype=dtype, count=count, offset=84)["vertices"].astype(float)


def bounds(triangles):
    points = triangles.reshape(-1, 3)
    return np.array((points.min(axis=0), points.max(axis=0)))


def check(name, passed, detail=None):
    REPORT["checks"][name] = {"passed": bool(passed), "detail": detail}
    print(("PASS" if passed else "FAIL") + " | " + name + " | " + str(detail), flush=True)


def topology(triangles):
    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    directed = np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    edges, indices, counts = np.unique(np.sort(directed, axis=1), axis=0, return_inverse=True, return_counts=True)
    signs = np.where(directed[:, 0] < directed[:, 1], 1, -1)
    balance = np.bincount(indices, weights=signs, minlength=len(edges))
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    signed_volume = np.einsum("ij,ij->i", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum() / 6
    parent = np.arange(len(vertices))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra
    return {
        "vertices": len(vertices), "triangles": len(faces),
        "unpaired_or_nonmanifold_edges": int(np.count_nonzero(counts != 2)),
        "inconsistent_directed_edges": int(np.count_nonzero(balance)),
        "degenerate_triangles": int(np.count_nonzero(np.linalg.norm(cross, axis=1) < 1e-10)),
        "connected_components": len({find(i) for i in range(len(vertices))}),
        "signed_volume_mm3": float(signed_volume), "bounds_mm": bounds(triangles).tolist(),
    }


def solid(triangles):
    vertices, inverse = np.unique(triangles.reshape(-1, 3).astype(np.float32), axis=0, return_inverse=True)
    result = mg.Manifold(mg.Mesh(vertices, inverse.reshape(-1, 3).astype(np.uint32)))
    assert result.status() == mg.Error.NoError, str(result.status())
    return result


def union(items):
    return mg.Manifold.batch_boolean(list(items), mg.OpType.Add)


def subtract(a, b):
    return mg.Manifold.batch_boolean([a, b], mg.OpType.Subtract)


def overlap(a, b):
    return float(mg.Manifold.batch_boolean([a, b], mg.OpType.Intersect).volume())


def ray(triangles, origin, direction):
    origin, direction = np.asarray(origin), np.asarray(direction)
    e1 = triangles[:, 1] - triangles[:, 0]
    e2 = triangles[:, 2] - triangles[:, 0]
    p = np.cross(direction, e2)
    determinant = np.einsum("ij,ij->i", e1, p)
    valid = np.abs(determinant) > 1e-10
    inverse = np.divide(1., determinant, out=np.zeros_like(determinant), where=valid)
    delta = origin - triangles[:, 0]
    u = np.einsum("ij,ij->i", delta, p) * inverse
    q = np.cross(delta, e1)
    v = q @ direction * inverse
    distance = np.einsum("ij,ij->i", e2, q) * inverse
    good = valid & (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8) & (distance >= 0)
    values = np.sort(distance[good])
    return values[np.r_[True, np.diff(values) > 2e-5]] if len(values) else values


def equal_rays(a, b, tolerance=3e-4):
    return len(a) == len(b) and bool(np.allclose(a, b, atol=tolerance, rtol=0))


def old_pose(triangles, kind, coordinates):
    t = triangles
    if kind == "upper":
        return np.stack((t[:, :, 0], coordinates["y_max"] - t[:, :, 2], t[:, :, 1]), axis=-1)
    if kind == "lower":
        return np.stack((t[:, :, 0], coordinates["y_min"] + t[:, :, 2], -t[:, :, 1]), axis=-1)
    if kind == "root_insert":
        result = t.copy()
        result[:, :, 2] -= 5
        return result
    if kind == "tip_anchor":
        return np.stack((t[:, :, 0], -t[:, :, 1], 77.6 - t[:, :, 2]), axis=-1)
    if kind == "root_end":
        result = t.copy()
        result[:, :, 2] += coordinates["z_min"] - 2
        return result
    if kind == "tip_end":
        return np.stack((t[:, :, 0], -t[:, :, 1], coordinates["z_max"] + 2 - t[:, :, 2]), axis=-1)
    raise ValueError(kind)


def cylinder_mask(x, z, radius, ymin, ymax):
    return mg.Manifold.cylinder(ymax - ymin, radius, circular_segments=96).transform(
        ((1, 0, 0, x), (0, 0, 1, ymin), (0, -1, 0, z)))


def original_cavity():
    vertices, faces = [], []
    path = ROOT.parent / "assets/mesh/tongue.obj"
    for line in path.read_text(encoding="utf-8").splitlines():
        words = line.split()
        if not words:
            continue
        if words[0] == "v":
            vertices.append([float(value) for value in words[1:4]])
        elif words[0] == "f":
            f = [int(value.split("/")[0]) - 1 for value in words[1:]]
            faces.extend((f[0], f[i], f[i + 1]) for i in range(1, len(f) - 1))
    points = np.asarray(vertices)
    xmid = (points[:, 0].min() + points[:, 0].max()) / 2
    ymin, zroot = points[:, 1].min(), points[:, 2].max()
    points = np.column_stack(((points[:, 0] - xmid) * 25,
                              (points[:, 1] - ymin) * 25,
                              (zroot - points[:, 2]) * 25))
    center = (points.min(axis=0) + points.max(axis=0)) / 2
    points = (points - center) * 1.004 + center
    triangles = points[np.asarray(faces)]
    if np.einsum("ij,ij->i", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum() < 0:
        triangles = triangles[:, [0, 2, 1]]
    return solid(triangles)


def main():
    primary = json.loads((OUT / "audit/V5_GEOMETRY_AUDIT.json").read_text(encoding="utf-8"))
    c = primary["coordinates"]
    old_a = read(ROOT / "revision_v4/reference/upper_world.stl")
    old_b = read(ROOT / "revision_v4/reference/lower_world.stl")
    names = ("tongue_mold_lower_inverted_v5.stl", "tongue_mold_upper_inverted_v5.stl")
    check("exactly_two_print_STLs", {p.name for p in (OUT / "print").glob("*.stl")} == set(names), list(names))
    released = {}
    for name, kind in zip(names, ("upper", "lower")):
        path = OUT / "print" / name
        t = read(path)
        stats = topology(t)
        stats["sha256"] = sha(path)
        REPORT["meshes"][name] = stats
        check("topology_" + name,
              stats["unpaired_or_nonmanifold_edges"] == 0 and stats["inconsistent_directed_edges"] == 0
              and stats["degenerate_triangles"] == 0 and stats["connected_components"] == 1
              and stats["signed_volume_mm3"] > 0, stats)
        check("bed_Z0_" + name, abs(stats["bounds_mm"][0][2]) < 1e-4, stats["bounds_mm"])
        released[kind] = old_pose(t, kind, c)
    new_a, new_b = released["upper"], released["lower"]
    a, b, oa, ob = map(solid, (new_a, new_b, old_a, old_b))
    for label, actual, path in (("new_lower", a, OUT / "reference/new_lower_world.stl"),
                                ("new_upper", b, OUT / "reference/new_upper_world.stl")):
        reference = solid(read(path))
        error = float(subtract(actual, reference).volume() + subtract(reference, actual).volume())
        check("print_to_world_" + label, error < .025, {"symmetric_difference_mm3": error})

    # The permitted edit masks are reconstructed as vertical cylinders; no
    # loading cutter, repair, or guard geometry is imported from the generator.
    all_old = primary["old_ports"]
    all_new = primary["new_ports"]
    old_mask = union(cylinder_mask(p["x"], p["z"], p["diameter_mm"] / 2 + .002,
                                  c["y_min"] - 3, c["y_max"] + 3) for p in all_old)
    new_mask = union(cylinder_mask(p["x"], p["z"], p["diameter_mm"] / 2 + .002,
                                  c["y_min"] - 3, c["y_max"] + 3) for p in all_new)
    # Fine float32 round-tripping may create tiny boundary wedges; use a
    # reported absolute-volume tolerance instead of silently repairing meshes.
    for label, old, new, mask in (("old_upper_now_lower", oa, a, old_mask),
                                  ("old_lower_now_upper", ob, b, new_mask)):
        added, removed = subtract(new, old), subtract(old, new)
        error = float(subtract(added, mask).volume() + subtract(removed, mask).volume())
        check("only_declared_ports_changed_" + label, error < .025,
              {"outside_port_masks_mm3": error, "added_mm3": float(added.volume()),
               "removed_mm3": float(removed.volume())})
    cavity_overlap = overlap(subtract(a, oa), original_cavity())
    check("old_hole_fill_not_inside_original_tongue", cavity_overlap < .025,
          {"added_plastic_inside_original_cavity_mm3": cavity_overlap})

    x_positions = [float(bounds(old_a)[0, 0] + 5), float(bounds(old_a)[1, 0] - 5)]
    z_positions = [c["z_min"] + 5, (c["z_min"] + c["z_max"]) / 2, c["z_max"] - 5]
    records = []
    for x in x_positions:
        for z in z_positions:
            record = {"x": x, "z": z, "open": True, "rim_preserved": True}
            for old, new in ((old_a, new_a), (old_b, new_b)):
                for dx, dz in ((0, 0), (.8, 0), (-.8, 0), (0, .8), (0, -.8)):
                    record["open"] &= not len(ray(new, [x + dx, c["y_min"] - 1, z + dz], [0, 1, 0]))
                for angle in np.arange(8) * math.pi / 4:
                    origin = [x + 2.7 * math.cos(angle), c["y_min"] - 1, z + 2.7 * math.sin(angle)]
                    record["rim_preserved"] &= equal_rays(ray(old, origin, [0, 1, 0]), ray(new, origin, [0, 1, 0]))
            records.append(record)
    check("six_bolts_and_bearing_rims_preserved", all(r["open"] and r["rim_preserved"] for r in records), records)
    records = []
    for x in (-24., 24.):
        for z in (c["z_min"] + 5, c["z_max"] - 5):
            passed = True
            for old, new in ((old_a, new_a), (old_b, new_b)):
                for dx, dz in ((0, 0), (.8, 0), (0, .8), (2.7, 0), (-2.7, 0), (0, 2.7)):
                    origin = [x + dx, c["y_min"] - 1, z + dz]
                    passed &= equal_rays(ray(old, origin, [0, 1, 0]), ray(new, origin, [0, 1, 0]))
            records.append({"x": x, "z": z, "preserved": passed})
    check("four_registration_features_preserved", all(r["preserved"] for r in records), records)

    old_closure = []
    for p in all_old:
        firsts = []
        for dx, dz in ((0, 0), (.25 * p["diameter_mm"], 0), (0, .25 * p["diameter_mm"])):
            hits = ray(new_a, [p["x"] + dx, c["y_max"] + 1, p["z"] + dz], [0, -1, 0])
            firsts.append(None if not len(hits) else float(hits[0]))
        passed = all(v is not None and abs(v - 1) < 2e-4 for v in firsts)
        old_closure.append({"name": p["name"], "first_hit_from_outer_plus_1mm": firsts, "closed": passed})
    check("legacy_three_outer_openings_closed", all(p["closed"] for p in old_closure), old_closure)
    port_records = []
    for p in all_new:
        firsts = []
        for dx, dz in ((0, 0), (.2 * p["diameter_mm"], 0), (-.2 * p["diameter_mm"], 0),
                       (0, .2 * p["diameter_mm"]), (0, -.2 * p["diameter_mm"])):
            origin = [p["x"] + dx, c["y_min"] - 1, p["z"] + dz]
            hits = ray(np.concatenate((new_a, new_b)), origin, [0, 1, 0])
            firsts.append(None if not len(hits) else float(c["y_min"] - 1 + hits[0]))
        # The first obstruction must lie beyond the intended inner channel end,
        # proving these are open-to-cavity ports rather than blind outside holes.
        passed = all(y is None or y >= p["inner_y"] - 3e-4 for y in firsts)
        port_records.append({"name": p["name"], "expected_inner_y": p["inner_y"],
                             "first_material_y_mm": firsts, "connected_to_cavity": passed})
    check("new_pour_and_vents_open_to_cavity", all(p["connected_to_cavity"] for p in port_records), port_records)

    fixed = {
        "root_insert": old_pose(read(ROOT / "stl/tongue_root_insert_v2.stl"), "root_insert", c),
        "tip_anchor": old_pose(read(ROOT / "stl/tongue_tip_anchor_v2.stl"), "tip_anchor", c),
        "root_end": old_pose(read(ROOT / "revision_v4/print/tongue_root_end_dam_v4.stl"), "root_end", c),
        "tip_end": old_pose(read(ROOT / "revision_v4/print/tongue_tip_end_dam_v4.stl"), "tip_end", c),
    }
    assembly = []
    shifts = (-45., -30., -15., -8., -4., -2., -1., -.5, 0.)
    for name, triangles in fixed.items():
        part = solid(triangles)
        entry = {"part": name, "new_lower_loading_max_mm3": 0., "new_upper_closing_max_mm3": 0.}
        for distance in shifts:
            # Historical assembly direction is unchanged even though casting
            # top/bottom names are reversed: parts load from world -Y.
            entry["new_lower_loading_max_mm3"] = max(entry["new_lower_loading_max_mm3"], overlap(a, part.translate((0, distance, 0))))
            entry["new_upper_closing_max_mm3"] = max(entry["new_upper_closing_max_mm3"], overlap(b.translate((0, distance, 0)), part))
        assembly.append(entry)
    check("four_reused_rigid_parts_sampled_fit",
          all(max(e["new_lower_loading_max_mm3"], e["new_upper_closing_max_mm3"]) < .025 for e in assembly),
          {"world_negative_Y_samples_mm": list(shifts), "parts": assembly})
    mold_overlap = [overlap(a, b.translate((0, distance, 0))) for distance in shifts]
    check("sampled_halves_close_without_added_interference", max(mold_overlap) < .025,
          {"samples_mm": list(shifts), "maximum_intersection_mm3": max(mold_overlap)})

    REPORT["primary_report_sha256"] = sha(OUT / "audit/V5_GEOMETRY_AUDIT.json")
    REPORT["summary"] = {"passed": all(v["passed"] for v in REPORT["checks"].values()),
                         "number_of_checks": len(REPORT["checks"])}
    path = OUT / "audit/V5_INDEPENDENT_CHECK.json"
    path.write_text(json.dumps(REPORT, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("REPORT | " + str(path), flush=True)
    if not REPORT["summary"]["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
