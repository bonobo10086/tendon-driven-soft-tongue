"""V5: invert casting orientation, changing ONLY feed/vent passages.

Read-only baselines are the released V4 upper and V2 lower.  Legacy names
describe anatomy, V5 names describe their position DURING CASTING:
new upper = old lower (four male pins); new lower = old upper (four sockets).
All assembly geometry and existing physical accessories remain unchanged.
No physical fill, seal, silicone adhesion, or strength claim is made.
"""
from pathlib import Path
import hashlib
import json
import math
import struct
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "revision_v5"
sys.path.insert(0, str(ROOT / "tools" / "vendor"))
import manifold3d as mg
import build_printables as original
import build_revision_v4 as v4
from render_assembly_guide_v3 import Mesh, read_mesh, load_world
from check_revision_v4_independent import rays, topology

UPPER_NAME = "tongue_mold_upper_inverted_v5.stl"
LOWER_NAME = "tongue_mold_lower_inverted_v5.stl"
SPLIT = 17.2
OLD_PORTS = [
    {"name": "old_root_pour", "x": 0., "z": 8., "diameter_mm": 9., "inner_y": 30.},
    {"name": "old_tip_vent_left", "x": -4.5, "z": 73., "diameter_mm": 2., "inner_y": 25.},
    {"name": "old_tip_vent_right", "x": 3., "z": 73., "diameter_mm": 2., "inner_y": 25.},
]
NEW_PORTS = [
    # Actual absolute minimum-Y belly vertex, not the symmetric X=0 axis.
    {"name": "pour", "label_cn": "灌注口", "x": 9.7100033975,
     "z": 36.6906680965, "diameter_mm": 9.},
    {"name": "vent_opposite", "label_cn": "另一侧深凹排气孔", "x": -8.2,
     "z": 38., "diameter_mm": 2.},
    {"name": "vent_root", "label_cn": "靠舌根排气孔", "x": 0.,
     "z": 12., "diameter_mm": 2.},
    {"name": "vent_tip", "label_cn": "靠舌尖排气孔", "x": -2.,
     "z": 64., "diameter_mm": 2.},
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def record(report, key, passed, detail=None):
    report["checks"][key] = {"passed": bool(passed), "detail": detail}
    print(("PASS" if passed else "FAIL") + " | " + key + " | " + str(detail), flush=True)
    if not passed:
        raise AssertionError(key)


def y_cylinder(x, z, y0, y1, diameter):
    return v4.solid(Mesh(original.cylinder_between(
        [x, y0, z], [x, y1, z], diameter / 2., sections=64).triangles))


def write_stl(path, part):
    triangles = v4.mesh_of(part).triangles.astype(np.float32)
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-20)
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    rows = np.zeros(len(triangles), dtype=dtype)
    rows["normal"], rows["vertices"] = normals, triangles
    header = b"V5 inverted tongue mold; units mm; casting orientation != print orientation"
    path.write_bytes(header.ljust(80, b" ")[:80]+struct.pack("<I", len(rows))+rows.tobytes())


def negative_loading_clearance(part):
    """Reuse read-only V4 continuous clearance caches; never write into V4."""
    pad, distance, axis, direction = .20, 45., 1, -1
    data = part.to_mesh()
    key = hashlib.sha256(np.asarray(data.vert_properties).tobytes()
        + np.asarray(data.tri_verts).tobytes()
        + str((axis, direction, distance, pad)).encode()).hexdigest()
    for folder in (ROOT/"revision_v4/audit/cache", OUT/"audit/cache"):
        candidate = folder/(key+".npz")
        if candidate.exists():
            with np.load(candidate) as saved:
                value = mg.Manifold(mg.Mesh(saved["vertices"], saved["faces"]))
            assert value.status() == mg.Error.NoError
            return value
    lo, hi = np.full(3, -pad), np.full(3, pad)
    lo[1] -= distance
    value = part.minkowski_sum(v4.cube(lo, hi))
    data = value.to_mesh()
    np.savez_compressed(OUT/"audit/cache"/(key+".npz"),
                        vertices=np.asarray(data.vert_properties), faces=np.asarray(data.tri_verts))
    return value


def main():
    started = time.time()
    for sub in ("print", "reference", "audit/cache", "preview"):
        (OUT/sub).mkdir(parents=True, exist_ok=True)
    protected = [ROOT/"BOM_CN.md", *sorted((ROOT/"stl").glob("*.stl")),
                 *sorted((ROOT/"revision_v3/print").glob("*.stl")),
                 *sorted((ROOT/"revision_v4/print").glob("*.stl")),
                 *sorted((ROOT/"revision_v4/reference").glob("*.stl")),
                 HERE/"build_printables.py", HERE/"build_revision_v4.py"]
    before = {str(p.relative_to(ROOT)): digest(p) for p in protected}
    legacy = json.loads((ROOT/"revision_v4/audit/V4_GEOMETRY_AUDIT.json").read_text(encoding="utf-8"))
    coordinates = legacy["coordinates"]
    ymin, ymax = coordinates["y_min"], coordinates["y_max"]
    zmin, zmax = coordinates["z_min"], coordinates["z_max"]
    old_upper_mesh = read_mesh(ROOT/"revision_v4/reference/upper_world.stl")
    old_lower_mesh = read_mesh(ROOT/"revision_v4/reference/lower_world.stl")
    old_upper, old_lower = v4.solid(old_upper_mesh), v4.solid(old_lower_mesh)
    source_tongue = original.load_scaled_tongue()
    tongue = v4.solid(Mesh(source_tongue.triangles))
    items = {name: v4.solid(read_mesh(ROOT/"revision_v4/reference"/(name+".stl")))
             for name in ("root_insert_v2", "tip_anchor_v2", "root_end_dam_v4", "tip_end_dam_v4")}
    report = {"release": "V5_INVERTED", "units": "mm", "coordinates": coordinates,
              "casting_up_world": [0, -1, 0], "checks": {}, "parts": {},
              "old_ports": OLD_PORTS, "new_ports": NEW_PORTS,
              "role_mapping": {UPPER_NAME: "old V2 lower, retains male pins",
                               LOWER_NAME: "old V4 upper, retains sockets and open loading slots"},
              "limits": ["Nominal mesh geometry, not physical fill/seal/cure/strength validation.",
                         "Top gravity pouring can still trap air; vents are not a void-free guarantee.",
                         "Flexible tube movement and the existing incomplete specimen are not validated.",
                         "Only feed/vent passages change; old dimensions, cavity and accessories are retained."]}
    record(report, "V4_baseline_print_sha_matches_original_release",
           digest(ROOT/"revision_v4/print/tongue_mold_upper_v4.stl")
           == legacy["parts"]["tongue_mold_upper_v4.stl"]["sha256"])
    record(report, "V2_lower_matches_reused_reference",
           abs(v4.solid(load_world("tongue_mold_lower_v2.stl", ymin, ymax)).volume()
               - old_lower.volume()) < .02)

    # Restore ONLY old port cylinders. Clip their filler to the original solid
    # stock, subtract the original cavity, insert seats and V4 assembly sweeps.
    # Blindly plugging whole cylinders would distort the cast and trap inserts.
    bb = old_upper.bounding_box()
    stock = v4.cube(bb[:3], bb[3:])
    old_port_zones = [y_cylinder(p["x"], p["z"], p["inner_y"], ymax+1,
                                  p["diameter_mm"]+.04) for p in OLD_PORTS]
    root_pocket = v4.union((v4.solid(Mesh(original.box([24.4,15.4,3.4],[.5,25,5]).triangles)),
                           v4.solid(Mesh(original.box([12.4,10.4,6.9],[.5,25,.75]).triangles)),
                           v4.solid(Mesh(original.box([38.4,19.4,3.4],[.5,25,-3.5]).triangles))))
    tip_pocket = v4.solid(Mesh(original.box([13.4,7.9,2.8],[-1.5,25,76.4]).triangles))
    load_sweeps = {name: negative_loading_clearance(part) for name, part in items.items()}
    # Tube/rod sweeps also retain any loading channel overlapping a former port.
    port_bounds = [np.asarray(p.bounding_box()) for p in old_port_zones]
    for path in sorted((ROOT/"revision_v4/reference").glob("*.stl")):
        if not path.stem.startswith(("tube_", "core_")):
            continue
        part = v4.solid(read_mesh(path))
        part_bb = np.asarray(part.bounding_box())
        swept_lo, swept_hi = part_bb[:3]-.20, part_bb[3:]+.20
        swept_lo[1] -= 45.
        if any(np.all(swept_hi >= b[:3]) and np.all(b[3:] >= swept_lo) for b in port_bounds):
            load_sweeps[path.stem] = negative_loading_clearance(part)
    fill = v4.difference(v4.intersection(stock, v4.union(old_port_zones)),
                         tongue, root_pocket, tip_pocket, *load_sweeps.values())
    new_lower = v4.union((old_upper, fill))
    added = v4.difference(new_lower, old_upper)
    removed = v4.difference(old_upper, new_lower)
    record(report, "new_lower_only_adds_material_in_old_port_zones",
           removed.volume() < .01 and v4.difference(added, v4.union(old_port_zones)).volume() < .01,
           {"added_mm3": added.volume(), "unexpected_removed_mm3": removed.volume()})
    record(report, "port_fill_does_not_invade_original_tongue_cavity",
           v4.volume_intersection(fill, tongue) < .01)
    for name, sweep in load_sweeps.items():
        record(report, "old_loading_clearance_preserved_"+name,
               v4.volume_intersection(fill, sweep) < .01)

    cutters = []
    for p in NEW_PORTS:
        hits = rays(source_tongue.triangles, [p["x"], ymin-1, p["z"]], [0,1,0])
        assert len(hits) >= 2, p
        surface_y = ymin-1+float(hits[0])
        p["cavity_roof_y_mm"] = surface_y
        p["inner_y"] = surface_y+.60
        p["outer_y"] = ymin
        # All new passages open into the old lower half's cavity. They are not
        # blind holes ending at a solid patch of the other half.
        record(report, "port_enters_cavity_before_split_"+p["name"],
               surface_y < SPLIT-.5, {"roof_y": surface_y, "split_y": SPLIT})
        cutter = y_cylinder(p["x"], p["z"], ymin-1, p["inner_y"], p["diameter_mm"])
        cutters.append(cutter)
        record(report, "port_axis_not_through_insert_or_dam_"+p["name"],
               max(v4.volume_intersection(cutter, item) for item in items.values()) < .01)
    new_upper = v4.difference(old_lower, *cutters)
    changed_upper = v4.difference(old_lower, new_upper)
    record(report, "new_upper_only_removes_new_port_material",
           v4.difference(new_upper, old_lower).volume() < .01
           and v4.difference(changed_upper, v4.union(cutters)).volume() < .01)
    min_vertex = source_tongue.vertices[np.argmin(source_tongue.vertices[:,1])]
    record(report, "pour_center_exactly_over_deepest_belly_point",
           np.linalg.norm(min_vertex[[0,2]]-np.array([NEW_PORTS[0]["x"],NEW_PORTS[0]["z"]])) < 1e-6,
           {"point_world_mm": min_vertex.tolist(), "definition": "absolute minimum world-Y of original cavity"})

    # Final 32-bit STL geometry is reloaded before every delivery audit.
    for name, part, old_role in ((UPPER_NAME, new_upper, "lower"),
                                 (LOWER_NAME, new_lower, "upper")):
        printable = v4.world_to_print(part, old_role, ymin, ymax, zmin, zmax)
        # As in the released V4 builder, eliminate sub-micron coplanar slivers
        # before binary STL rounding; never hand over nonmanifold triangles.
        printable = printable.simplify(.001)
        rounded_mesh = Mesh(v4.mesh_of(printable).triangles.astype(np.float32))
        printable = v4.solid(rounded_mesh).simplify(.001)
        path = OUT/"print"/name
        write_stl(path, printable)
        actual_mesh = read_mesh(path)
        actual = v4.solid(actual_mesh)
        stats = topology(actual_mesh.triangles)
        record(report, "binary_stl_closed_and_consistent_"+name,
               stats["unpaired_or_nonmanifold_edges"] == 0
               and stats["inconsistent_directed_edges"] == 0
               and stats["degenerate_triangles"] == 0
               and stats["signed_volume_mm3"] > 0, stats)
        record(report, "one_solid_body_"+name, len(actual.decompose()) == 1)
        record(report, "flat_back_on_print_bed_"+name, abs(actual_mesh.bounds[0,2]) < 1e-4)
        report["parts"][name] = {"quantity": 1, "dimensions_mm": actual_mesh.extents.tolist(),
                                "volume_mm3": actual.volume(), "triangles": actual.num_tri(),
                                "sha256": digest(path), "legacy_role_for_print_pose": old_role}
        if old_role == "lower":
            transform = ((1,0,0,0),(0,0,1,ymin),(0,-1,0,0))
            new_upper = actual.transform(transform)
        else:
            transform = ((1,0,0,0),(0,0,-1,ymax),(0,1,0,0))
            new_lower = actual.transform(transform)
    write_stl(OUT/"reference/new_upper_world.stl", new_upper)
    write_stl(OUT/"reference/new_lower_world.stl", new_lower)
    combined = v4.union((new_upper,new_lower))
    record(report, "closed_halves_have_no_new_overlap", v4.volume_intersection(new_upper,new_lower) < .02)
    for name, item in items.items():
        record(report, "reused_part_fit_"+name,
               v4.volume_intersection(combined,item) < .03,
               {"intersection_mm3": v4.volume_intersection(combined,item)})
    for p in NEW_PORTS:
        # Probe well inside bore, from the top exterior to below the cavity roof.
        probe = y_cylinder(p["x"],p["z"],ymin-.5,p["cavity_roof_y_mm"]+.20,
                           p["diameter_mm"]-.30)
        record(report, "open_casting_passage_"+p["name"],
               v4.volume_intersection(combined,probe) < .02)
    for p in OLD_PORTS:
        # The former exterior holes must now be solid at their original axes.
        probe = y_cylinder(p["x"],p["z"],ymax-2,ymax-1,.8)
        volume = v4.volume_intersection(new_lower,probe)
        record(report, "old_backside_port_sealed_"+p["name"],
               abs(volume-probe.volume()) < .002)
    after = {str(p.relative_to(ROOT)): digest(p) for p in protected}
    record(report, "all_legacy_STL_BOM_and_builders_unchanged", before == after)
    report["protected_inputs_sha256"] = after
    report["export_numerical_simplification_tolerance_mm"] = .001
    report["elapsed_seconds"] = time.time()-started
    report["summary"] = {"passed": all(c["passed"] for c in report["checks"].values()),
                         "number_of_checks": len(report["checks"]), "new_print_parts": 2}
    (OUT/"audit/V5_GEOMETRY_AUDIT.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("DONE | "+str(OUT), flush=True)


if __name__ == "__main__":
    main()
