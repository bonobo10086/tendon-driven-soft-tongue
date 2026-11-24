"""V4: open loading mold and pre-threaded removable core end dams.

Read-only inputs: V2 molds/inserts and V3 tube centerlines. No mesh repairs,
no changes to purchased-part specifications, and no edits to BOM_CN.md.
Continuous translation envelopes are checked, not just final intersections.
This is geometric verification of nominal rigid CAD, not a physical test.
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
OUT = ROOT / "revision_v4"
sys.path.insert(0, str(ROOT / "tools" / "vendor"))
import manifold3d as m
import build_revision_v3 as v3
from render_assembly_guide_v3 import read_mesh, load_world, Mesh

SPLIT = 17.2
PAD = 0.20
CHECK_PAD = 0.15
STROKE = 45.0
NAMES = v3.NAMES
REPORT = {"version": "V4", "units": "mm", "checks": {}, "parts": {},
          "limits": ["Nominal rigid geometry only; FDM tolerances not measured.",
                     "Steel extraction, seal, cure, strength and fatigue NOT physically tested.",
                     "Opening loading channels locally changes the original cast contour."]}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(key, passed, detail=None):
    REPORT["checks"][key] = {"passed": bool(passed), "detail": detail}
    print(("PASS" if passed else "FAIL") + " | " + key + " | " + str(detail), flush=True)
    if not passed:
        raise AssertionError(key)


def solid(mesh):
    vertices, inverse = np.unique(mesh.vertices.astype(np.float32), axis=0, return_inverse=True)
    faces = inverse[mesh.faces].astype(np.uint32)
    result = m.Manifold(m.Mesh(vertices, faces))
    if result.status() != m.Error.NoError:
        raise ValueError("Invalid input solid: " + str(result.status()))
    return result


def union(items):
    return m.Manifold.batch_boolean(list(items), m.OpType.Add)


def difference(base, *items):
    return m.Manifold.batch_boolean([base, *items], m.OpType.Subtract)


def intersection(a, b):
    return m.Manifold.batch_boolean([a, b], m.OpType.Intersect)


def volume_intersection(a, b):
    return float(intersection(a, b).volume())


def cube(lo, hi):
    lo, hi = np.asarray(lo), np.asarray(hi)
    return m.Manifold.cube(tuple(hi-lo)).translate(tuple(lo))


def axial_bore(x, y, z0, z1, diameter):
    return m.Manifold.cylinder(z1-z0, diameter/2, circular_segments=48).translate((x, y, z0))


def sweep(points, radius, sections=24):
    _, first, second = v3.frames(points)
    angle = np.arange(sections) * 2*math.pi/sections
    rings = points[:, None, :] + radius*(np.cos(angle)[None, :, None]*first[:, None, :]
                                        + np.sin(angle)[None, :, None]*second[:, None, :])
    vertices = np.vstack((rings.reshape(-1, 3), points[0], points[-1]))
    faces = []
    count = len(points)
    for i in range(count-1):
        for j in range(sections):
            k = (j+1) % sections
            a,b,c,d = i*sections+j,i*sections+k,(i+1)*sections+k,(i+1)*sections+j
            faces.extend(((a,b,c),(a,c,d)))
    for j in range(sections):
        k = (j+1) % sections
        faces.extend(((count*sections,k,j),(count*sections+1,(count-1)*sections+j,(count-1)*sections+k)))
    faces = np.asarray(faces, dtype=np.uint32)
    triangles = vertices[faces]
    signed_volume = np.einsum("ij,ij->i", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum()/6
    if signed_volume < 0:
        faces = faces[:, [0,2,1]]
    result = m.Manifold(m.Mesh(vertices.astype(np.float32), faces))
    if result.status() != m.Error.NoError:
        raise ValueError("Invalid sweep: " + str(result.status()))
    return result


def envelope(part, axis, direction, distance=STROKE, pad=PAD):
    lo = np.full(3, -pad)
    hi = np.full(3, pad)
    if direction < 0:
        lo[axis] -= distance
    else:
        hi[axis] += distance
    data = part.to_mesh()
    digest = hashlib.sha256(np.asarray(data.vert_properties).tobytes()+np.asarray(data.tri_verts).tobytes()
                            +str((axis,direction,distance,pad)).encode()).hexdigest()
    cache = OUT/"audit"/"cache"/(digest+".npz")
    cache.parent.mkdir(parents=True,exist_ok=True)
    if cache.exists():
        with np.load(cache) as saved:
            restored = m.Manifold(m.Mesh(saved["vertices"],saved["faces"]))
        if restored.status()==m.Error.NoError:
            return restored
    result = part.minkowski_sum(cube(lo, hi))
    data = result.to_mesh()
    np.savez_compressed(cache,vertices=np.asarray(data.vert_properties),faces=np.asarray(data.tri_verts))
    return result


def mesh_of(part):
    data = part.to_mesh()
    return Mesh(np.asarray(data.vert_properties)[:, :3][np.asarray(data.tri_verts)])


def write_stl(path, part):
    triangles = mesh_of(part).triangles.astype(np.float32)
    normals = np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
    normals /= np.maximum(np.linalg.norm(normals,axis=1)[:,None], 1e-20)
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3,3)), ("attribute", "<u2")])
    records = np.zeros(len(triangles), dtype=dtype)
    records["normal"], records["vertices"] = normals, triangles
    header = b"V4 tongue prototype; units millimetres; nominal CAD, physically unvalidated"
    path.write_bytes(header.ljust(80,b" ")[:80]+struct.pack("<I",len(triangles))+records.tobytes())


def dam(which, zmin, zmax):
    if which == "root":
        body_lo, body_hi = (-9,17.4,zmin), (9,31,-5.4)
        flange_lo, flange_hi = (-11,17.4,zmin-2), (11,33,zmin)
        coords = v3.old.ROOT_PATHS
    else:
        body_lo, body_hi = (-7,17.4,78), (4,29.5,zmax)
        flange_lo, flange_hi = (-9,17.4,zmax), (6,31.5,zmax+2)
        coords = v3.old.TIP_PATHS
    body = union((cube(body_lo,body_hi),cube(flange_lo,flange_hi)))
    bounds = body.bounding_box()
    bores = [axial_bore(x,y,bounds[2]-1,bounds[5]+1,1.4) for x,y in coords.values()]
    return difference(body,*bores)


def world_to_print(part, name, ymin, ymax, zmin, zmax):
    if "upper" in name:
        return part.transform(((1,0,0,0),(0,0,1,0),(0,-1,0,ymax)))
    if "lower" in name:
        return part.transform(((1,0,0,0),(0,0,-1,0),(0,1,0,-ymin)))
    if "root_end" in name:
        return part.translate((0,0,-zmin+2))
    if "tip_end" in name:
        return part.transform(((1,0,0,0),(0,-1,0,0),(0,0,-1,zmax+2)))
    raise ValueError(name)


def main():
    t0 = time.time()
    for sub in ("print", "audit", "preview", "reference"):
        (OUT/sub).mkdir(parents=True,exist_ok=True)
    protected = [ROOT/"BOM_CN.md",*sorted((ROOT/"stl").glob("*.stl")),
                 *sorted((ROOT/"revision_v3"/"templates").glob("*")),
                 ROOT/"revision_v3"/"print"/"tongue_root_bracket_v3.stl"]
    before = {str(p.relative_to(ROOT)):sha(p) for p in protected if p.is_file()}
    up_print = read_mesh(ROOT/"stl"/"tongue_mold_upper_v2.stl")
    low_print = read_mesh(ROOT/"stl"/"tongue_mold_lower_v2.stl")
    ymax = SPLIT + float(up_print.extents[2])
    ymin = SPLIT - (float(low_print.extents[2])-4)
    upper = solid(load_world("tongue_mold_upper_v2.stl",ymin,ymax))
    lower = solid(load_world("tongue_mold_lower_v2.stl",ymin,ymax))
    root = solid(load_world("tongue_root_insert_v2.stl",ymin,ymax))
    tip = solid(load_world("tongue_tip_anchor_v2.stl",ymin,ymax))
    bb = upper.bounding_box()
    zmin,zmax = bb[2],bb[5]
    root_dam,tip_dam = dam("root",zmin,zmax),dam("tip",zmin,zmax)
    items = {"root_insert_v2":root,"tip_anchor_v2":tip,
             "root_end_dam_v4":root_dam,"tip_end_dam_v4":tip_dam}
    rod_ends = {}
    for name in NAMES:
        points = v3.tube_points(name,201)
        items["tube_"+name] = sweep(points,1)
        _,rz,tz = v3.rod_points(name,zmin)
        tube = v3.tube_points(name,201)
        rxy,txy = v3.old.ROOT_PATHS[name],v3.old.TIP_PATHS[name]
        rod_points = np.vstack(([*rxy,rz],tube,[*txy,tz]))
        items["core_"+name] = sweep(rod_points,.5)
        rod_ends[name] = {"root_z":rz,"tip_z":tz,
                          "root_grip_mm":zmin-2-rz,"tip_grip_mm":tz-zmax-2,
                          "tube_centreline_mm":v3.arc_length(v3.tube_points(name,2001))}
    REPORT["coordinates"] = {"split_y":SPLIT,"y_min":ymin,"y_max":ymax,"z_min":zmin,"z_max":zmax}
    REPORT["rod_ends"] = rod_ends
    # Actual target solids, enlarged by 0.2mm, follow the entire loading path.
    negative_sweeps,positive_sweeps = {},{}
    for name,part in items.items():
        print("ENVELOPE | "+name,flush=True)
        negative_sweeps[name] = envelope(part,1,-1)
        positive_sweeps[name] = envelope(part,1,1)
    new_upper = difference(upper,union(negative_sweeps.values()))
    components = new_upper.decompose()
    print("UPPER COMPONENTS | "+str([(p.volume(),p.bounding_box()) for p in components])+" | "+str(new_upper.status()),flush=True)
    if len(components)>1:
        components = sorted(components,key=lambda p:p.volume(),reverse=True)
        discarded = components[1:]
        check("cutting_detached_ribs_are_only_local_cavity_slivers",sum(p.volume() for p in discarded)<40
              and all(p.bounding_box()[0]>-5 and p.bounding_box()[3]<5 for p in discarded),
              [{"volume_mm3":p.volume(),"bounds":p.bounding_box()} for p in discarded])
        REPORT["discarded_cut_slivers_mm3"] = sum(p.volume() for p in discarded)
        new_upper = components[0]
    # Remove near-coplanar Boolean slivers before ANY validation/export.
    # In binary STL, float32 rounding can otherwise collapse their vertices.
    # Simplification moves surfaces by <0.001mm; the actual exported bytes are
    # additionally checked for directed-edge topology by the separate audit.
    new_upper = new_upper.simplify(0.001)
    REPORT["upper_surface_simplification_tolerance_mm"] = 0.001
    upper_print_pose = world_to_print(new_upper,"upper",ymin,ymax,zmin,zmax)
    upper_print_float32 = Mesh(mesh_of(upper_print_pose).triangles.astype(np.float32))
    upper_print_canonical = solid(upper_print_float32).simplify(0.001)
    new_upper = upper_print_canonical.transform(((1,0,0,0),(0,0,-1,ymax),(0,1,0,0)))
    REPORT["print_float32_reconstruction_simplification_mm"] = 0.001
    old_lower_sweeps = {name:volume_intersection(lower,shape) for name,shape in positive_sweeps.items()}
    lower_reuse = max(old_lower_sweeps.values()) < 1e-5
    new_lower = lower if lower_reuse else difference(lower,union(positive_sweeps.values()))
    REPORT["old_lower_loading_intersections_mm3"] = old_lower_sweeps
    REPORT["reuse_old_lower"] = lower_reuse
    REPORT["removed_material_mm3"] = {"upper":upper.volume()-new_upper.volume(),
                                        "lower":lower.volume()-new_lower.volume()}
    added_void = difference(difference(upper,new_upper),root_dam,tip_dam)
    REPORT["net_added_unoccupied_space_mm3"] = added_void.volume()
    REPORT["added_space_note"] = "Subtracts new dams occupying the cut volume; not a total silicone dose or leak-test result."
    check("upper_valid_connected",new_upper.status()==m.Error.NoError and len(new_upper.decompose())==1)
    check("lower_valid_connected",new_lower.status()==m.Error.NoError and len(new_lower.decompose())==1)
    # Regression with a smaller clearance envelope, continuous in translation.
    # A separate checker uses actual STL rays and sampled poses independently.
    for name,part in items.items():
        uv = volume_intersection(new_upper,envelope(part,1,-1,pad=CHECK_PAD))
        lv = volume_intersection(new_lower,envelope(part,1,1,pad=CHECK_PAD))
        check("loading_upper_"+name,uv<1e-5,{"intersection_mm3":uv,"clearance_mm":CHECK_PAD,"stroke_mm":STROKE})
        check("closing_lower_"+name,lv<1e-5,{"intersection_mm3":lv,"clearance_mm":CHECK_PAD,"stroke_mm":STROKE})
    # Final nominal material collisions; rod/tube overlap is intentional.
    pairs = []
    keys = list(items)
    for i,a in enumerate(keys):
        for b in keys[i+1:]:
            if a.startswith("tube_") and b=="core_"+a[5:]:
                continue
            if b.startswith("tube_") and a=="core_"+b[5:]:
                continue
            vol = volume_intersection(items[a],items[b])
            pairs.append({"a":a,"b":b,"intersection_mm3":vol})
    REPORT["pairwise_intersections"] = pairs
    check("final_group_no_unintended_collisions",max(x["intersection_mm3"] for x in pairs)<1e-5)
    # Sliding preassembly from straight free tails, before mold insertion.
    cores = union(items["core_"+name] for name in NAMES)
    # Exclude only 0.01mm at intended tube/stop face contact for this padded
    # sliding test; the exact unpadded final pairwise test above remains zero.
    tubes = union(sweep(v3.tube_points(name,201,zs=np.linspace(5.01,76.39,201)),1) for name in NAMES)
    for key,direction,distance in (("root_insert_v2",-1,24),("tip_anchor_v2",1,20),
                                   ("root_end_dam_v4",-1,12),("tip_end_dam_v4",1,17)):
        axial = envelope(items[key],2,direction,distance,pad=0.001)
        cv,tv = volume_intersection(axial,cores),volume_intersection(axial,tubes)
        check("prethread_"+key,max(cv,tv)<1e-5,{"core_mm3":cv,"tube_mm3":tv,"travel_mm":distance})
    check("core_grip_minimum_5mm",min(min(e["root_grip_mm"],e["tip_grip_mm"]) for e in rod_ends.values())>=5,rod_ends)
    # Existing six bolt bores and four registration pin bores are measured at
    # the original coordinate pattern, and the surrounding material retained.
    xs = (bb[0]+5,bb[3]-5)
    zs = (zmin+5,(zmin+zmax)/2,zmax-5)
    fastening = []
    removed = union((difference(upper,new_upper),difference(lower,new_lower)))
    mold_pair = union((new_upper,new_lower))
    for x in xs:
        for z in zs:
            probe = m.Manifold.cylinder(ymax-ymin+2,1.65,circular_segments=32).transform(((1,0,0,x),(0,0,1,ymin-1),(0,-1,0,z)))
            guard = m.Manifold.cylinder(ymax-ymin+2,3.75,circular_segments=32).transform(((1,0,0,x),(0,0,1,ymin-1),(0,-1,0,z)))
            fastening.append({"x":x,"z":z,"bore_intersection_mm3":volume_intersection(mold_pair,probe),
                              "removed_from_radius_3_75_guard_mm3":volume_intersection(removed,guard)})
    check("six_M3_bores_and_2mm_radial_rims_preserved",all(max(p["bore_intersection_mm3"],p["removed_from_radius_3_75_guard_mm3"])<1e-5 for p in fastening),fastening)
    for x in (-24,24):
        for z in (zmin+5,zmax-5):
            hole = m.Manifold.cylinder(4.6,2.1,circular_segments=32).transform(((1,0,0,x),(0,0,1,SPLIT),(0,-1,0,z)))
            guard = m.Manifold.cylinder(8,3,circular_segments=32).transform(((1,0,0,x),(0,0,1,SPLIT-3),(0,-1,0,z)))
            check("registration_"+str((x,z)),max(volume_intersection(new_upper,hole),volume_intersection(removed,guard))<1e-5)
    lower_motion = envelope(new_lower,1,-1,pad=0.000001)
    check("mold_halves_continuous_closure",volume_intersection(new_upper,lower_motion)<0.02,
          {"intersection_mm3":volume_intersection(new_upper,lower_motion),"contact_tolerance_mm3":0.02})
    # UP tube crosses below the pour-port centre. Verify an actual side flow
    # route, not a falsely unobstructed centreline. Do not insert tools here.
    for name,x,z,r,y0 in (("pour_side_route",2.5,8,1,26),("vent_left",-4.5,73,.8,27.5),("vent_right",3,73,.8,27.5)):
        probe = m.Manifold.cylinder(ymax-y0+1,r,circular_segments=32).transform(((1,0,0,x),(0,0,1,y0),(0,-1,0,z)))
        group = union(items.values())
        check("open_"+name,volume_intersection(union((new_upper,group)),probe)<1e-5,
              {"x":x,"z":z,"probe_radius_mm":r,"from_y":y0,"intersection_mm3":volume_intersection(union((new_upper,group)),probe)})
    old_code = (HERE/"build_printables.py").read_text(encoding="utf-8")
    REPORT["builder_reference_sha256"] = hashlib.sha256(old_code.encode()).hexdigest()
    outputs = {"tongue_mold_upper_v4.stl":new_upper,
               "tongue_root_end_dam_v4.stl":root_dam,
               "tongue_tip_end_dam_v4.stl":tip_dam}
    if not lower_reuse:
        outputs["tongue_mold_lower_v4.stl"] = new_lower
    # All reference meshes are in common assembly coordinates, not for print.
    for name,part in {"upper_world":new_upper,"lower_world":new_lower,**items}.items():
        write_stl(OUT/"reference"/(name+".stl"),part)
    for name,part in outputs.items():
        printable = upper_print_canonical if "upper" in name else world_to_print(part,name,ymin,ymax,zmin,zmax)
        path = OUT/"print"/name
        write_stl(path,printable)
        reloaded = solid(read_mesh(path))
        dims = np.asarray(reloaded.bounding_box())[3:]-np.asarray(reloaded.bounding_box())[:3]
        check("stl_roundtrip_"+name,reloaded.status()==m.Error.NoError and len(reloaded.decompose())==1 and reloaded.volume()>0)
        check("print_bed_"+name,abs(reloaded.bounding_box()[2])<1e-4,reloaded.bounding_box())
        REPORT["parts"][name] = {"quantity":1,"dimensions_mm":dims.tolist(),"volume_mm3":reloaded.volume(),
                                  "triangles":reloaded.num_tri(),"sha256":sha(path)}
    after = {str(p.relative_to(ROOT)):sha(p) for p in protected if p.is_file()}
    check("protected_BOM_old_STLs_V3_templates_unchanged",before==after)
    REPORT["protected_inputs_sha256"] = after
    REPORT["elapsed_seconds"] = time.time()-t0
    (OUT/"audit"/"V4_GEOMETRY_AUDIT.json").write_text(json.dumps(REPORT,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("DONE | "+str(OUT)+" | "+str(REPORT["elapsed_seconds"]),flush=True)


if __name__ == "__main__":
    main()
