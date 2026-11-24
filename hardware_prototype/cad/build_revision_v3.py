"""Create a backward-compatible V3 bracket and corrected routing templates.

Reads the existing V2 STL bytes without replacing any V2 part, template or
BOM. Only the L-bracket needs a replacement print. The corrected tube curves
keep the V2 root/tip seats, mould pockets and fixing holes unchanged.

Digital checks are not an experimental strength, curing or fatigue test.
"""

from pathlib import Path
import hashlib
import json
import math
import sys
import zipfile

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / "revision_v3"
sys.path.insert(0, str(ROOT / "tools" / "vendor"))
import trimesh
import build_printables as old
from audit_printables import connected_body_count
from inspect_day3 import font, ray_hits


NAMES = ["LEFT", "RIGHT", "DOWN", "UP"]
COLORS = {"LEFT":"#20824c", "RIGHT":"#7e40af", "DOWN":"#207bc1", "UP":"#c33131"}
PRINT_NAME = "tongue_root_bracket_v3.stl"
ROOT_STOP_Z = 5.0
TIP_STOP_Z = 76.4
ROOT_STRAIGHT_END_Z = 8.5
TIP_STRAIGHT_START_Z = 74.0
VALLEY_Z = 42.0
VALLEY_Y = 9.5
ROD_LENGTH = 120.0
ROOT_ROD_OVERHANG = 8.0


def load_part(filename):
    return trimesh.load_mesh(ROOT / "stl" / filename, force="mesh", process=True)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def smooth(t):
    return t*t*(3-2*t)


def tube_points(name, count=501, zs=None):
    if zs is None:
        zs=np.linspace(ROOT_STOP_Z,TIP_STOP_Z,count)
    zs=np.asarray(zs,dtype=float)
    first=np.clip((zs-ROOT_STRAIGHT_END_Z)/(VALLEY_Z-ROOT_STRAIGHT_END_Z),0,1)
    second=np.clip((zs-VALLEY_Z)/(TIP_STRAIGHT_START_Z-VALLEY_Z),0,1)
    centre=np.where(zs<=VALLEY_Z,25+(VALLEY_Y-25)*smooth(first),VALLEY_Y+(25-VALLEY_Y)*smooth(second))
    ratio=np.clip((zs-ROOT_STRAIGHT_END_Z)/(TIP_STRAIGHT_START_Z-ROOT_STRAIGHT_END_Z),0,1)
    rx,ry=old.ROOT_PATHS[name]
    tx,ty=old.TIP_PATHS[name]
    xs=rx+(tx-rx)*smooth(ratio)
    ys=centre+(ry-25)+(ty-ry)*smooth(ratio)
    return np.column_stack((xs,ys,zs))


def arc_length(points):
    return float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())


def frames(points):
    tangent=np.gradient(points,axis=0)
    tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    first=np.cross(tangent,np.broadcast_to(np.array([1.,0.,0.]),tangent.shape))
    first/=np.linalg.norm(first,axis=1)[:,None]
    second=np.cross(tangent,first)
    return tangent,first,second


def sweep(points, radius, sections=32):
    _,first,second=frames(points)
    theta=np.arange(sections)*(2*math.pi/sections)
    rings=points[:,None,:]+radius*(np.cos(theta)[None,:,None]*first[:,None,:]+np.sin(theta)[None,:,None]*second[:,None,:])
    vertices=rings.reshape(-1,3).tolist()
    faces=[]
    count=len(points)
    for i in range(count-1):
        for j in range(sections):
            nxt=(j+1)%sections
            a,b,c,d=i*sections+j,i*sections+nxt,(i+1)*sections+nxt,(i+1)*sections+j
            faces.extend([[a,b,c],[a,c,d]])
    start_cap=len(vertices)
    vertices.append(points[0].tolist())
    end_cap=len(vertices)
    vertices.append(points[-1].tolist())
    for j in range(sections):
        nxt=(j+1)%sections
        faces.append([start_cap,nxt,j])
        faces.append([end_cap,(count-1)*sections+j,(count-1)*sections+nxt])
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    if mesh.volume<0:
        mesh.invert()
    assert mesh.is_watertight and mesh.is_winding_consistent
    return mesh


def window_cutter():
    # Window in PRINT coordinates: flat bottom, vertical sides and 45-degree
    # roof. The roof avoids introducing a new 20mm horizontal bridge.
    polygon=[(-10.,11.),(10.,11.),(10.,20.),(0.,30.),(-10.,20.)]
    n=len(polygon)
    vertices=[[x,y,z] for y in (4.,10.) for x,z in polygon]
    faces=[]
    for i in range(1,n-1):
        faces.extend([[0,i,i+1],[n,n+i+1,n+i]])
    for i in range(n):
        j=(i+1)%n
        faces.extend([[i,n+i,n+j],[i,n+j,j]])
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
    if mesh.volume<0:
        mesh.invert()
    assert mesh.is_watertight and mesh.is_winding_consistent
    return mesh


def to_world(filename, mesh, tongue):
    result=mesh.copy()
    y_min=float(tongue.bounds[0,1]-old.MOLD_BOTTOM_WALL)
    y_max=float(tongue.bounds[1,1]+old.MOLD_TOP_WALL)
    matrix=np.eye(4)
    if "mold_lower" in filename:
        matrix[:3,:3]=[[1,0,0],[0,0,1],[0,-1,0]]
        matrix[1,3]=y_min
    elif "mold_upper" in filename:
        matrix[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
        matrix[1,3]=y_max
    elif "root_insert" in filename:
        matrix[2,3]=-5.
    elif "tip_anchor" in filename:
        matrix[:3,:3]=[[1,0,0],[0,-1,0],[0,0,-1]]
        matrix[2,3]=77.6
    elif "root_bracket" in filename:
        matrix[:3,:3]=[[1,0,0],[0,0,1],[0,-1,0]]
        matrix[1,3]=6.
    else:
        raise ValueError(filename)
    result.apply_transform(matrix)
    return result


def rod_points(name, z_min):
    tube=tube_points(name,2001)
    tube_len=arc_length(tube)
    root_z=z_min-ROOT_ROD_OVERHANG
    root_free=ROOT_STOP_Z-root_z
    tip_z=TIP_STOP_Z+(ROD_LENGTH-tube_len-root_free)
    root_x,root_y=old.ROOT_PATHS[name]
    tip_x,tip_y=old.TIP_PATHS[name]
    root=np.array([[root_x,root_y,z] for z in np.linspace(root_z,ROOT_STOP_Z,60)[:-1]])
    tip=np.array([[tip_x,tip_y,z] for z in np.linspace(TIP_STOP_Z,tip_z,60)[1:]])
    return np.vstack((root,tube,tip)),root_z,tip_z


def check(report, key, condition, detail=None):
    report["checks"][key]={"passed":bool(condition),"detail":detail}
    print(("PASS" if condition else "FAIL")+" | "+key+(" | "+str(detail) if detail is not None else ""),flush=True)
    if not condition:
        raise AssertionError(key)


def boolean_volume(mesh):
    # Empty Boolean results have no volume. Avoid asking trimesh to evaluate
    # mass properties of an empty mesh, which can emit divide-by-zero warnings.
    return 0.0 if len(mesh.faces) == 0 else float(mesh.volume)


def create_template(path, projection, z_min, z_max, lengths):
    # A4 portrait, four separate paths, exact millimetre SVG dimensions.
    lines=['<?xml version="1.0" encoding="UTF-8"?>','<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="297mm" viewBox="0 0 210 297">','<rect width="210" height="297" fill="white"/>']
    title="SIDE VIEW (y-z)" if projection=="side" else "TOP VIEW (x-z)"
    lines.append(f'<text x="10" y="9" font-size="4">V3 mandrel template: {title}</text>')
    lines.append('<text x="10" y="15" font-size="3">Print at 100% / actual size. Rod: diameter 1 mm, length 120 mm.</text>')
    for i,name in enumerate(NAMES):
        row=24+i*61
        rod,root_z,tip_z=rod_points(name,z_min)
        tube=tube_points(name,501)
        def xy(points):
            px=10+points[:,2]-root_z
            py=row+12+(30-points[:,1]) if projection=="side" else row+28-points[:,0]
            return np.column_stack((px,py))
        def line(points,color,width):
            coords=" ".join(f"{x:.4f},{y:.4f}" for x,y in xy(points))
            return f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{width}"/>'
        lines.append(f'<rect x="5" y="{row-3}" width="200" height="57" fill="none" stroke="#cbd5e1" stroke-width="0.2"/>')
        lines.append(f'<text x="10" y="{row+2}" font-size="3.5" fill="{COLORS[name]}">{name}</text>')
        lines.append(line(rod,"#444",0.35))
        lines.append(line(tube,COLORS[name],0.5))
        for z,label in [(ROOT_STOP_Z,"ROOT SEAT"),(TIP_STOP_Z,"TIP SEAT")]:
            x=10+z-root_z
            lines.append(f'<line x1="{x:.4f}" y1="{row+7}" x2="{x:.4f}" y2="{row+44}" stroke="#7c8898" stroke-width="0.2" stroke-dasharray="1,1"/>')
            lines.append(f'<text x="{x-8:.4f}" y="{row+49}" font-size="2.5">{label}</text>')
        lines.append(f'<text x="137" y="{row+18}" font-size="3">Tube centreline:</text>')
        lines.append(f'<text x="137" y="{row+24}" font-size="3">{lengths[name]:.2f} mm</text>')
        lines.append(f'<text x="137" y="{row+32}" font-size="2.6">Grey ends = bare rod</text>')
        lines.append(f'<text x="137" y="{row+38}" font-size="2.6">Both views are required</text>')
    lines.extend(['<line x1="10" y1="284" x2="30" y2="284" stroke="black" stroke-width="0.5"/>','<text x="10" y="280" font-size="3">20 mm scale check</text>','<text x="45" y="284" font-size="3">Do not use old V2 bending templates.</text>','</svg>'])
    path.write_text("\n".join(lines)+"\n",encoding="utf-8")


def render_fitted(mesh, size, direction):
    forward=np.asarray(direction,dtype=float)
    forward/=np.linalg.norm(forward)
    reference=np.array([0.,1.,0.]) if abs(forward[1])<.92 else np.array([0.,0.,1.])
    right=np.cross(reference,forward)
    right/=np.linalg.norm(right)
    up=np.cross(forward,right)
    coords=np.column_stack((mesh.vertices@right,mesh.vertices@up))
    centre=(coords.min(axis=0)+coords.max(axis=0))/2
    scale=min((size[0]-60)/np.ptp(coords[:,0]),(size[1]-60)/np.ptp(coords[:,1]))
    def pixels(points):
        points=np.atleast_2d(points)
        projected=np.column_stack((points@right,points@up))-centre
        return np.column_stack((size[0]/2+projected[:,0]*scale,size[1]/2-projected[:,1]*scale))
    image=Image.new("RGB",size,"#eef3f8")
    draw=ImageDraw.Draw(image)
    xy=pixels(mesh.vertices)
    depths=mesh.vertices@forward
    light=np.array([.2,-.4,1.])
    light/=np.linalg.norm(light)
    for i in np.argsort(depths[mesh.faces].mean(axis=1)):
        strength=.38+.62*max(0.,float(mesh.face_normals[i]@light))
        draw.polygon([tuple(p) for p in xy[mesh.faces[i]]],fill=tuple(int(v*strength) for v in (150,185,214)))
    return image,pixels


def draw_preview(bracket):
    sheet=Image.new("RGB",(1500,850),"white")
    draw=ImageDraw.Draw(sheet)
    draw.text((25,15),"V3：只重印L形舌根支架；V2舌根嵌件保持不变",fill="#17324d",font=font(30))
    for x,direction,label in [(20,(.35,-.8,.6),"新支架：中心通窗，外形和安装孔不变"),(760,(0,-1,0),"正面：四个拉索出口均落在通窗内")]:
        picture,pixels=render_fitted(bracket,size=(720,570),direction=direction)
        sheet.paste(picture,(x,75))
        draw.text((x+10,660),label,fill="#17324d",font=font(23))
        if x==760:
            for name,(px,py) in old.ROOT_PATHS.items():
                sx,sy=pixels([px,5.,py-6.])[0]+np.array([x,75])
                draw.ellipse((sx-9,sy-9,sx+9,sy+9),outline=COLORS[name],width=3)
                draw.text((sx+13,sy-12),name,fill=COLORS[name],font=font(17))
    draw.text((25,730),"外形 56×36×34 mm；4个M3安装孔、4个底座孔沿用V2。",fill="#17324d",font=font(25))
    draw.text((25,775),"这是一张数字图纸预览，不代表实物强度或长期寿命已经验证。",fill="#526779",font=font(23))
    sheet.save(OUT/"preview"/"bracket_v3.png")


def main():
    preserved=[ROOT/"stl"/name for name in ["tongue_mold_lower_v2.stl","tongue_mold_upper_v2.stl","tongue_root_insert_v2.stl","tongue_tip_anchor_v2.stl","tongue_root_bracket_v2.stl"]]
    preserved += [ROOT/"BOM_CN.md",ROOT/"tube_path_template_1to1.svg",ROOT/"tube_path_template_top_1to1.svg"]
    before={str(p.relative_to(ROOT)):sha(p) for p in preserved}
    for directory in ["print","templates","preview","audit"]:
        (OUT/directory).mkdir(parents=True,exist_ok=True)
    report={"version":"V3","scope":"Digital geometry, compatibility, routing and file integrity only; no physical strength or casting test", "checks":{},"parts_to_reprint":[PRINT_NAME],"preserved_v2_sha256":before}

    original=load_part("tongue_root_bracket_v2.stl")
    bracket=trimesh.boolean.difference([original,window_cutter()],engine="manifold")
    target=OUT/"print"/PRINT_NAME
    old.export_mesh(bracket,target)
    bracket=trimesh.load_mesh(target,process=True)
    check(report,"new_stl_closed_and_consistent",bracket.is_watertight and bracket.is_winding_consistent)
    check(report,"new_stl_single_body",connected_body_count(bracket)==1)
    check(report,"unchanged_bracket_external_dimensions",np.allclose(bracket.extents,[56,36,34],atol=.001),bracket.extents.tolist())
    check(report,"default_print_z_zero",abs(float(bracket.bounds[0,2]))<1e-6)
    check(report,"revision_only_removes_material",boolean_volume(trimesh.boolean.difference([bracket,original],engine="manifold"))<.001)

    for name,(x,y) in old.ROOT_PATHS.items():
        clear=True
        for angle in np.linspace(0,2*math.pi,25)[:-1]:
            origin=np.array([x+.65*math.cos(angle),4.8,y-6+.65*math.sin(angle)])
            clear &= len(ray_hits(bracket,origin,np.array([0.,1.,0.])))==0
        check(report,"bracket_exit_clear_"+name,clear)
    mating_face_ok=True
    for i,(x,z) in enumerate([(-15,13),(16,13),(-15,25),(16,25)]):
        check(report,"unchanged_M3_hole_"+str(i+1),not ray_hits(bracket,np.array([x,4.8,z]),np.array([0.,1.,0.])))
        supported=True
        for radius in (2.,3.45):
            for angle in np.linspace(0,2*math.pi,25)[:-1]:
                point=[x+radius*math.cos(angle),7.,z+radius*math.sin(angle)]
                supported &= old.point_inside_mesh(bracket,point)
                if radius == 2.:
                    origin=np.array([point[0],4.8,point[2]])
                    direction=np.array([0.,1.,0.])
                    hits=ray_hits(bracket,origin,direction)
                    mating_face_ok &= hits == [0.2,4.2] and hits == ray_hits(original,origin,direction)
        check(report,"M3_surrounding_bearing_land_"+str(i+1),supported)
    # Check the actual planar contact lands around all four mounting holes,
    # not the overall bounding box (the base protrudes beyond the wall).
    check(report,"root_mounting_face_unchanged",mating_face_ok,"Front and rear wall faces remain at print y=5 and 9 mm around all four M3 holes")
    for i,(x,y) in enumerate([(-22,14),(23,14),(-22,30),(23,30)]):
        check(report,"unchanged_base_hole_"+str(i+1),not ray_hits(bracket,np.array([x,y,-1.]),np.array([0.,0.,1.])))

    tongue=old.load_scaled_tongue()
    names=["tongue_mold_lower_v2.stl","tongue_mold_upper_v2.stl","tongue_root_insert_v2.stl","tongue_tip_anchor_v2.stl"]
    world={name:to_world(name,load_part(name),tongue) for name in names}
    world_bracket=to_world(PRINT_NAME,bracket,tongue)
    root=world["tongue_root_insert_v2.stl"]
    overlap=boolean_volume(trimesh.boolean.intersection([root,world_bracket],engine="manifold"))
    check(report,"V2_root_insert_V3_bracket_no_collision",overlap<.001,round(float(overlap),6))
    for insert in ["tongue_root_insert_v2.stl","tongue_tip_anchor_v2.stl"]:
        for mold in ["tongue_mold_lower_v2.stl","tongue_mold_upper_v2.stl"]:
            vol=boolean_volume(trimesh.boolean.intersection([world[insert],world[mold]],engine="manifold"))
            check(report,insert+"_fits_"+mold,vol<.01,round(float(vol),6))

    z_min=float(tongue.bounds[0,2]-old.MOLD_MARGIN_Z)
    z_max=float(tongue.bounds[1,2]+old.MOLD_MARGIN_Z)
    lengths={}
    report["paths"]={}
    tubes={}
    for name in NAMES:
        dense=tube_points(name,2001)
        lengths[name]=arc_length(dense)
        check(report,"tube_seat_centres_"+name,np.allclose(dense[0],list(old.ROOT_PATHS[name])+[ROOT_STOP_Z]) and np.allclose(dense[-1],list(old.TIP_PATHS[name])+[TIP_STOP_Z]))
        rods,root_z,tip_z=rod_points(name,z_min)
        check(report,"120mm_rod_length_"+name,abs(arc_length(rods)-ROD_LENGTH)<.001)
        check(report,"rod_external_grips_"+name,(z_min-root_z)>=5 and (tip_z-z_max)>=5,dict(root_mm=round(z_min-root_z,3),tip_mm=round(tip_z-z_max,3)))
        minimum_rod=lengths[name]+ROOT_STOP_Z-z_min+z_max-TIP_STOP_Z
        check(report,"100mm_cannot_span_both_outer_faces_"+name,minimum_rod>100,round(minimum_rod,3))
        first=np.gradient(dense,dense[:,2],axis=0)
        second=np.gradient(first,dense[:,2],axis=0)
        curvature=np.linalg.norm(np.cross(first,second),axis=1)/np.linalg.norm(first,axis=1)**3
        radius=float(1/max(curvature))
        check(report,"minimum_curve_radius_above_10mm_"+name,radius>10,round(radius,3))
        report["paths"][name]={"tube_centreline_mm":round(lengths[name],4),"initial_tube_cut_mm":80,"minimum_bend_radius_mm":round(radius,3),"minimum_rod_to_span_mold_mm":round(minimum_rod,3),"rod_length_mm":ROD_LENGTH,"root_outside_mm":round(z_min-root_z,3),"tip_outside_mm":round(tip_z-z_max,3)}
        tube_mesh=sweep(tube_points(name,301),1.0)
        rod_mesh=sweep(rods[::4] if (len(rods)-1)%4==0 else np.vstack((rods[::4],rods[-1])),.5)
        tubes[name]=tube_mesh
        for part_name,part in world.items():
            tube_vol=boolean_volume(trimesh.boolean.intersection([tube_mesh,part],engine="manifold"))
            rod_vol=boolean_volume(trimesh.boolean.intersection([rod_mesh,part],engine="manifold"))
            check(report,name+"_tube_clear_"+part_name,tube_vol<.01,round(tube_vol,6))
            check(report,name+"_rod_clear_"+part_name,rod_vol<.01,round(rod_vol,6))
        points=tube_points(name,181)
        _,n1,n2=frames(points)
        skin_ok=True
        bad=None
        for i,p in enumerate(points):
            if p[2]<8 or p[2]>73.5:
                continue
            for angle in np.linspace(0,2*math.pi,17)[:-1]:
                shell=p+1.5*(math.cos(angle)*n1[i]+math.sin(angle)*n2[i])
                if not old.point_inside_mesh(tongue,shell):
                    skin_ok=False
                    bad=shell.round(3).tolist()
                    break
            if not skin_ok:
                break
        check(report,name+"_1p5mm_envelope_within_cavity",skin_ok,bad)
    for i,a in enumerate(NAMES):
        for b in NAMES[i+1:]:
            vol=boolean_volume(trimesh.boolean.intersection([tubes[a],tubes[b]],engine="manifold"))
            check(report,"tubes_no_collision_"+a+"_"+b,vol<.001,round(float(vol),6))

    create_template(OUT/"templates"/"tube_path_side_v3_A4_1to1.svg","side",z_min,z_max,lengths)
    create_template(OUT/"templates"/"tube_path_top_v3_A4_1to1.svg","top",z_min,z_max,lengths)
    draw_preview(bracket)
    after={str(p.relative_to(ROOT)):sha(p) for p in preserved}
    check(report,"all_V2_STLs_templates_and_BOM_unchanged",before==after)
    archive=OUT/"ONLY_REPRINT_BRACKET_V3.zip"
    with zipfile.ZipFile(archive,"w",compression=zipfile.ZIP_DEFLATED) as zipped:
        zipped.write(target,arcname=PRINT_NAME)
    with zipfile.ZipFile(archive) as zipped:
        check(report,"print_zip_exactly_one_STL",zipped.namelist()==[PRINT_NAME] and zipped.testzip() is None and zipped.read(PRINT_NAME)==target.read_bytes())
    report["new_stl_sha256"]=sha(target)
    report["all_digital_checks_passed"]=all(row["passed"] for row in report["checks"].values())
    (OUT/"audit"/"V3_GEOMETRY_AUDIT.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print("V3 OUTPUT |",OUT,flush=True)


if __name__ == "__main__":
    main()
