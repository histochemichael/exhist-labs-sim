"""Build the CAD-based ExHist layout; no original asset is modified."""
from pathlib import Path
import json, math, shutil, collections
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"
NORI = ROOT/'source/nori/Nori with parallel histo gripper.xml'
HEIGHT = 0.80  # provisional; tune to measured docking heights
def vec(v): return " ".join(f"{float(x):.8g}" for x in v)
def el(parent, tag, **kw): return ET.SubElement(parent, tag, {k:str(v) for k,v in kw.items()})
def box(parent, name, pos, size, rgba, **kw):
    return el(parent,"geom",name=name,type="box",pos=vec(pos),size=vec(size),rgba=vec(rgba),**kw)

def convert(key):
    source_key={"leica_rack":"leica_rack_handled","leica_workstation":"leica_workstation_handled"}.get(key,key)
    src=json.loads((ASSETS/f"{source_key}.json").read_text())
    parts=src["parts"]
    rot=np.eye(3)
    if key in ["s60","scanner_cassette"]: rot=np.array([[1,0,0],[0,0,-1],[0,1,0]])
    if key=="lehisto": rot=np.array([[0,1,0],[-1,0,0],[0,0,1]])
    verts=[np.asarray(p["vertices"]).reshape(-1,3)@rot.T for p in parts]
    whole=np.concatenate(verts);lo=whole.min(0);hi=whole.max(0)
    origin=np.array([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]])
    groups=collections.defaultdict(list)
    for p,v in zip(parts,verts):
        groups[tuple(round(c,3) for c in p["rgba"])].append((v-origin,np.asarray(p["triangles"]).reshape(-1,3)))
    meshes=[]
    for i,(rgba,chunks) in enumerate(groups.items()):
        name=f"{key}_{i}";path=ASSETS/f"{name}.obj"
        with path.open("w") as f:
            offset=1
            for vs,fs in chunks:
                f.writelines("v "+vec(v)+"\n" for v in vs)
                f.writelines("f "+" ".join(str(int(n)+offset) for n in face)+"\n" for face in fs)
                offset+=len(vs)
        meshes.append(dict(name=name,file=f"assets/{name}.obj",rgba=rgba))
    return dict(document=src["document"],bounds_m=(hi-lo).tolist(),meshes=meshes,part_count=len(parts),source_json=f"assets/{source_key}.json")

def build():
    from prepare_handled_racks import build as prepare_racks
    prepare_racks()
    keys=["lehisto","quincy","leica_workstation","s60","slide_folder","rack24","leica_rack","cassette_block"]
    models={key:convert(key) for key in keys}
    root=ET.parse(NORI).getroot();root.set("model","ExHist Labs - CAD layout v01")
    asset=root.find("asset");world=root.find("worldbody")
    head=root.find("visual/headlight")
    head.set("ambient","0.3 0.3 0.3");head.set("diffuse","0.4 0.4 0.4")
    head.set("specular","0.05 0.05 0.05")
    for lamp in world.findall("light"):
        lamp.set("diffuse","0.3 0.3 0.3" if lamp.get("name")=="key_light" else "0.12 0.12 0.12")
        lamp.set("specular","0.02 0.02 0.02")
    # Copy Nori meshes to make the delivered MJCF self-contained.
    ndir=ASSETS/"nori";ndir.mkdir(exist_ok=True)
    for i,m in enumerate(asset.findall("mesh")):
        source=(NORI.parent/m.get("file")).resolve()
        target=ndir/f"{i:02d}_{source.name}";shutil.copy2(source,target)
        m.set("file",target.relative_to(ROOT).as_posix())
    for model in models.values():
        for m in model["meshes"]:
            el(asset,"mesh",name=m["name"],file=m["file"],inertia="shell")
    root.find("visual/global").set("offwidth","1920")
    root.find("visual/global").set("offheight","1080")
    el(root,"option",timestep="0.002",integrator="implicitfast")
    # Original URDF import was world anchored: wrap the robot in a planar base.
    robot=ET.Element("body",name="nori_mobile_base",pos="0 -1.0 0")
    for name,kind,axis in [("base_x","slide","1 0 0"),("base_y","slide","0 1 0"),("base_yaw","hinge","0 0 1")]:
        el(robot,"joint",name=name,type=kind,axis=axis,limited="false",damping="10")
    el(robot,"inertial",pos="0 0 0.12",mass="10",diaginertia="0.3 0.3 0.3")
    for child in list(world):
        if child.tag=="light" or child.get("name")=="checkerboard_floor":continue
        world.remove(child);robot.append(child)
    world.append(robot)
    floor=world.find("geom[@name='checkerboard_floor']")
    floor.set("rgba","0.96 0.97 0.98 1");floor.attrib.pop("material",None)
    placements=[]
    def place(key,name,x,y,z=HEIGHT):
        b=el(world,"body",name=name,pos=vec([x,y,z]))
        for mesh in models[key]["meshes"]:
            el(b,"geom",type="mesh",mesh=mesh["name"],rgba=vec(mesh["rgba"]),
               contype="0",conaffinity="0",group="1",density="0")
        dims=models[key]["bounds_m"]
        # Conservative whole-machine envelope; never use for insertion validation.
        if key in ["lehisto","quincy","leica_workstation","s60"]:
            box(b,name+"_envelope",[0,0,dims[2]/2],np.maximum(np.array(dims)/2,0.001),
                [0.9,0.3,0.1,0],group="3")
        placements.append(dict(name=name,asset=key,position=[x,y,z],bounds_m=dims))
        return b
    stations=[
      ("sorting","PROTOCOL SORTING",1.85),
      ("baking","BAKING",1.65),
      ("routine_a","STAIN + COVERSLIP 1",1.95),
      ("routine_b","STAIN + COVERSLIP 2",1.95),
      ("special","SPECIAL STAIN",1.15),
      ("sendout","SEND-OUT / BLOCK QC",1.95),
      ("imaging_a","IMAGING 1",1.65),
      ("imaging_b","IMAGING 2",1.65)]
    length=sum(s[2] for s in stations)+0.15*(len(stations)-1)
    cursor=-length/2;records=[]
    font_path="C:/Windows/Fonts/arialbd.ttf"
    for i,(name,label,width) in enumerate(stations):
        x=cursor+width/2;cursor+=width+0.15
        depth=1.2 if name.startswith("routine") else 1.0
        y=depth/2+0.05
        t=el(world,"body",name=name+"_table")
        box(t,name+"_top",[x,y,HEIGHT-0.025],[width/2,depth/2,0.025],[0.76,0.84,0.89,1])
        for j,(dx,dy) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
            box(t,f"{name}_leg_{j}",[x+dx*(width/2-0.065),y+dy*(depth/2-0.065),(HEIGHT-0.05)/2],
                [0.025,0.025,(HEIGHT-0.05)/2],[0.26,0.31,0.36,1])
        el(world,"site",name=name+"_dock",type="box",pos=vec([x,-0.62,0.009]),
           size="0.25 0.19 0.008",rgba="0.98 0.55 0.12 0.7",group="0")
        el(world,"site",name=name+"_handoff",pos=vec([x,0.19,HEIGHT+0.035]),
           size="0.023",rgba="0.05 0.75 0.6 1")
        # Clear front-facing labels are real scene textures.
        img=Image.new("RGB",(768,96),(20,43,64));draw=ImageDraw.Draw(img)
        fnt=ImageFont.truetype(font_path,38)
        draw.text((384,48),label,font=fnt,fill="white",anchor="mm")
        img.save(ASSETS/f"{name}_label.png")
        el(asset,"texture",name=name+"_label_tex",type="2d",file=f"assets/{name}_label.png")
        el(asset,"material",name=name+"_label_mat",texture=name+"_label_tex",texuniform="false",emission="0.25")
        # Plane local normal +Z rotated toward aisle (-Y).
        el(world,"geom",name=name+"_label",type="plane",size=vec([width/2-0.05,0.085,0.001]),
           pos=vec([x,0.032,HEIGHT-0.13]),euler=vec([math.pi/2,0,0]),
           material=name+"_label_mat",contype="0",conaffinity="0",group="1")
        records.append(dict(name=name,label=label,x=x,width=width,depth=depth,height=HEIGHT,
                            dock=[x,-0.62,0],handoff=[x,0.19,HEIGHT+0.035]))
        if name=="sorting":
            for j,dx in enumerate([-0.46,0.46]):place("lehisto",f"sorter_{j+1}",x+dx,0.67)
            for j,dx in enumerate([-0.7,-0.48,-0.26]):place("leica_rack",f"filled_leica_{j}",x+dx,0.22)
            for j,dx in enumerate([0.15,0.29,0.43,0.57]):place("rack24",f"protocol_rack24_{j}",x+dx,0.22)
        elif name=="baking":
            for j,dx in enumerate([-0.53,0,0.53]):place("quincy",f"quincy_{j+1}",x+dx,0.63)
        elif name.startswith("routine"):
            place("leica_workstation",name+"_workstation",x,0.69)
        elif name=="special":
            place("lehisto","special_lehisto",x,0.68)
            # Provisional buckets: individual walls, open tops, reagent surface visual.
            for j in range(6):
                bx=x-0.375+j*0.15;by=0.30
                box(world,f"bucket_{j}_base",[bx,by,HEIGHT+0.004],[0.065,0.075,0.004],[0.8,0.83,0.85,1])
                for k,dx in enumerate([-0.062,0.062]):
                    box(world,f"bucket_{j}_side{k}",[bx+dx,by,HEIGHT+0.055],[0.003,0.075,0.055],[0.8,0.83,0.85,1])
                for k,dy in enumerate([-0.072,0.072]):
                    box(world,f"bucket_{j}_end{k}",[bx,by+dy,HEIGHT+0.055],[0.065,0.003,0.055],[0.8,0.83,0.85,1])
                box(world,f"reagent_{j}",[bx,by,HEIGHT+0.07],[0.058,0.068,0.001],
                    [0.2+0.12*j,0.55,0.7-0.07*j,0.6],contype="0",conaffinity="0")
            place("rack24","special_dropoff",x-0.47,0.11)
        elif name=="sendout":
            for j,dx in enumerate([-0.46,0.46]):place("lehisto",f"sendout_lehisto_{j+1}",x+dx,0.72)
            for j,dx in enumerate([-0.56,-0.06]):place("slide_folder",f"sendout_folder_{j+1}",x+dx,0.26)
            for j in range(8):
                place("cassette_block",f"qc_cassette_{j+1}",x+0.48+(j%2)*0.065,0.15+(j//2)*0.055)
        else:
            place("s60",name+"_s60_pc",x,0.68)
            place("rack24",name+"_staging",x-0.45,0.17)
    box(world,"robot_travel_lane",[0,-0.90,0.001],[length/2+0.3,0.8,0.001],
        [0.75,0.87,0.96,1],contype="0",conaffinity="0")
    el(root,"statistic",center="0 0 0.65",extent=str(length*0.65))
    manifest=dict(version=1,stage="CAD layout / kinematic preview",
      sources=models,stations=records,placements=placements,nori_source=str(NORI),
      assumptions=["Table heights 0.80 m are provisional.",
      "CAD equipment is static; articulation export and device controllers remain pending.",
      "Nori planar base is an added kinematic preview abstraction, not a wheel controller.",
      "Machine envelope colliders are conservative and unsuitable for insertion validation.",
      "Special-stain buckets remain provisional; QC cassettes use generic CAD, 40 x 28 x 6 mm cassette with illustrative wax and tissue.",
      "Rack24 is the actual 24 Capacity Slide Rack body; the Leica rack includes 28 slides.",
      "S60 uses the local door-fit-v04 archive; internal cassette depth remains provisional.",
      "No grasp, throughput, robot reach or collision-free transfer claim is made."])
    (ROOT/"layout.json").write_text(json.dumps(manifest,indent=2))
    from canopy_hoods import add_canopies
    add_canopies(root)
    from lab_room import add_room
    add_room(root)
    from machine_glazing import apply as apply_glazing
    apply_glazing(root, articulated=False)
    from special_layout import apply_static
    apply_static(root)
    ET.indent(root);ET.ElementTree(root).write(ROOT/"exhist.xml",encoding="utf-8",xml_declaration=True)
    print(f"Built {len(records)} tables, {len(placements)} CAD instances. Length {length:.2f} m.")
    return manifest
if __name__=="__main__":build()
