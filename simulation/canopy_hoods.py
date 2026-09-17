"""Conceptual suspended canopies; no airflow or ventilation certification."""
import json
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def add_special_canopy(root):
    """Open-bottom concept over the eleven special-stain jars; not ventilation design."""
    from build_scene import el,box,vec
    world=root.find('worldbody');old=world.find("body[@name='special_canopy']")
    if old is not None:world.remove(old)
    center=[1.15,.46,1.70];width=1.08;depth=.76;height=.20
    b=el(world,'body',name='special_canopy',pos=vec(center))
    # Individual thin panels preserve an open capture volume in collision geometry.
    for sign in (-1,1):
        box(b,f'special_canopy_side_{sign}',[sign*width/2,0,height/2],[.006,depth/2,height/2],[.67,.72,.75,1])
        box(b,f'special_canopy_frontback_{sign}',[0,sign*depth/2,height/2],[width/2,.006,height/2],[.67,.72,.75,1])
    box(b,'special_canopy_roof',[0,0,height],[width/2,depth/2,.006],[.67,.72,.75,1])
    for i in range(7):
        box(b,f'special_canopy_baffle_{i}',[(i-3)*.13,0,.16],[.035,.29,.004],[.38,.43,.46,1])
    box(b,'special_canopy_light',[0,-.29,.14],[.38,.012,.005],[.95,.98,1,1],contype='0',conaffinity='0')
    el(b,'geom',name='special_canopy_duct',type='cylinder',pos='0 0 .75',size='.10 .55',rgba='.62 .67 .70 1')
    for i,(x,y) in enumerate([(-.45,-.30),(-.45,.30),(.45,-.30),(.45,.30)]):
        el(b,'geom',name=f'special_canopy_hanger_{i}',type='capsule',fromto=vec([x,y,height,x,y,1.30]),size='.004',rgba='.4 .43 .46 1')
    report=dict(station='special',position_m=center,width_m=width,depth_m=depth,underside_m=1.70,
                table_top_m=.80,clearance_above_table_m=.90,
                scope='Concept only: airflow, containment, chemical compatibility, duct routing and structural design unvalidated.')
    (ROOT/'special_canopy_layout.json').write_text(json.dumps(report,indent=2))
    return report

def add_canopies(root):
    from build_scene import el,box,vec
    source=json.loads((ROOT/"assets/leica_workstation_handled.json").read_text())
    allv=np.concatenate([np.array(p["vertices"]).reshape(-1,3) for p in source["parts"]])
    origin=np.r_[(allv.min(0)[:2]+allv.max(0)[:2])/2,allv.min(0)[2]]
    stv=np.concatenate([np.array(p["vertices"]).reshape(-1,3) for p in source["parts"] if p["name"].startswith("ST5020")])
    lo=stv.min(0)-origin;hi=stv.max(0)-origin
    width=hi[0]-lo[0]+.30;depth=hi[1]-lo[1]+.30;height=.22
    # Wide open-bottom capture canopy with taper to a flat upper plenum.
    lower=[[-width/2,-depth/2,0],[width/2,-depth/2,0],[width/2,depth/2,0],[-width/2,depth/2,0]]
    upper=[[x*.65,y*.65,height] for x,y,z in lower]
    vertices=lower+upper
    faces=[(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7),(4,5,6),(4,6,7)]
    path=ROOT/"assets/stainer_canopy.obj"
    with path.open("w") as f:
        for v in vertices:f.write("v "+vec(v)+"\n")
        for face in faces:f.write("f "+" ".join(str(i+1) for i in face)+"\n")
    asset=root.find("asset");world=root.find("worldbody")
    if asset.find("mesh[@name='stainer_canopy_mesh']") is None:
        el(asset,"mesh",name="stainer_canopy_mesh",file="assets/stainer_canopy.obj",inertia="shell")
    records=[]
    for station in ["routine_a","routine_b"]:
        old=world.find(f"body[@name='{station}_canopy']")
        if old is not None:world.remove(old)
        machine=world.find(f"body[@name='{station}_workstation']")
        base=np.fromstring(machine.get("pos"),sep=" ")
        center=base+np.r_[(lo[:2]+hi[:2])/2,hi[2]+.50]
        body=el(world,"body",name=station+"_canopy",pos=vec(center))
        el(body,"geom",name=station+"_canopy_shell",type="mesh",mesh="stainer_canopy_mesh",rgba=".65 .70 .73 1",contype="1",conaffinity="1",group="1")
        for sign in [-1,1]:
            box(body,station+f"_canopy_lip_y{sign}",[0,sign*depth/2,.006],[width/2,.012,.016],[.78,.81,.83,1])
            box(body,station+f"_canopy_lip_x{sign}",[sign*width/2,0,.006],[.012,depth/2,.016],[.78,.81,.83,1])
        # Shallow underside baffle strips and task light are visual details.
        for i in range(7):
            box(body,station+f"_canopy_baffle_{i}",[(i-3)*width/9,0,.05],[.007,depth*.36,.004],[.3,.34,.36,1],contype="0",conaffinity="0")
        box(body,station+"_canopy_light",[0,-depth*.35,.045],[width*.30,.01,.005],[.93,.97,1,1],contype="0",conaffinity="0")
        top=max(3.0,center[2]+height+.45);ductheight=top-center[2]-height
        el(body,"geom",name=station+"_canopy_duct",type="cylinder",pos=vec([0,0,height+ductheight/2]),
           size=vec([.10,ductheight/2]),rgba=".62 .67 .70 1")
        for i,(dx,dy) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
            start=[dx*width*.31,dy*depth*.31,height]
            end=[start[0],start[1],top-center[2]]
            el(body,"geom",name=station+f"_canopy_hanger_{i}",type="capsule",fromto=vec(start+end),size=".004",rgba=".4 .43 .46 1")
        records.append(dict(station=station,position_m=center.tolist(),width_m=width,depth_m=depth,
            underside_height_m=float(center[2]-.01),machine_static_top_m=float(base[2]+hi[2]),static_clearance_m=.49,
            duct_diameter_m=.20,top_height_m=float(top)))
    report=dict(canopies=records,design="Suspended stainless-look canopy, tapered plenum, vertical duct stub and hanger rods",
        limitations=["Conceptual layout geometry only; no airflow, fume capture, duct routing, structural or ventilation-code validation.",
          "Clearance is above current CAD envelope, not a guarantee for every possible machine joint configuration."])
    (ROOT/"canopy_layout.json").write_text(json.dumps(report,indent=2))
    return report

if __name__=="__main__":
    for name in ["exhist.xml","exhist_operational.xml"]:
        root=ET.parse(ROOT/name).getroot();report=add_canopies(root);add_special_canopy(root)
        ET.indent(root);ET.ElementTree(root).write(ROOT/name,encoding="utf-8",xml_declaration=True)
    print(json.dumps(report,indent=2))
