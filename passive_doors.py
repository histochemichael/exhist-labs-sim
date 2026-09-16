"""Import measured CAD door pivots as passive joints, never scripted actuators.

This preserves the exported CAD pose at qpos0. Door masses/damping are explicit
estimates, and collision cavity construction is NOT complete: physics is gated.
"""
import json,collections
import numpy as np
from build_scene import ROOT,ASSETS,el,vec

def add_passive_doors(root):
    joints=json.loads((ASSETS/"passive_door_joints.json").read_text())
    layout=json.loads((ROOT/"layout.json").read_text())
    rows=[]
    for key in ("quincy","s60"):
        source=json.loads((ASSETS/f"{key}.json").read_text())
        prefix="Quincy 10GC" if key=="quincy" else "NanoZoomer S60 C13210-01"
        joint=next(j for j in joints if j["kind"] in (1,2) and j["document"].startswith(prefix))
        rot=np.eye(3) if key=="quincy" else np.array([[1,0,0],[0,0,-1],[0,1,0]])
        vs=[np.array(p["vertices"]).reshape(-1,3)@rot.T for p in source["parts"]]
        whole=np.concatenate(vs);lo=whole.min(0);hi=whole.max(0)
        offset=np.array([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]])
        pivot=rot@np.array(joint["origin_m"])-offset
        chunks=collections.defaultdict(list);counts=collections.Counter()
        for p,v in zip(source["parts"],vs):
            moving=p["name"].startswith(joint["one"]+"/")
            group="door" if moving else "fixed"
            counts[group]+=1
            rgba=tuple(round(x,3) for x in p["rgba"])
            chunks[(group,rgba)].append((v-offset-(pivot if moving else 0),np.array(p["triangles"]).reshape(-1,3)))
        meshes=collections.defaultdict(list)
        for i,((group,rgba),parts) in enumerate(chunks.items()):
            name=f"passive_{key}_{i}";path=ASSETS/(name+".obj")
            with path.open("w") as f:
                index=1
                for v,faces in parts:
                    f.writelines("v "+vec(point)+"\n" for point in v)
                    f.writelines("f "+" ".join(str(int(x)+index) for x in face)+"\n" for face in faces)
                    index+=len(v)
            el(root.find("asset"),"mesh",name=name,file="assets/"+path.name,inertia="shell")
            meshes[group].append((name,rgba))
        for placement in layout["placements"]:
            if placement["asset"]!=key:continue
            name=placement["name"];body=root.find(f"worldbody/body[@name='{name}']")
            for geom in list(body.findall("geom")):
                if geom.get("mesh"):body.remove(geom)
            door=el(body,"body",name=name+"_passive_door",pos=vec(pivot))
            el(door,"joint",name=name+"_door_joint",type="hinge" if joint["kind"]==1 else "slide",
               axis=vec(rot@np.array(joint["axis"])),range=vec(joint["limits"]),ref=str(joint["value"]),
               damping=".3",frictionloss=".05")
            el(door,"inertial",pos="0 0 0",mass="1",diaginertia=".02 .02 .02")
            for group,parent in [("fixed",body),("door",door)]:
                for mesh,rgba in meshes[group]:
                    el(parent,"geom",type="mesh",mesh=mesh,rgba=vec(rgba),contype="0",conaffinity="0",group="1",density="0")
            rows.append(dict(machine=name,joint=name+"_door_joint",cad_joint=joint["name"],
                passive=True,actuator=False,axis_local=(rot@np.array(joint["axis"])).tolist(),
                limits=joint["limits"],cad_pose_qpos0=joint["value"],parts=dict(counts)))
    report=dict(doors=rows,limitations=["Door masses (1 kg), inertia, damping and hinge friction are placeholders.",
        "Exported geometry and pivot preserved; no door collision or handle-contact validation.",
        "Whole-machine envelope colliders still block insertion. Do NOT enable full-lab physics yet.",
        "S60 CAD door stroke is marked provisional; not manufacturer validation."])
    (ROOT/"passive_door_articulation.json").write_text(json.dumps(report,indent=2))
