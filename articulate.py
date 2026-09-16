"""Preserve Fusion CAD rigid groups and joint axes in MuJoCo."""
import json, collections, re
import numpy as np
from build_scene import ROOT, ASSETS, el, vec

def add_robots(root):
    src=json.loads((ASSETS/"lehisto.json").read_text())
    kin=json.loads((ASSETS/"lehisto_kinematics.json").read_text())
    uf={}
    def find(a):
        uf.setdefault(a,a)
        if uf[a]!=a:uf[a]=find(uf[a])
        return uf[a]
    for g in kin["groups"]:
        first=g["occurrences"][0]
        for p in g["occurrences"]:uf[find(p)]=find(first)
    joints=kin["joints"]
    aliases={"Arm Carriage Travel":"carriage","Lead Screw Rotation - DRIVE THIS":"screw",
             "Slide_Jaw_Right":"jaw_right","Slide_Jaw_Left":"jaw_left","Drive_Pinion":"pinion"}
    for j in joints:
        j["key"]=aliases.get(j["name"],"j"+j["name"][7:8])
    maps={find(j["one"]):j["key"] for j in joints}
    maps[find(joints[0]["two"])]="fixed"
    order=["fixed","screw","carriage","j1","j2","j3","j4","j5","jaw_right","jaw_left","pinion"]
    bykey={j["key"]:j for j in joints}
    rot=np.array([[0,1,0],[-1,0,0],[0,0,1]])
    verts=[np.asarray(p["vertices"]).reshape(-1,3)@rot.T for p in src["parts"]]
    whole=np.concatenate(verts);lo=whole.min(0);hi=whole.max(0)
    offset=np.array([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]])
    origins={"fixed":np.zeros(3)}
    for key,j in bykey.items():origins[key]=rot@np.array(j["origin"])-offset
    chunks=collections.defaultdict(list);counts=collections.Counter();unmatched=[]
    for p,v in zip(src["parts"],verts):
        path=p["name"].rsplit("/",1)[0]
        candidates=[a for a in uf if path==a or path.startswith(a+"+")]
        candidates.sort(key=len,reverse=True)
        key=next((maps[find(a)] for a in candidates if find(a) in maps),None)
        if key is None:
            key="carriage" if path.startswith("Carriage:") else "j5" if path.startswith("Gripper:") else "fixed"
            unmatched.append(p["name"])
        counts[key]+=1
        rgba=tuple(round(x,3) for x in p["rgba"])
        chunks[(key,rgba)].append((v-offset-origins[key],np.asarray(p["triangles"]).reshape(-1,3)))
    assets=root.find("asset");world=root.find("worldbody");meshmap=collections.defaultdict(list)
    for i,((key,rgba),pieces) in enumerate(chunks.items()):
        name=f"art_lehisto_{i}";path=ASSETS/(name+".obj")
        with path.open("w") as f:
            idx=1
            for vs,fs in pieces:
                f.writelines("v "+vec(v)+"\n" for v in vs)
                f.writelines("f "+" ".join(str(int(n)+idx) for n in face)+"\n" for face in fs)
                idx+=len(vs)
        el(assets,"mesh",name=name,file="assets/"+path.name,inertia="shell")
        meshmap[key].append((name,rgba))
    names=["sorter_1","sorter_2","special_lehisto","sendout_lehisto_1","sendout_lehisto_2"]
    for name in names:
        base=world.find(f"body[@name='{name}']")
        for child in list(base):base.remove(child)
        bodies={"fixed":base}
        for key in order:
            if key!="fixed":
                j=bykey[key];parent=maps.get(find(j["two"]),"fixed")
                body=el(bodies[parent],"body",name=name+"_"+key,pos=vec(origins[key]-origins[parent]))
                kw=dict(name=name+"_"+key,type="hinge" if j["kind"]==1 else "slide",
                        axis=vec(rot@np.array(j["axis"])),damping="1")
                if j["minimum"] is not None and j["maximum"] is not None:
                    scale=.01 if j["kind"]==2 else 1
                    kw["range"]=vec([j["minimum"]*scale-j["value"],j["maximum"]*scale-j["value"]])
                else:kw["limited"]="false"
                el(body,"joint",**kw)
                el(body,"inertial",mass=".1",pos="0 0 0",diaginertia=".0001 .0001 .0001")
                bodies[key]=body
            for mesh,rgba in meshmap[key]:
                el(bodies[key],"geom",type="mesh",mesh=mesh,rgba=vec(rgba),contype="0",conaffinity="0",density="0",group="1")
        # Jaw-center TCP is provisional; jaw contact faces need measurement.
        tip=(np.array(bykey["jaw_left"]["origin"])+np.array(bykey["jaw_right"]["origin"]))/2
        el(bodies["j5"],"site",name=name+"_tcp",pos=vec(rot@tip-offset-origins["j5"]),size=".006",rgba="0 1 1 1")
        # Use source wrist/jaw axes, not global CAD axes, for the real grooves.
        from gripper_physics import FRAMES,add_contacts
        x=np.array(bykey['jaw_right']['axis']);x/=np.linalg.norm(x)
        z=np.array(bykey['j5']['axis']);z/=np.linalg.norm(z)
        y=np.cross(z,x);y/=np.linalg.norm(y);z=np.cross(x,y)
        wrist_basis=np.column_stack([x,y,z]);basis=rot@wrist_basis
        for mode,point in FRAMES.items():
            el(bodies['j5'],'site',name=name+'_'+mode+'_groove',pos=vec(basis@point),xyaxes=vec(np.r_[basis[:,0],basis[:,1]]),size='.002',rgba='1 .5 0 1')
        for key,index in [('jaw_right',14),('jaw_left',15)]:
            add_contacts(root,bodies[key],index,name+'_groove_'+key,basis,origins['j5']-origins[key])
    for side in ["left","right"]:
        b=world.find(f".//body[@name='{side}_wrist_roll_link']")
        if side=="right":
            b=world.find(".//body[@name='lehisto_mount']")
        el(b,"site",name="nori_"+side+"_tcp",pos="0 -.02 .095" if side=="right" else "0 0 -.075",size=".009",rgba="0 1 1 1")
    report=dict(parts_by_link=dict(counts),fallback_parts=unmatched,instances=names,
                source="Fusion LeHisto v38 as-built joints / rigid groups; slider cm converted to m",
                limitation="Legacy TCP is provisional. Separate original handle/slide groove frames and convex-partition finger contacts added; remaining robot geometry/mass remains incomplete.")
    (ROOT/"articulation.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
