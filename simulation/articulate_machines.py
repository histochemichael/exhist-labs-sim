"""Import every source workstation joint, retaining hidden configurations."""
import json,collections,re
import numpy as np
from build_scene import ROOT,ASSETS,el,vec

def add_machines(root):
    src=json.loads((ASSETS/"leica_workstation_handled.json").read_text())
    kin=json.loads((ASSETS/"leica_kinematics.json").read_text())
    uf={}
    def find(x):
        uf.setdefault(x,x)
        if uf[x]!=x:uf[x]=find(uf[x])
        return uf[x]
    def union(a,b):uf[find(a)]=find(b)
    for g in kin["groups"]:
        for p in g["occurrences"]:union(p,g["occurrences"][0])
    for j in kin["joints"]:
        assert j["kind"] in [0,1,2,5],j
        assert not j["suppressed"],"Suppressed joint needs explicit treatment"
        if j["kind"]==0:union(j["one"],j["two"])
        else:find(j["one"]);find(j["two"])
    moving=[j for j in kin["joints"] if j["kind"]!=0]
    keys={}
    for i,j in enumerate(moving):
        key="j"+str(i)+"_"+re.sub(r"[^a-zA-Z0-9_]","_",j["name"])
        assert find(j["one"]) not in keys,"Two moving joints drive one rigid assembly"
        keys[find(j["one"])]=key;j["key"]=key
    bykey={j["key"]:j for j in moving}
    def owner(path):
        candidates=sorted((p for p in uf if path==p or path.startswith(p+"+")),key=len,reverse=True)
        return next((keys[find(p)] for p in candidates if find(p) in keys),"fixed")
    parents={j["key"]:owner(j["two"]) for j in moving}
    for k,p in parents.items():assert k!=p,(k,"self cycle")
    vertices=[np.asarray(p["vertices"]).reshape(-1,3) for p in src["parts"]]
    whole=np.concatenate(vertices);lo=whole.min(0);hi=whole.max(0)
    offset=np.array([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]])
    origins={"fixed":np.zeros(3),**{j["key"]:np.array(j["origin"])-offset for j in moving}}
    chunks=collections.defaultdict(list);counts=collections.Counter();partowners={}
    for p,v in zip(src["parts"],vertices):
        key=owner(p["name"].rsplit("/",1)[0]);counts[key]+=1;partowners[p["name"]]=key
        rgba=tuple(round(c,3) for c in p["rgba"])
        # Separate enclosure shells so inspection mode can see internal motion.
        shell=any(s in p["name"] for s in ["transparent_hood","02_OUTER_ENCLOSURE","FIXED CHASSIS","FIXED LID","COVER","HOUSING"])
        chunks[(key,rgba,shell)].append((v-origins[key]-offset,np.asarray(p["triangles"]).reshape(-1,3)))
    assets=root.find("asset");world=root.find("worldbody");meshmap=collections.defaultdict(list)
    for i,((key,rgba,shell),pieces) in enumerate(chunks.items()):
        name=f"art_leica_{i}";path=ASSETS/(name+".obj")
        with path.open("w") as f:
            idx=1
            for vs,fs in pieces:
                f.writelines("v "+vec(v)+"\n" for v in vs)
                f.writelines("f "+" ".join(str(int(n)+idx) for n in face)+"\n" for face in fs);idx+=len(vs)
        el(assets,"mesh",name=name,file="assets/"+path.name,inertia="shell")
        meshmap[key].append((name,rgba,shell))
    eq=root.find("equality");records=[]
    def subtree_parts(key):
        return counts[key]+sum(subtree_parts(child) for child,parent in parents.items() if parent==key)
    for station in ["routine_a","routine_b"]:
        base=world.find(f"body[@name='{station}_workstation']")
        # Retain conservative static envelope for external Nori checks only.
        for c in list(base):
            if not c.get("name","").endswith("_envelope"):base.remove(c)
        bodies={"fixed":base};pending=set(bykey)
        while pending:
            ready=[k for k in bykey if k in pending and parents[k] in bodies]
            assert ready,("CAD joint graph cycle",pending)
            for key in ready:
                j=bykey[key];parent=parents[key]
                body=el(bodies[parent],"body",name=station+"_"+key,pos=vec(origins[key]-origins[parent]))
                el(body,"inertial",mass=".1",pos="0 0 0",diaginertia=".001 .001 .001")
                from drawer_pulls import add_pull
                add_pull(root,body,j,station)
                names=[];dofrows=[]
                for i,dof in enumerate(j["dofs"]):
                    name=station+"_"+key+"_"+str(i);names.append(name)
                    low,high=dof["limits"];value=dof["value"]
                    locked=low is not None and high is not None and abs(high-low)<1e-10
                    kw=dict(name=name,type=dof["kind"],axis=vec(dof["axis"]),damping="2")
                    if low is not None and high is not None and not locked:kw["range"]=vec([low-value,high-value])
                    else:kw["limited"]="false"
                    el(body,"joint",**kw)
                    if locked:el(eq,"joint",name=name+"_locked",joint1=name,polycoef=vec([low-value,0,0,0,0]))
                    dofrows.append(dict(name=name,kind=dof["kind"],limits_delta=[None if low is None else low-value,None if high is None else high-value],locked=locked))
                records.append(dict(station=station,cad_name=j["name"],cad_context=j["context"],
                    body=station+"_"+key,parent=station+"_"+parent,dofs=dofrows,
                    visible_parts=counts[key],subtree_visible_parts=subtree_parts(key),hidden_reference=subtree_parts(key)==0,
                    device="ST5020" if j["one"].startswith("ST5020") else "CV5030"))
                bodies[key]=body;pending.remove(key)
        for key in bodies:
            for mesh,rgba,shell in meshmap[key]:
                el(bodies[key],"geom",name=station+"_"+mesh+("_shell" if shell else ""),type="mesh",mesh=mesh,rgba=vec(rgba),contype="0",conaffinity="0",density="0",group="1")
        for device,dx in [("ST5020",.35),("CV5030",-.45)]:
            el(base,"site",name=station+"_"+device+"_activity",pos=vec([dx,-.39,.4]),size=".018",rgba=".2 .4 .3 1")
    report=dict(source=kin["document"],source_joint_count=len(kin["joints"]),rigid_joints=[j for j in kin["joints"] if j["kind"]==0],
        movable_joints_per_workstation=len(moving),dofs_per_workstation=sum(len(j["dofs"]) for j in moving),
        joint_map=records,parts_by_link=dict(counts),part_owners=partowners,
        limitations=["CAD estimate geometry and ranges, not manufacturer-certified kinematics.",
          "Fixed joints represented as rigid assemblies. Locked planar rotation retained with equality constraints.",
          "Hidden reference joints retained without restoring hidden orange proxies or optional meshes.",
          "Mass/inertia placeholders; kinematic preview only; no internal collision validation."])
    (ROOT/"machine_articulation.json").write_text(json.dumps(report,indent=2))
    from machine_glazing import apply as apply_glazing
    apply_glazing(root)
    from drawer_pulls import manifest
    manifest()
    print("Machine CAD joints:",len(kin["joints"]),"per workstation;",len(records),"moving assemblies across two copies.")
