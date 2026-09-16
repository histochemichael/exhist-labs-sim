"""Printed-gripper mass estimates and groove-preserving CAD contact partitions.

Original visual CAD is unchanged. Mass values are construction estimates, not a
weighing. Finger contact prisms preserve cross-section recesses instead of filling
them with one convex hull. The distal 1 mm rounded tip and proximal housing are
not covered by these finger contact partitions.
"""
import copy,json,math,struct
import numpy as np
import xml.etree.ElementTree as ET
from build_scene import ROOT,el,vec

SOURCE=ROOT/'source/nori/Nori with parallel histo gripper.xml'
REGIONS={'slide':(-.1211885,-.1021885),'bridge':(-.1021885,-.1001885),'handle':(-.1001885,-.0921885),'upper':(-.0921885,-.0721885)}
FRAMES={'handle':[0,-.0961885,.0933],'slide':[0,-.1116885,.0933]}
def cross2(a,b):return a[0]*b[1]-a[1]*b[0]

def triangles(index):
    raw=(SOURCE.parent/f'lehisto_meshes/lehisto_part_{index:02d}.stl').read_bytes()
    count=struct.unpack_from('<I',raw,80)[0]
    return np.array([struct.unpack_from('<9f',raw,84+i*50+12) for i in range(count)]).reshape(-1,3,3)

def section(tris,y):
    edges=[]
    for tri in tris:
        pts=[]
        for a,b in zip(tri,np.roll(tri,-1,axis=0)):
            if (a[1]-y)*(b[1]-y)<0:
                p=a+(b-a)*(y-a[1])/(b[1]-a[1]);pts.append(tuple(np.round(p[[0,2]],8)))
        if len(pts)==2 and pts[0]!=pts[1]:edges.append(pts)
    graph={}
    for a,b in edges:graph.setdefault(a,set()).add(b);graph.setdefault(b,set()).add(a)
    assert all(len(v)==2 for v in graph.values()),'Non-manifold CAD section'
    first=min(graph);poly=[first];previous=None;current=first
    while True:
        nxt=next(p for p in graph[current] if p!=previous)
        if nxt==first:break
        poly.append(nxt);previous,current=current,nxt
        assert len(poly)<=len(graph)
    assert len(poly)==len(graph),'Multiple section loops need explicit handling'
    p=np.array(poly)
    changed=True
    while changed:
        changed=False
        for i in range(len(p)):
            a=p[i-1];b=p[i];c=p[(i+1)%len(p)]
            if abs(cross2(b-a,c-b))<1e-11:
                p=np.delete(p,i,axis=0);changed=True;break
    if sum(cross2(a,b) for a,b in zip(p,np.roll(p,-1,axis=0)))<0:p=p[::-1]
    return p

def triangulate(p):
    ids=list(range(len(p)));out=[]
    while len(ids)>3:
        found=False
        for k,b in enumerate(ids):
            a=ids[k-1];c=ids[(k+1)%len(ids)]
            if cross2(p[b]-p[a],p[c]-p[b])<=1e-12:continue
            def inside(q):return all(cross2(v-u,q-u)>=-1e-12 for u,v in [(p[a],p[b]),(p[b],p[c]),(p[c],p[a])])
            if any(inside(p[i]) for i in ids if i not in (a,b,c)):continue
            out.append([a,b,c]);ids.pop(k);found=True;break
        assert found,'CAD section triangulation failed'
    out.append(ids);return out

def add_contacts(root,parent,index,prefix,rotation=None,offset=None):
    r=np.eye(3) if rotation is None else rotation;o=np.zeros(3) if offset is None else offset
    count=0;profiles={}
    for region,(low,high) in REGIONS.items():
        p=section(triangles(index),(low+high)/2);profiles[region]=p.tolist()
        for face in triangulate(p):
            v=np.array([[p[i,0],y,p[i,1]] for y in (low,high) for i in face])@r.T+o
            name=f'{prefix}_{region}_{count}'
            el(root.find('asset'),'mesh',name=name,vertex=vec(v.ravel()))
            el(parent,'geom',name=name,type='mesh',mesh=name,contype='2',conaffinity='1',group='3',density='0',
               rgba='0 .8 .5 0',condim='4',friction='.6 .005 .0001',solref='.004 1',solimp='.95 .99 .001')
            count+=1
    return dict(convex_parts=count,profiles_xz_m=profiles)

def mass_properties(v,mass):
    a,b,c=v[:,0],v[:,1],v[:,2];vol=np.einsum('ij,ij->i',a,np.cross(b,c))/6
    total=vol.sum();s=a+b+c;com=(vol[:,None]*s).sum(0)/(4*total)
    second=sum(vol[i]*(sum(np.outer(q,q) for q in v[i])+np.outer(s[i],s[i])) for i in range(len(v)))/(20*total)
    covariance=second-np.outer(com,com)
    inertia=mass*(np.trace(covariance)*np.eye(3)-covariance)
    assert np.linalg.eigvalsh(inertia).min()>0
    return com,inertia,abs(total)

def build_mass_model():
    bom=json.loads((ROOT/'assets/gripper_bom_cad.json').read_text())
    servo=list(range(2,13));camera=list(range(21,31));printed=[0,14,15,17,19,20,31];rods=[1,16]
    masses=[];rows=[]
    for i,p in enumerate(bom):
        if i in servo:mass=.055*p['volume_cm3']/sum(bom[j]['volume_cm3'] for j in servo);basis='STS3215 assembly: 55 g datasheet'
        elif i in camera:mass=.035*p['volume_cm3']/sum(bom[j]['volume_cm3'] for j in camera);basis='B0332 assembly: assumed 35 g, not a sourced net weight'
        elif i in printed:mass=p['volume_cm3']*1.24/1000;basis='printed part, conservative solid PLA at 1.24 g/cm3'
        elif i in rods:mass=math.pi/4*(.006**2-.004**2)*.125*2700;basis='aluminum tube D6 x assumed D4 ID x 125 mm'
        else:mass=p['mass_kg'];basis='bearing/fastener CAD steel retained'
        masses.append(mass);rows.append(dict(mesh=f'lehisto_part_{i:02d}',source=p['path'],mass_kg=mass,basis=basis))
    groups={'right_wrist_roll_link':[i for i in range(45) if i not in (0,14,15,17,31)],
            'lehisto_jaw_a':[0,14],'lehisto_jaw_b':[15,17],'lehisto_pinion':[31]}
    original=ET.parse(SOURCE).getroot()
    # Resolve actual body names from source, not a guessed jaw naming convention.
    groups={('right_wrist_roll_link' if name=='right_wrist_roll_link' else
             original.find(f".//joint[@name='{name+'_slide' if 'jaw' in name else name+'_joint'}']/..").get('name')):indices
            for name,indices in groups.items()}
    inertials={}
    for name,indices in groups.items():
        parts=[(masses[i],*mass_properties(triangles(i),masses[i])[:2]) for i in indices]
        if name=='right_wrist_roll_link':parts.append((.015,np.zeros(3),np.eye(3)*1e-6))
        mass=sum(p[0] for p in parts);com=sum(m*c for m,c,_ in parts)/mass
        inertia=sum(I+m*(np.dot(c-com,c-com)*np.eye(3)-np.outer(c-com,c-com)) for m,c,I in parts)
        if name=='right_wrist_roll_link':
            r=np.array([[0,0,-1],[1,0,0],[0,-1,0]])
            com=r@com;inertia=r@inertia@r.T
        inertials[name]=dict(mass=mass,pos=com.tolist(),fullinertia=[inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]])
    printed_volume=sum(bom[i]['volume_cm3'] for i in printed)
    report=dict(total_kg=sum(masses)+.015,body_inertials=inertials,parts=rows,
        old_all_steel_CAD_kg=sum(p['mass_kg'] for p in bom),printed_volume_cm3=printed_volume,
        conservative_scenario_kg=printed_volume*.0014+.056+.050+2*math.pi*.003**2*.125*2700+.050+sum(masses[i] for i in range(45) if i not in servo+camera+printed+rods),
        sources={'servo':'https://www.feetechrc.com/products.html?keyword=STS3215',
        'PLA':'https://polymaker.com/wp-content/tech-docs/PolyLite_PLA_PIS_EN_V1.1.pdf',
        'camera':'https://www.welectron.com/mediafiles/productimg/arducam/Amazon/B0332_OV9281_Global_Shutter_UVC_Camera_Datasheet.pdf'},
        assumptions=['Filament identity/infill not measured; solid PLA is the nominal model.',
            'Tube ID assumed 4 mm; CAD currently models solid D6 rods. Aluminum density assumed 2700 kg/m3.',
            'B0332 manufacturer datasheet reviewed but no net mass found; 35 g nominal / 50 g conservative allowance.',
            '15 g nominal wiring/attachment allowance at wrist; 50 g in conservative scenario.',
            'Servo/camera internal mass distributed by CAD volume; tube mass uses CAD solid-rod shape moments. Inertias approximate.'])
    (ROOT/'gripper_mass_model.json').write_text(json.dumps(report,indent=2));return report

def apply_nori(root):
    report=build_mass_model()
    for name,values in report['body_inertials'].items():
        body=root.find(f".//body[@name='{name}']")
        old=body.find('inertial')
        if old is not None:body.remove(old)
        el(body,'inertial',mass=values['mass'],pos=vec(values['pos']),fullinertia=vec(values['fullinertia']))
    records={}
    for side,index in [('a',14),('b',15)]:
        geom=root.find(f".//geom[@name='lehisto_part_{index}_contact']")
        parent=root.find(f".//geom[@name='lehisto_part_{index}_contact']/..")
        if geom is not None:parent.remove(geom)
        records[side]=add_contacts(root,parent,index,'nori_groove_'+side)
    mount=root.find(".//body[@name='lehisto_mount']")
    for name,pos in FRAMES.items():el(mount,'site',name='nori_right_'+name+'_groove',pos=vec(pos),size='.002',rgba='1 .5 0 1')
    (ROOT/'groove_contact_model.json').write_text(json.dumps(dict(jaws=records,frames_mount_m=FRAMES,
        scope='CAD cross-section convex partitions; original printed geometry retained; not full robot collision validation'),indent=2))

if __name__=='__main__':
    print(json.dumps({k:v for k,v in build_mass_model().items() if k not in ('parts','body_inertials')},indent=2))
    for name,(a,b) in REGIONS.items():print(name,section(triangles(14),(a+b)/2).tolist())
