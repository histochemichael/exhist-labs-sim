"""Inspect CAD finger side profiles; no model or geometry changes."""
import numpy as np,struct,json
from pathlib import Path
import xml.etree.ElementTree as ET
from gripper_physics import SOURCE,cross2,triangulate
from left_claw_contacts import clipped

def profile(tris,z):
    edges=[]
    for tri in tris:
        pts=[]
        for a,b in zip(tri,np.roll(tri,-1,axis=0)):
            if (a[2]-z)*(b[2]-z)<0:
                p=a+(b-a)*(z-a[2])/(b[2]-a[2]);pts.append(tuple(np.round(p[:2],8)))
        if len(pts)==2 and pts[0]!=pts[1]:edges.append(pts)
    graph={}
    for a,b in edges:graph.setdefault(a,set()).add(b);graph.setdefault(b,set()).add(a)
    assert all(len(v)==2 for v in graph.values())
    unused=set(graph);loops=[]
    while unused:
        first=min(unused);p=[first];previous=None;current=first
        while True:
            unused.discard(current);nxt=next(t for t in graph[current] if t!=previous)
            if nxt==first:break
            p.append(nxt);previous,current=current,nxt
        loops.append(np.array(p))
    p=max(loops,key=lambda p:abs(sum(cross2(a,b) for a,b in zip(p,np.roll(p,-1,axis=0)))))
    p=clipped(p,1,.04,True)
    changed=True
    while changed:
        changed=False
        for i in range(len(p)):
            a=p[i-1];b=p[i];c=p[(i+1)%len(p)]
            if abs(cross2(b-a,c-a))/max(np.linalg.norm(c-a),1e-9)<1e-6:
                p=np.delete(p,i,axis=0);changed=True;break
    if sum(cross2(a,b) for a,b in zip(p,np.roll(p,-1,axis=0)))<0:p=p[::-1]
    return p

if __name__=='__main__':
    r=ET.parse(SOURCE).getroot()
    for name in ['gripper_r_mirrored','gripper_l_mirrored']:
        mesh=r.find(f"asset/mesh[@name='{name}']");raw=(SOURCE.parent/mesh.get('file')).read_bytes();n=struct.unpack_from('<I',raw,80)[0];tris=np.array([struct.unpack_from('<9f',raw,84+50*i+12) for i in range(n)]).reshape(-1,3,3)*np.fromstring(mesh.get('scale'),sep=' ')
        for z in [-.040,-.030,-.015,0.,.010]:
            p=profile(tris,z);print(name,z,len(p),p.min(0),p.max(0),sum(cross2(a,b) for a,b in zip(p,np.roll(p,-1,axis=0)))/2,len(triangulate(p)))
        print('profile',profile(tris,-.015).tolist())
