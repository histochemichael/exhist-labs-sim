"""Thin convex prisms on actual tapered CAD wall faces; cavities stay hollow."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parent
CAD_SHIFT=np.array([.6,.0728575,0])
BACK_RACK=np.array([-2.817075,.7971425,.934314]) # first vessel in back row (13)

def orthogonal_boxes(part):
    v=np.array(part['vertices']).reshape(-1,3);tri=v[np.array(part['triangles']).reshape(-1,3)]
    coords=[np.unique(np.round(v[:,k],7)) for k in range(3)]
    e1=tri[:,1]-tri[:,0];e2=tri[:,2]-tri[:,0];normal=np.cross(e1,e2)
    nonzero=np.abs(normal)>1e-12
    if np.any(nonzero.sum(1)>1):raise ValueError(('Non-orthogonal CAD',part['name']))
    shape=tuple(len(x)-1 for x in coords);occ=np.zeros(shape,bool)
    use=np.abs(normal[:,0])>1e-12;tr=tri[use]
    a=tr[:,0,1:];b=tr[:,1,1:];c=tr[:,2,1:];ab=b-a;ac=c-a;det=ab[:,0]*ac[:,1]-ab[:,1]*ac[:,0]
    for j in range(shape[1]):
        for k in range(shape[2]):
            p=np.array([(coords[1][j]+coords[1][j+1])/2+1.7e-9,(coords[2][k]+coords[2][k+1])/2+2.3e-9]);ap=p-a
            u=(ap[:,0]*ac[:,1]-ap[:,1]*ac[:,0])/det;w=(ab[:,0]*ap[:,1]-ab[:,1]*ap[:,0])/det
            xs=np.unique(np.round(tr[(u>=0)&(w>=0)&(u+w<=1),0,0],7))
            for i in range(shape[0]):occ[i,j,k]=np.count_nonzero(xs>(coords[0][i]+coords[0][i+1])/2)%2==1
    out=[]
    while occ.any():
        lo=np.array(np.argwhere(occ)[0]);hi=lo+1
        for axis in range(3):
            while hi[axis]<shape[axis]:
                test=hi.copy();test[axis]+=1
                if not occ[tuple(slice(lo[a],test[a]) for a in range(3))].all():break
                hi=test
        occ[tuple(slice(lo[a],hi[a]) for a in range(3))]=False
        out.append((np.array([coords[a][lo[a]] for a in range(3)]),np.array([coords[a][hi[a]] for a in range(3)])))
    volume=sum(np.prod(b-a) for a,b in out);meshvol=abs(np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2])).sum()/6)
    if abs(volume-meshvol)>1e-10:raise ValueError(('CAD solid volume mismatch',part['name'],volume,meshvol))
    return out

def build_container_checks(root):
    parts=json.loads((ROOT/'assets/leica_workstation_handled.json').read_text())['parts'];counts={};cache={};assets=root.find('asset')
    for i in list(range(467,471))+list(range(534,562)):
        p=parts[i]
        if 'vessel' not in p['name'].lower() and 'container' not in p['name'].lower():continue
        moving=i<471
        b=root.find(".//body[@name='routine_a_j66_LOAD_DRAWER_OPEN_mm']") if moving else root.find("worldbody/body[@name='routine_a_workstation']")
        origin=np.array([.495075,-.3648575,.0687])+CAD_SHIFT if moving else CAD_SHIFT
        v=np.array(p['vertices']).reshape(-1,3);center=(v.min(0)+v.max(0))/2;tri=(v-center)[np.array(p['triangles']).reshape(-1,3)]
        holder=ET.SubElement(b,'body',name=f'promo_vessel_{i}_surfaces',pos=' '.join(map(str,center-origin)))
        counts[p['name']]=len(tri)
        for k,t in enumerate(tri):
            normal=np.cross(t[1]-t[0],t[2]-t[0]);norm=np.linalg.norm(normal)
            if norm<1e-12:continue
            # 10-micrometre thickness avoids degenerate zero-volume mesh hulls.
            # Clearance acceptance uses millimetres, far above this inflation.
            normal=normal/norm*.000005;vv=np.vstack([t-normal,t+normal]);key=tuple(sorted(map(tuple,np.round(t,7))))
            if key not in cache:
                name=f'promo_wall_face_{len(cache)}';cache[key]=name
                ET.SubElement(assets,'mesh',name=name,vertex=' '.join(map(str,vv.ravel())),face='0 2 1 3 4 5 0 1 4 0 4 3 1 2 5 1 5 4 2 0 3 2 3 5',inertia='shell')
            ET.SubElement(holder,'geom',name=f'promo_vessel_{i}_{k}',type='mesh',mesh=cache[key],rgba='0 0 0 0',contype='0',conaffinity='0',density='0',group='5')
    return counts

class VesselClearance:
    def __init__(self,m,d):
        self.m=m;self.d=d;self.ft=np.zeros(6)
        self.walls=np.array([g for g in range(m.ngeom) if m.geom(g).name.startswith('promo_vessel_')])
        self.fingers=[g for g in range(m.ngeom) if m.geom(g).name.startswith(('nori_groove_a_','nori_groove_b_'))]
        self.other=[g for g in range(m.ngeom) if m.geom(g).name.startswith('lehisto_part_') and g not in [m.geom('lehisto_part_00').id,m.geom('lehisto_part_14').id]]
    def gaps(self,geometry=None,limit=.015):
        m=self.m;d=self.d;best=limit;pair=None
        for a in self.fingers+self.other if geometry is None else geometry:
            # Bounding-sphere broad phase is conservative; exact convex distance follows.
            ds=np.linalg.norm(d.geom_xpos[self.walls]-d.geom_xpos[a],axis=1)-m.geom_rbound[self.walls]-m.geom_rbound[a]
            for b in self.walls[ds<limit]:
                distance=mujoco.mj_geomDistance(m,d,a,int(b),limit,self.ft)
                if distance<best:best=float(distance);pair=(m.geom(a).name,m.geom(int(b)).name)
        return best,pair

if __name__=='__main__':
    root=ET.parse(ROOT/'exhist_operational.xml').getroot()
    print(json.dumps(build_container_checks(root),indent=2))
