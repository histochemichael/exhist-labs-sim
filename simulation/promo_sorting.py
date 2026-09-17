"""Indexed-slide sorting choreography for the promo; geometric playback only."""
import json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco
from first_pass import ik,kinematics,GRASP_ROTATION,yaw_matrix,smooth

ROOT=Path(__file__).resolve().parent
OFFSET=np.array([.0185250499,.0839950024,.0000139999995])
BASES=np.array([[-6.12,.45,.8],[-5.93,.45,.8]])
ROT=yaw_matrix(np.pi)@GRASP_ROTATION
GRIP=np.array([0,0,.033]);HEIGHT=.130
def vec(v):return ' '.join(str(float(x)) for x in np.ravel(v))
def quat(R):
    q=np.zeros(4);mujoco.mju_mat2Quat(q,np.ascontiguousarray(R).ravel());return q
def parts():return json.loads((ROOT/'assets/leica_rack_handled.json').read_text())['parts']
def centers():
    return np.array([(np.array(p['vertices']).reshape(-1,3).min(0)+np.array(p['vertices']).reshape(-1,3).max(0))/2-OFFSET for p in parts()[10:14]])

def build_sorting(root):
    a=root.find('asset');w=root.find('worldbody');cad=parts();cs=centers()
    for i,p in enumerate([cad[0],cad[-1]]):
        n=f'promo_sort_rack_{i}';ET.SubElement(a,'mesh',name=n,vertex=vec(np.array(p['vertices']).reshape(-1,3)-OFFSET),face=' '.join(map(str,p['triangles'])),inertia='shell')
    for k,base in enumerate(BASES):
        b=ET.SubElement(w,'body',name=f'promo_sort_rack_body_{k}',pos=vec(base))
        for i in range(2):ET.SubElement(b,'geom',name=f'promo_sort_rack_{k}_{i}',type='mesh',mesh=f'promo_sort_rack_{i}',rgba='.66 .67 .68 1',contype='0',conaffinity='0',density='0',group='1')
    for i,p in enumerate(cad[10:14]):
        n=f'promo_sort_slide_{i}';v=np.array(p['vertices']).reshape(-1,3)-OFFSET-cs[i]
        ET.SubElement(a,'mesh',name=n,vertex=vec(v),face=' '.join(map(str,p['triangles'])),inertia='shell')
        b=ET.SubElement(w,'body',name=n,mocap='true',pos=vec(BASES[0]+cs[i]))
        ET.SubElement(b,'geom',name=n,type='mesh',mesh=n,rgba='.46 .76 .83 1',contype='0',conaffinity='0',density='0',group='1')

class Sorting:
    def __init__(self,m,d):
        self.m=m;self.d=d;self.cs=centers();self.names=['sorter_2_carriage']+['sorter_2_j'+str(i) for i in range(1,6)];self.qa=[m.joint(n).qposadr[0] for n in self.names]
        self.mid=[m.body(f'promo_sort_slide_{i}').mocapid[0] for i in range(4)];self.tables=[];self.errors=[]
        scratch=mujoco.MjData(m);scratch.qpos[:]=d.qpos
        for k in range(4):
            src=BASES[0]+self.cs[k]+GRIP;dst=BASES[1]+self.cs[k]+GRIP;nextsrc=BASES[0]+self.cs[min(3,k+1)]+GRIP
            ts=[0,2,3,5,8,10,11,13,16];ps=[src+[0,0,HEIGHT],src,src,src+[0,0,HEIGHT],dst+[0,0,HEIGHT],dst,dst,dst+[0,0,HEIGHT],nextsrc+[0,0,HEIGHT]]
            qs=[]
            for t in np.linspace(0,16,161):
                j=min(np.searchsorted(ts,t,side='right')-1,len(ts)-2);u=smooth((t-ts[j])/(ts[j+1]-ts[j]));p=ps[j]*(1-u)+ps[j+1]*u
                q,pe,re=ik(m,scratch,'sorter_2_slide_groove',self.names,p,ROT,attempts=16 if t==0 else 3,iterations=220)
                if pe>.001 or re>.01:raise RuntimeError(('sorter pose',k,t,pe,re))
                qs.append(q);self.errors.append([float(pe),float(re)])
            self.tables.append(np.array(qs))
        print('Background sorter: four indexed slide transfers planned',flush=True)

    def update(self,t):
        m=self.m;d=self.d;elapsed=max(0,t-3);k=min(3,int(elapsed//16));u=min(16,elapsed-k*16)
        f=u*10;lo=min(int(f),160);hi=min(lo+1,160);q=(1-f+lo)*self.tables[k][lo]+(f-lo)*self.tables[k][hi];d.qpos[self.qa]=q
        jaw=0 if u<2 else -.0335*smooth(u-2) if u<3 else -.0335 if u<10 else -.0335*(1-smooth(u-10)) if u<11 else 0
        for n in ['sorter_2_jaw_left','sorter_2_jaw_right']:d.qpos[m.joint(n).qposadr[0]]=jaw
        d.qpos[m.joint('sorter_2_pinion').qposadr[0]]=jaw/.0072;d.qpos[m.joint('sorter_2_screw').qposadr[0]]=q[0]*2*np.pi/.008
        kinematics(m,d)
        positions=[];rotations=[]
        for i in range(4):
            done=(i<k or i==k and u>=10);p=BASES[int(done)]+self.cs[i];R=np.eye(3)
            if i==k and 3<=u<10:
                s=d.site('sorter_2_slide_groove');R=s.xmat.reshape(3,3)@ROT.T;p=s.xpos-R@GRIP
            d.mocap_pos[self.mid[i]]=p;d.mocap_quat[self.mid[i]]=quat(R);positions.append(p.copy());rotations.append(quat(R))
        return np.array(positions),np.array(rotations)
