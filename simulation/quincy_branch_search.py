"""Plan hinge access backwards from the difficult open pose, using real joints."""
import json,math,argparse
import numpy as np
import mujoco
import xml.etree.ElementTree as ET
import equipment_access as e
from access_planning import fit

parser=argparse.ArgumentParser();parser.add_argument('--follow',type=float,default=1.);args=parser.parse_args()
root,meta=e.build('quincy');m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
names=e.ARM+['lift_extension_joint'];ids=[m.joint(n).id for n in names];qa=m.jnt_qposadr[ids];mid=m.joint('lift_middle_joint').qposadr[0]
d.qpos[m.joint(meta['joint']).qposadr[0]]=0;mujoco.mj_forward(m,d)
h=d.site_xpos[m.site('access_handle_frame').id].copy();anchor=d.xanchor[m.joint(meta['joint']).id].copy();rng=np.random.default_rng(163)
base=m.body('mobile_chassis').id;best=0
print('handle',h,'anchor',anchor,flush=True)
for lateral,stand,yaw,follow,offset in [(x,y,a,args.follow,o) for o in (0,-30,30,-60,60) for a in (90,105,75) for x,y in ((.25,.34),(.20,.32),(.15,.30),(.30,.30),(.25,.38),(.30,.36),(.20,.38),(.35,.38))]:
    m.body_pos[base]=h+[lateral,-stand,-h[2]+.002];angle=math.radians(yaw);m.body_quat[base]=[math.cos(angle/2),0,0,math.sin(angle/2)]
    R=e.rz(math.radians(offset))@np.array([[1,0,0],[0,0,-1],[0,1,0]],float)@e.rz(math.pi)
    for attempt in range(12):
        seed=d.qpos.copy();seed[qa]=rng.uniform(m.jnt_range[ids,0],m.jnt_range[ids,1]);seed[qa[-1]]=rng.uniform(.32,.68);path=[];worst=0.;angles=0.
        for t in np.linspace(1,0,41):
            q=t*meta['open_target'];p=anchor+e.rz(q)@(h-anchor);rot=e.rz(q*follow)@R
            sol,pe,re=fit(m,seed,e.SITE,names,p,rot,iterations=250)
            if pe>.0007 or re>.008:break
            path.append(sol.tolist());seed[qa]=sol;seed[mid]=sol[-1]/2;worst=max(worst,float(pe));angles=max(angles,float(re))
        if len(path)>best:best=len(path);print('reverse best',best,lateral,stand,yaw,follow,attempt,flush=True)
        if len(path)!=41:continue
        for delta in ([0,-.02,0],[0,-.01,.02],[.02,-.005,0],[-.02,-.005,0],[0,-.005,.03]):
            pre,pe,re=fit(m,seed,e.SITE,names,h+delta,R,iterations=400)
            if pe<.001 and re<.01:break
        else:continue
        plan=dict(kind='quincy',lateral=lateral,stand=stand,yaw=angle,follow=follow,orientation=R.tolist(),names=names,path=path[::-1],approach=pre.tolist(),max_error_m=worst,max_angle_error_rad=angles,planner='reverse branch search',initial_wrist_yaw_offset_deg=offset,approach_delta_m=delta)
        (e.ROOT/'access_quincy_plan.json').write_text(json.dumps(plan,indent=2));print('FOUND',json.dumps({k:v for k,v in plan.items() if k not in ('path','approach')}),flush=True);raise SystemExit(0)
raise RuntimeError('No continuous full hinge path found')
