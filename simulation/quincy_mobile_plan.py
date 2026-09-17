"""Whole-stroke arm planning with a differential-drive reverse arc, not teleportation."""
import json,math
import numpy as np
import mujoco
import xml.etree.ElementTree as ET
import equipment_access as e
from access_planning import fit

root,meta=e.build('quincy');m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
names=e.ARM+['lift_extension_joint'];ids=[m.joint(n).id for n in names];qa=m.jnt_qposadr[ids];mid=m.joint('lift_middle_joint').qposadr[0]
d.qpos[m.joint(meta['joint']).qposadr[0]]=0;mujoco.mj_forward(m,d);h=d.site_xpos[m.site('access_handle_frame').id].copy();anchor=d.xanchor[m.joint(meta['joint']).id].copy();base=m.body('mobile_chassis').id;rng=np.random.default_rng(313)
best=0
for lateral,stand,radius,gain,flip in [(x,y,r,k,f) for r,k in ((.3,1.),(.4,.75),(.3,.8),(.35,.4),(.2,1.)) for f in (0,math.pi) for x,y in ((.3,.38),(.25,.36),(.2,.38),(.35,.38),(.3,.34))]:
    origin=h+[lateral,-stand,-h[2]+.002];R=np.array([[1,0,0],[0,0,-1],[0,1,0]],float)@e.rz(flip)
    for attempt in range(15):
        seed=d.qpos.copy();seed[qa]=[1.8,1.,0,-.6,0,0,0,.48]
        if attempt:seed[qa]=rng.uniform(m.jnt_range[ids,0],m.jnt_range[ids,1]);seed[qa[-1]]=rng.uniform(.35,.65)
        path=[];worst=0;angles=0
        for t in np.linspace(0,1,41):
            q=t*meta['open_target'];turn=gain*q;pos=origin+[radius*(1-math.cos(turn)),-radius*math.sin(turn),0];yaw=math.pi/2+turn
            m.body_pos[base]=pos;m.body_quat[base]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
            sol,pe,re=fit(m,seed,e.SITE,names,anchor+e.rz(q)@(h-anchor),e.rz(q)@R,iterations=250)
            if pe>.0007 or re>.008:break
            path.append(sol.tolist());seed[qa]=sol;seed[mid]=sol[-1]/2;worst=max(worst,float(pe));angles=max(angles,float(re))
        if len(path)>best:best=len(path);print('mobile best',best,lateral,stand,radius,gain,flip,flush=True)
        if len(path)!=41:continue
        m.body_pos[base]=origin;m.body_quat[base]=[2**-.5,0,0,2**-.5];seed[qa]=path[0]
        pre,pe,re=fit(m,seed,e.SITE,names,h+[0,-.05,0],R,iterations=400)
        if pe>.001 or re>.01:continue
        plan=dict(kind='quincy',lateral=lateral,stand=stand,yaw=math.pi/2,follow=1.,orientation=R.tolist(),names=names,path=path,approach=pre.tolist(),max_error_m=worst,max_angle_error_rad=angles,grasp_depth_m=meta['grasp_depth_m'],mobile=dict(radius_m=radius,yaw_gain=gain),planner='Differential-drive reverse arc + eight-DOF arm/lift')
        (e.ROOT/'access_quincy_plan.json').write_text(json.dumps(plan,indent=2));print('FOUND',json.dumps({k:v for k,v in plan.items() if k not in ('path','approach')}),flush=True);raise SystemExit(0)
raise RuntimeError('No whole-stroke mobile access path')
