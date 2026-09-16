"""Offline whole-stroke reach planning. Never writes a live simulation state."""
import math,json
import numpy as np
import mujoco
from pose_control import rotation_error

def fit(m,initial,site,names,p,R,reference=None,iterations=120):
    d=mujoco.MjData(m);d.qpos[:]=initial
    ids=np.array([m.joint(n).id for n in names]);qa=m.jnt_qposadr[ids];va=m.jnt_dofadr[ids]
    lo=m.jnt_range[ids,0].copy()+.008;hi=m.jnt_range[ids,1].copy()-.008
    li=names.index('lift_extension_joint') if 'lift_extension_joint' in names else None
    if li is not None:lo[li]=.30;hi[li]=.69
    sid=m.site(site).id;jp=np.zeros((3,m.nv));jr=jp.copy()
    ref=np.clip(initial[qa] if reference is None else reference,lo+1e-7,hi-1e-7)
    def fun(x):
        d.qpos[qa]=x
        if li is not None:d.qpos[m.joint('lift_middle_joint').qposadr[0]]=x[li]/2
        mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
        return np.r_[d.site_xpos[sid]-p,.10*rotation_error(d.site_xmat[sid].reshape(3,3),R),.00015*(x-ref)]
    def jac(x):
        fun(x);mujoco.mj_jacSite(m,d,jp,jr,sid)
        return np.vstack([jp[:,va],.10*jr[:,va],np.eye(len(names))*.00015])
    x=ref.copy();best=x.copy();score=float('inf')
    for _ in range(iterations):
        e=fun(x);s=float(e@e)
        if s<score:best=x.copy();score=s
        if np.linalg.norm(e[:3])<.00015 and np.linalg.norm(e[3:6])<.00015:break
        j=jac(x);free=np.ones(len(names),dtype=bool);delta=np.zeros(len(names))
        # Re-solve in the tangent space of active joint stops. Simply clipping a
        # Newton step can falsely label a reachable pose as unreachable.
        for _active in range(len(names)):
            jj=j[:,free];delta[:]=0;delta[free]=-np.linalg.solve(jj.T@jj+np.eye(free.sum())*1e-5,jj.T@e)
            blocked=((x<=lo+1e-6)&(delta<0))|((x>=hi-1e-6)&(delta>0))
            if not np.any(blocked&free):break
            free[blocked]=False
            if not free.any():break
        delta=np.clip(delta,-.08,.08)
        improved=False
        for scale in (1,.5,.2,.05):
            trial=np.clip(x+scale*delta,lo,hi);ee=fun(trial)
            if ee@ee<s:x=trial;improved=True;break
        if not improved:break
    e=fun(best);return best,np.linalg.norm(e[:3]),np.linalg.norm(e[3:6])/.1

def search(kind,target_override=None,follow_override=None):
    from equipment_access import build,ARM,SITE,rz
    import xml.etree.ElementTree as ET
    root,meta=build(kind);m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    if target_override is not None:meta['open_target']=target_override
    names=ARM+['lift_extension_joint'];qa=np.array([m.joint(n).qposadr[0] for n in names]);mid=m.joint('lift_middle_joint').qposadr[0]
    jq=m.joint(meta['joint']).qposadr[0];d.qpos[jq]=0;d.qpos[qa]=[1.8,1.,0,-.6,0,0,0,meta['lift_target']];d.qpos[mid]=meta['lift_target']/2
    mujoco.mj_forward(m,d);h=d.site_xpos[m.site('access_handle_frame').id].copy();axis=d.xaxis[m.joint(meta['joint']).id].copy();anchor=d.xanchor[m.joint(meta['joint']).id].copy()
    base=m.body('mobile_chassis').id;R=np.array(meta['orientation']);rng=np.random.default_rng(19);best=None
    candidates=[(x,y,f,k,yaw,branch) for yaw in ((90,105,75,120,60) if kind=='quincy' else (90,)) for k in ((follow_override,) if follow_override is not None else ((0.,.5,1.) if kind=='quincy' else (1.,))) for f in ((math.pi,0) if kind=='quincy' else (0,math.pi)) for x in (.25,.2,.15,.3,.1,0,.4,-.1) for y in (.34,.32,.36,.38,.40,.30) for branch in range(10 if kind=='quincy' else 1)]
    for lateral,stand,flip,follow,yaw,branch in candidates:
        angle=math.radians(yaw);m.body_quat[base]=[math.cos(angle/2),0,0,math.sin(angle/2)]
        m.body_pos[base]=h+[lateral,-stand,-h[2]+.002]
        orient=R@rz(flip);seed=d.qpos.copy();seed[qa]=[1.8,1.,0,-.6,0,0,0,meta['lift_target']]
        if branch:seed[qa[:-1]]=rng.uniform(m.jnt_range[[m.joint(n).id for n in ARM],0],m.jnt_range[[m.joint(n).id for n in ARM],1])
        path=[];worst=0;angles=0
        for t in np.linspace(0,1,21):
            q=t*meta['open_target'];p=anchor+rz(q)@(h-anchor) if kind=='quincy' else h+axis*q
            r=rz(q*follow)@orient if kind=='quincy' else orient
            solution,pe,re=fit(m,seed,SITE,names,p,r)
            if t==0 and (pe>.001 or re>.01):
                for _ in range(5):
                    trial=seed.copy();trial[qa[:-1]]=rng.uniform(m.jnt_range[[m.joint(n).id for n in ARM],0],m.jnt_range[[m.joint(n).id for n in ARM],1]);z,e,a=fit(m,trial,SITE,names,p,r)
                    if e+.1*a<pe+.1*re:solution,pe,re=z,e,a
                    if pe<.001 and re<.01:break
            worst=max(worst,pe);angles=max(angles,re)
            if pe>.001 or re>.01:break
            seed[qa]=solution;seed[mid]=solution[-1]/2;path.append(solution.tolist())
        if len(path)==21:
            seed[qa]=path[0]
            approach,pe,re=fit(m,seed,SITE,names,h+[0,-(.02 if kind=='s60' else .05),0],orient,iterations=300)
            if pe>.001 or re>.01:continue
            candidate=dict(kind=kind,lateral=lateral,stand=stand,flip=flip,follow=follow,yaw=angle,orientation=orient.tolist(),names=names,path=path,approach=approach.tolist(),open_target=meta['open_target'],grasp_depth_m=meta['grasp_depth_m'],max_error_m=worst,max_angle_error_rad=angles)
            print(json.dumps(candidate),flush=True);return candidate
        if best is None or len(path)>best[0]:best=(len(path),lateral,stand,flip,worst,angles,follow,yaw);print('partial',kind,best,flush=True)
    return dict(kind=kind,failure=best)

if __name__=='__main__':
    import sys
    from build_scene import ROOT
    for kind in sys.argv[1:] or ['st_load','quincy','s60']:
        result=search(kind);(ROOT/f'access_{kind}_plan.json').write_text(json.dumps(result,indent=2))
