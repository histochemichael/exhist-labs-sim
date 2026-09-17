"""Six-dimensional IK on scratch state; live motion must use actuator controls."""
import numpy as np
import mujoco

def rotation_error(target,current):
    matrix=target@current.T
    q=np.zeros(4);mujoco.mju_mat2Quat(q,matrix.ravel())
    if q[0]<0:q=-q
    norm=np.linalg.norm(q[1:])
    return 2*q[1:] if norm<1e-10 else q[1:]/norm*(2*np.arctan2(norm,q[0]))

def solve_pose(model,initial,site,joints,position,rotation,iterations=500):
    # The supplied live state is never modified by IK.
    scratch=mujoco.MjData(model);scratch.qpos[:]=initial
    ids=np.array([model.joint(n).id for n in joints]);qa=model.jnt_qposadr[ids];va=model.jnt_dofadr[ids]
    sid=model.site(site).id;jp=np.zeros((3,model.nv));jr=np.zeros_like(jp)
    best=None;best_score=float('inf')
    for _ in range(iterations):
        mujoco.mj_forward(model,scratch)
        ep=np.asarray(position)-scratch.site_xpos[sid]
        er=rotation_error(rotation,scratch.site_xmat[sid].reshape(3,3))
        score=np.linalg.norm(ep)+.10*np.linalg.norm(er)
        if score<best_score:best=(scratch.qpos[qa].copy(),float(np.linalg.norm(ep)),float(np.linalg.norm(er)));best_score=score
        if np.linalg.norm(ep)<.0002 and np.linalg.norm(er)<.002:break
        mujoco.mj_jacSite(model,scratch,jp,jr,sid)
        jac=np.vstack([jp[:,va],.10*jr[:,va]]);error=np.r_[ep,.10*er]
        delta=jac.T@np.linalg.solve(jac@jac.T+np.eye(6)*1e-5,error)
        scratch.qpos[qa]+=np.clip(delta,-.08,.08)
        for ji,qi in zip(ids,qa):
            if model.jnt_limited[ji]:scratch.qpos[qi]=np.clip(scratch.qpos[qi],*model.jnt_range[ji])
    return best

def multistart(model,initial,site,joints,position,rotation,attempts=20):
    rng=np.random.default_rng(72);qa=np.array([model.joint(n).qposadr[0] for n in joints]);best=None
    for attempt in range(attempts):
        seed=initial.copy()
        if attempt:
            for name,q in zip(joints,qa):seed[q]=rng.uniform(*model.jnt_range[model.joint(name).id])
        result=solve_pose(model,seed,site,joints,position,rotation)
        if best is None or result[1]+.1*result[2]<best[1]+.1*best[2]:best=result
        if best[1]<.0003 and best[2]<.003:break
    return best
