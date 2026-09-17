"""Read-only runtime diagnostics for acquisition, force and actual jaw geometry."""
import sys,json
import numpy as np
import mujoco
import equipment_access as e

kind=sys.argv[1] if len(sys.argv)>1 else 'quincy'
a=e.Access(kind)
for _ in range(6500):a.step()
print(dict(phase=a.phase,filtered_N=a.filtered_forces.tolist(),alignment=(a.d.site_xpos[a.sid]-a.d.site_xpos[a.hsid]).tolist(),door_q=float(a.d.qpos[a.jq]),door_velocity=float(a.d.qvel[a.jv]),grip=float(a.d.qpos[a.m.joint('left_gripper_joint').qposadr[0]]),idler=float(a.d.qpos[a.m.joint('left_gripper_idler_joint').qposadr[0]]),grip_effort=float(a.d.actuator_force[a.grip]),arm_effort=a.d.actuator_force[a.ai].tolist()),flush=True)
for i,c in enumerate(a.d.contact):
    if a.handle not in (c.geom1,c.geom2):continue
    f=np.zeros(6);mujoco.mj_contactForce(a.m,a.d,i,f)
    if f[0]>.1:print('contact',a.m.geom(c.geom1).name,a.m.geom(c.geom2).name,'gap',c.dist,'force',f[:3].tolist(),'normal',c.frame[:3].tolist(),flush=True)
for angle in (0.,.05,.1,.2,.3):
    d=mujoco.MjData(a.m);d.qpos[:]=a.d.qpos
    d.qpos[a.m.joint('left_gripper_joint').qposadr[0]]=angle;d.qpos[a.m.joint('left_gripper_idler_joint').qposadr[0]]=-angle;mujoco.mj_kinematics(a.m,d)
    wrist=a.m.body('left_wrist_roll_link').id;rotation=d.xmat[wrist].reshape(3,3);bounds=[]
    for name in ('left_gripper_link','left_gripper_idler_link'):
        bid=a.m.body(name).id;gid=next(i for i in range(a.m.ngeom) if a.m.geom_bodyid[i]==bid and a.m.geom_group[i]==1);mid=a.m.geom_dataid[gid];start=a.m.mesh_vertadr[mid];count=a.m.mesh_vertnum[mid]
        v=a.m.mesh_vert[start:start+count]@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
        v=(v-d.xpos[wrist])@rotation;sample=v[(abs(v[:,2]+.114)<.003)&(abs(v[:,1]-.00652)<.010)]
        bounds.append([name,sample.min(0).tolist() if len(sample) else None,sample.max(0).tolist() if len(sample) else None])
    print('angle',angle,bounds,flush=True)
