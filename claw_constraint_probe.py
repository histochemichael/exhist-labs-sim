import equipment_access as e
a=e.Access('s60')
print('integrator',a.m.opt.integrator,flush=True)
gq=a.m.joint('left_gripper_joint').qposadr[0]
iq=a.m.joint('left_gripper_idler_joint').qposadr[0]
print('damping armature',[(j,a.m.dof_damping[a.m.joint(j).dofadr[0]],a.m.dof_armature[a.m.joint(j).dofadr[0]]) for j in ['left_gripper_joint','left_gripper_idler_joint']],flush=True)
while a.d.time<4.4:
    a.step()
    if a.steps%20==0 and a.d.time>4.3:
        print(a.d.time,a.phase,a.d.qpos[gq],a.d.qpos[iq],a.d.efc_pos[1],a.d.efc_aref[1],a.d.efc_force[1],a.d.actuator_force[a.grip],flush=True)
