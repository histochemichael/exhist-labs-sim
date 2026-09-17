import json
import numpy as np
from operate_lab import Scene,ROOT

s=Scene(preview=True);qmin=s.d.qpos.copy();qmax=qmin.copy();contacts={};limits=[]
assert s.m.joint("lehisto_jaw_a_slide").id>=0
assert s.m.body("lehisto_mount").id>=0
assert all(s.m.equality(n).id>=0 for n in ["lift_mimic","left_gripper_mimic","lehisto_jaw_sync","lehisto_rack_pinion"])
for _ in range(3400):
    s.lab.tick(.1);s.sync();qmin=np.minimum(qmin,s.d.qpos);qmax=np.maximum(qmax,s.d.qpos)
    qa=s.d.qpos[s.m.joint("lehisto_jaw_a_slide").qposadr[0]]
    qb=s.d.qpos[s.m.joint("lehisto_jaw_b_slide").qposadr[0]]
    qp=s.d.qpos[s.m.joint("lehisto_pinion_joint").qposadr[0]]
    assert abs(qa-qb)<1e-9 and abs(qp-qa/.0072)<1e-9
    for i in range(s.m.njnt):
        if s.m.jnt_limited[i]:
            q=s.d.qpos[s.m.jnt_qposadr[i]];lo,hi=s.m.jnt_range[i]
            if not lo-1e-6<=q<=hi+1e-6:limits.append(s.m.joint(i).name)
    for c in s.d.contact:
        if c.dist>=-.001:continue
        a=s.m.geom(c.geom1).name or str(c.geom1);b=s.m.geom(c.geom2).name or str(c.geom2)
        if any(p in a+b for p in ["_top","_leg_","_envelope"]):
            pair=" / ".join(sorted([a,b]));contacts[pair]=min(contacts.get(pair,0),float(c.dist))
    if s.lab.completed==36:break
travel={s.m.joint(i).name:float(qmax[s.m.jnt_qposadr[i]]-qmin[s.m.jnt_qposadr[i]]) for i in range(s.m.njnt)}
report={"completed_slides":s.lab.completed,"joint_limit_violations":sorted(set(limits)),
        "environment_penetrations_m":contacts,"joint_travel":travel,"ik":s.motion.report,
        "scope":"Sampled existing coarse Nori contacts only; LeHisto visual meshes have no contact collision coverage."}
(ROOT/"robot_motion_validation.json").write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!="joint_travel"},indent=2))
assert not limits
assert not contacts
assert s.motion.report["robots"]["nori"]["reach_valid"]
assert all(s.motion.report["robots"][n]["all_targets_reached"] for n in s.motion.plans)
assert all(travel[n+"_j2"]>.01 for n in s.motion.plans)
assert travel["right_elbow_pitch_joint"]>.01
assert travel["lehisto_jaw_a_slide"]>.01
