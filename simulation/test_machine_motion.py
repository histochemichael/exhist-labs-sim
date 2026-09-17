"""Coverage, limits, parent motion, pause and fault checks for imported CAD joints."""
import json
import numpy as np
import mujoco
from operate_lab import Scene,ROOT,camera
from PIL import Image,ImageDraw,ImageFont

s=Scene(preview=True);man=s.machines.manifest
assert man["source_joint_count"]==84
assert len(man["rigid_joints"])==10
assert man["movable_joints_per_workstation"]==74
assert man["dofs_per_workstation"]==88
assert len(s.machines.dofs)==176
q0=s.d.qpos.copy();tested=0;locked=0
for row,dof in s.machines.dofs:
    joint=s.m.joint(dof["name"]);adr=joint.qposadr[0]
    bid=s.m.body(row["body"]).id
    s.d.qpos[:]=q0;mujoco.mj_forward(s.m,s.d)
    before=np.r_[s.d.xpos[bid],s.d.xmat[bid]].copy()
    if dof["locked"]:
        locked+=1;continue
    value=s.machines.target(dof,.5)
    s.d.qpos[adr]=value;mujoco.mj_forward(s.m,s.d)
    after=np.r_[s.d.xpos[bid],s.d.xmat[bid]]
    assert np.isfinite(after).all()
    assert np.linalg.norm(after-before)>1e-7,(row["cad_name"],dof)
    tested+=1
s.d.qpos[:]=q0
motion={d["name"]:0. for _,d in s.machines.dofs};seen=set();rendered=False
for i in range(3500):
    previous=s.d.qpos.copy();s.lab.tick(.1);s.sync()
    for row,dof in s.machines.dofs:
        joint=s.m.joint(dof["name"]);q=s.d.qpos[joint.qposadr[0]]
        motion[dof["name"]]+=abs(q-previous[joint.qposadr[0]])
        lo,hi=dof["limits_delta"]
        if lo is not None:assert q>=lo-1e-8
        if hi is not None:assert q<=hi+1e-8
    seen.update((station,device) for (station,device),state in s.machines.states.items() if state=="RUNNING")
    if i==450:
        frozen=s.d.qpos.copy();s.lab.paused=True;s.lab.tick(.5);s.sync()
        assert np.allclose(frozen,s.d.qpos);s.lab.paused=False
        s.lab.fault("routine_a");frozen=s.d.qpos.copy()
        s.lab.tick(.1);s.sync()
        for row,dof in s.machines.dofs:
            if row["station"]=="routine_a":assert s.d.qpos[s.m.joint(dof["name"]).qposadr[0]]==frozen[s.m.joint(dof["name"]).qposadr[0]]
        s.lab.recover()
    if not rendered and s.machines.states.get(("routine_a","CV5030"))=="RUNNING":
        s.machines.toggle_xray()
        cam=camera(s.lab.x["routine_a"],True);cam.distance=2.2;cam.elevation=-22
        with mujoco.Renderer(s.m,height=1080,width=1920) as renderer:
            renderer.update_scene(s.d,cam);img=Image.fromarray(renderer.render())
            draw=ImageDraw.Draw(img);draw.rectangle([0,0,1920,140],fill="#10283c")
            draw.multiline_text((24,18),"ExHist | Articulated Leica workstation\n"+s.machines.status()+"\nX-ray inspection | illustrative motion; not a validated instrument cycle",
                font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",23),fill="white",spacing=4)
            img.save(ROOT/"machine_motion_xray.png")
        s.machines.toggle_xray();rendered=True
    if s.lab.completed==36:break
assert len(seen)==4,seen
assert s.lab.completed==36
report=dict(source_joints_per_workstation=84,rigid_per_workstation=10,moving_per_workstation=74,
    imported_dofs_per_workstation=88,total_dofs=176,independent_dof_motion_tests=tested,locked_dofs=locked,
    workflow_active_dofs=int(sum(v>1e-6 for v in motion.values())),workflow_complete_slides=36,
    all_four_machine_states_observed=True,pause_and_fault_freeze=True,joint_limits_passed=True,
    limitation="No internal machine collision, grip/contact, chemistry or real-device status validation.")
(ROOT/"machine_motion_validation.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
