"""HISTORICAL flat/convex jaw experiment. Use groove_bench.py for current fingers.

The optional pads and clearance diagnosis are superseded by original groove
contacts in gripper_physics.py and the user's successful physical tests.

The upstream Cartesian lift is a TEST FIXTURE, not a validated Nori arm.
No payload weld, mocap, position reset after initialization, or adhesion is used.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import copy,json,math
import numpy as np
import mujoco
from build_scene import ROOT,el,box,vec
from handling_contracts import pose_check,grasp_check

SOURCE=ROOT/'source/nori/Nori with parallel histo gripper.xml'

def build(kind="slide",offset=0.,friction=.6,padded=False):
    original=ET.parse(SOURCE).getroot()
    root=ET.Element("mujoco",model="ExHist contact regression / test fixture")
    el(root,"compiler",angle="radian")
    el(root,"option",timestep=".001",integrator="implicitfast",cone="elliptic",iterations="100",gravity="0 0 -9.81")
    visual=el(root,"visual");el(visual,"global",offwidth="1280",offheight="960")
    el(visual,"headlight",ambient=".5 .5 .5",diffuse=".6 .6 .6")
    asset=el(root,"asset")
    for mesh in original.find("asset").findall("mesh"):
        if mesh.get("name","").startswith("lehisto_part"):
            new=copy.deepcopy(mesh);new.set("file",str((SOURCE.parent/mesh.get("file")).resolve()));asset.append(new)
    world=el(root,"worldbody")
    el(world,"geom",name="floor",type="plane",size="1 1 .01",rgba=".65 .73 .75 1")
    box(world,"support",[0,0,.0125],[.10,.10,.0125],[.78,.82,.84,1])
    fixture=el(world,"body",name="actuated_test_fixture",pos="0 .0933 .1982",quat=".707106781  .707106781 0 0")
    el(fixture,"joint",name="fixture_lift",type="slide",axis="0 1 0",range="0 .10",damping="20")
    el(fixture,"inertial",mass=".6",pos="0 0 0",diaginertia=".003 .003 .003")
    mount=copy.deepcopy(original.find(".//body[@name='lehisto_mount']"))
    mount.attrib.pop("xyaxes",None);fixture.append(mount)
    for geom in mount.iter("geom"):
        if geom.get("name","").endswith("_contact"):
            geom.set("friction",f"{friction} .005 .0001");geom.set("condim","4")
            geom.set("solref",".004 1");geom.set("solimp",".95 .99 .001")
    if padded:
        # CONCEPT ONLY: 3 mm inner-face pads, 18 mm along finger, 5 mm thick.
        # They extend the CAD tip at x=+/-42 mm inward to +/-39 mm.
        # Attachment, compliance and material properties require engineering.
        for jaw,sign in [("a",1),("b",-1)]:
            parent=mount.find(f".//joint[@name='lehisto_jaw_{jaw}_slide']/..")
            el(parent,"geom",name=f"pad_{jaw}",type="box",pos=vec([sign*.0405,-.1111885,.0933]),
               size=".0015 .009 .0025",mass=".002",rgba=".12 .65 .56 1",friction=f"{friction} .005 .0001",
               contype="2",conaffinity="1",condim="4",solref=".004 1",solimp=".95 .99 .001")
    # Payload mass is an explicit test assumption; glass uses nominal 2500 kg/m3.
    if kind=="slide":size=[.0125,.0005,.0375];mass=.0046875;z=.0625
    else:size=[.0045,.001,.00375];mass=.003;z=.02875
    payload=el(world,"body",name="payload",pos=vec([offset,0,z]))
    el(payload,"freejoint",name="payload_free")
    el(payload,"geom",name="payload_contact",type="box",size=vec(size),mass=str(mass),
       rgba=".65 .9 .96 .9" if kind=="slide" else ".95 .7 .2 1",friction=f"{friction} .005 .0001",
       condim="4",solref=".004 1",solimp=".95 .99 .001")
    eq=el(root,"equality")
    for e in original.find("equality"):
        if e.get("name","").startswith("lehisto_"):eq.append(copy.deepcopy(e))
    actuator=el(root,"actuator")
    el(actuator,"position",name="fixture_servo",joint="fixture_lift",kp="3000",kv="80",ctrlrange="0 .10",forcerange="-80 80")
    el(actuator,"position",name="jaw_servo",joint="lehisto_jaw_a_slide",kp="1500",kv="10",ctrlrange="-.0355 0",forcerange="-.8 .8")
    return root

def run_case(name,offset=0.,friction=.6,release=True,padded=False,render=False):
    root=build(offset=offset,friction=friction,padded=padded)
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding="unicode"));d=mujoco.MjData(m)
    payload=m.body("payload").id;pg=m.geom("payload_contact").id
    target=np.array([0,0,.0625]);pose_rejection=[]
    jaws={m.geom("lehisto_part_14_contact").id:"a",m.geom("lehisto_part_15_contact").id:"b"}
    if padded:jaws.update({m.geom("pad_a").id:"a",m.geom("pad_b").id:"b"})
    start=None;peak=0.;forces=[];pose=[];records=[];events=[]
    phase="SETTLE";phase_start=0.;bilateral_ticks=0;hold_ticks=0;good_hold_ticks=0
    qualified=False;lift_start=None;release_z=None;hold_local=None;max_slip=0.
    lost_ticks=0;limits_ok=True;efforts_ok=True;unexpected=False;grasp_local=None
    lift=0.;close=0.;max_fixture=0.;lift_frame=None
    for i in range(10000):
        t=d.time
        if phase=="SETTLE" and t>=.5:
            admission=pose_check(d.xpos[payload]-target,d.xmat[payload].reshape(3,3),[.002]*3,math.radians(2),True)
            # The fixture's target frame is analytically defined here only;
            # this does not calibrate any lab robot TCP or rack pose.
            pose_rejection=list(admission.reasons)
            phase="CLOSE" if admission.allowed else "POSE_REJECTED"
            phase_start=t;events.append(dict(t=t,event=phase,reasons=pose_rejection))
        close=-.0355 if phase in ("CLOSE","LIFT","HOLD","KEEP_HOLD","GRASP_LOST") else 0.
        if phase=="LIFT":lift=.06*min(1,(t-phase_start))
        elif phase in ("HOLD","RELEASE","KEEP_HOLD"):lift=.06
        d.ctrl[:]=[lift,close]
        mujoco.mj_step(m,d)
        assert np.isfinite(d.qpos).all()
        if start is None and t>=.4:start=d.xpos[payload].copy()
        contact={}
        for cidx,c in enumerate(d.contact):
            other=c.geom2 if c.geom1==pg else c.geom1 if c.geom2==pg else -1
            if other in jaws:
                wrench=np.zeros(6);mujoco.mj_contactForce(m,d,cidx,wrench)
                contact[jaws[other]]=contact.get(jaws[other],0)+max(0,float(wrench[0]))
        both=contact.get("a",0)>.005 and contact.get("b",0)>.005
        # Soft contact/limit solvers admit small residual errors; bounds below
        # are explicit fixture tolerances, not authorization to exceed hardware.
        for ji in range(m.njnt):
            if m.jnt_limited[ji]:
                q=float(d.qpos[m.jnt_qposadr[ji]]);low,high=m.jnt_range[ji]
                tolerance=.0005 if m.jnt_type[ji]==mujoco.mjtJoint.mjJNT_SLIDE else .002
                limits_ok &= bool(low-tolerance<=q<=high+tolerance)
        efforts_ok &= bool(np.all(d.actuator_force>=m.actuator_forcerange[:,0]-1e-6) and np.all(d.actuator_force<=m.actuator_forcerange[:,1]+1e-6))
        if phase in ("LIFT","HOLD"):
            lost_ticks=0 if both else lost_ticks+1
            slip_from_grasp=float(np.linalg.norm((d.xpos[payload]-d.xpos[m.body("actuated_test_fixture").id])-grasp_local))
            if lost_ticks>=20 or slip_from_grasp>.002:
                phase="GRASP_LOST";phase_start=t
                events.append(dict(t=t,event=phase,reasons=["slip_detected_stop_fixture" if slip_from_grasp>.002 else "lost_bilateral_contact_stop_fixture"]))
        if phase=="CLOSE":
            bilateral_ticks=bilateral_ticks+1 if both else 0
            if bilateral_ticks>=120:
                phase="LIFT";phase_start=t;lift_start=t;events.append(dict(t=t,event=phase))
                grasp_local=d.xpos[payload]-d.xpos[m.body("actuated_test_fixture").id]
            elif t-phase_start>=4:
                phase="NO_GRASP";phase_start=t;events.append(dict(t=t,event=phase))
        elif phase=="LIFT" and t-phase_start>=1.2:
            phase="HOLD";phase_start=t;events.append(dict(t=t,event=phase))
            hold_local=d.xpos[payload]-d.xpos[m.body("actuated_test_fixture").id]
        elif phase=="HOLD":
            hold_ticks+=1;good_hold_ticks+=int(both)
            local=d.xpos[payload]-d.xpos[m.body("actuated_test_fixture").id]
            max_slip=max(max_slip,float(np.linalg.norm(local-hold_local)))
            forces.extend(contact.values());pose.append(d.xpos[payload].copy())
            unexpected |= any((c.geom1==pg or c.geom2==pg) and (c.geom2 if c.geom1==pg else c.geom1) not in jaws for c in d.contact)
            if render and lift_frame is None and t-phase_start>.5:
                with mujoco.Renderer(m,height=720,width=960) as renderer:
                    cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.14];cam.distance=.40;cam.azimuth=130;cam.elevation=-15
                    renderer.update_scene(d,cam);lift_frame=renderer.render().copy()
            if t-phase_start>=2:
                acceptance=grasp_check(normal_forces=[contact.get("a",0),contact.get("b",0)],normal_force_range=[.005,.8],
                    bilateral_dwell_s=good_hold_ticks*.001,minimum_dwell_s=1.98,relative_speed_mps=float(np.linalg.norm(d.qvel[m.joint("payload_free").dofadr[0]:][:3])),
                    max_speed_mps=.005,relative_slip_m=max_slip,max_slip_m=.002,lifted_m=peak,minimum_lift_m=.045,
                    joint_limits_ok=bool(limits_ok),effort_limits_ok=bool(efforts_ok),unexpected_contact=bool(unexpected))
                # Limits/contact exclusions here apply to the TEST FIXTURE only.
                qualified=acceptance.allowed and good_hold_ticks/hold_ticks>.99
                phase="RELEASE" if release else "KEEP_HOLD";phase_start=t
                release_z=float(d.xpos[payload,2]);events.append(dict(t=t,event=phase,qualified=qualified,reasons=acceptance.reasons))
        if start is not None:peak=max(peak,float(d.xpos[payload,2]-start[2]))
        max_fixture=max(max_fixture,float(d.qpos[m.joint("fixture_lift").qposadr[0]]))
        if i%100==0:records.append(dict(t=round(t,3),phase=phase,xyz=d.xpos[payload].tolist(),contacts=contact))
    if lift_frame is not None:
        from PIL import Image,ImageDraw,ImageFont
        im=Image.fromarray(lift_frame);draw=ImageDraw.Draw(im)
        draw.rectangle([0,0,960,68],fill="#10283c")
        draw.text((15,10),f"{name}: contact-only test fixture, NOT the Nori arm",font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",21),fill="white")
        draw.text((15,38),"Green tip pads are an unbuilt concept; nominal friction/mass.",font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",18),fill="white")
        im.save(ROOT/f"contact_{name}.png")
    result=dict(case=name,friction=friction,offset_m=offset,peak_lift_m=peak,
        settled_final_z_m=float(d.xpos[payload,2]),start_z_m=float(start[2]),
        bilateral_hold_fraction=good_hold_ticks/max(hold_ticks,1),hold_samples=hold_ticks,
        relative_hold_slip_m=max_slip,concept_pads=padded,
        hold_position_std_m=np.std(pose,axis=0).tolist() if pose else None,
        peak_sampled_normal_force_N=max(forces,default=0),released=release,
        passed_grasp=qualified,maximum_fixture_lift_m=max_fixture,
        release_drop_m=release_z-float(d.xpos[payload,2]) if release_z is not None and release else None,
        events=events,pose_rejection=pose_rejection,payload_has_free_joint=True,payload_has_actuator=False,payload_weld=False)
    result.update(joint_limits_within_fixture_tolerance=bool(limits_ok),effort_limits_ok=bool(efforts_ok),unexpected_hold_contact=bool(unexpected))
    return result,records

if __name__=="__main__":
    results=[]
    for name,offset,friction,release,padded in [("stock",0,.6,True,False),("padded",0,.6,True,True),("missed",.065,.6,True,True),("slippery",0,0,False,True)]:
        result,trace=run_case(name,offset,friction,release,padded,render=name=="padded");results.append(result)
        (ROOT/f"contact_{name}_trace.json").write_text(json.dumps(trace,indent=2))
    report=dict(cases=results,scope="Actual CAD jaw contacts / nominal glass slide / actuated test fixture, NOT a Nori arm validation",
        limitations=["Friction, glass density and servo limits are test assumptions.",
                     "No material fracture model; impact can be detected but breakage cannot be predicted.",
                     "Mesh collisions use MuJoCo convex hulls, not exact concave CAD surfaces.",
                     "Release is an intentional DROP TEST over the support, not a successful placement.",
                     "A 2-second nominal hold is not extraction-from-rack or full-trajectory validation.",
                     "Rack-handle retention and individual-slide extraction from loaded racks are separate unvalidated tasks."])
    (ROOT/"contact_bench_validation.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    ET.indent(scene:=build(padded=True));ET.ElementTree(scene).write(ROOT/"contact_bench.xml",encoding="utf-8",xml_declaration=True)
