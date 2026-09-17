"""Passive drawer / original left claw on an actuated Cartesian TEST FIXTURE.

Not Nori arm reach validation. Drawer has no motor, weld or pose updates.
Mass, friction and effort thresholds are explicit development assumptions.
Whole finger mesh hulls are conservative contact proxies, not detailed recesses.
"""
import copy,json,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from build_scene import ROOT,el,vec
from gripper_physics import SOURCE

def build(clear_width=.064,friction=2.,force_limit=12.,grip_torque=.5):
    src=ET.parse(SOURCE).getroot()
    root=ET.Element('mujoco',model='Passive drawer / left-claw fixture ONLY')
    el(root,'compiler',angle='radian');el(root,'option',timestep='.001',integrator='implicitfast',iterations='100',cone='elliptic',noslip_iterations='20')
    asset=el(root,'asset');world=el(root,'worldbody');eq=el(root,'equality');act=el(root,'actuator')
    el(el(root,'visual'),'global',offwidth='960',offheight='720')
    wrist=copy.deepcopy(src.find(".//body[@name='left_wrist_roll_link']"))
    wrist.set('name','actuated_drawer_fixture');wrist.set('pos','0 -.139 .1475')
    # Local -Z faces drawer (+world Y); local X is jaw closing direction (world Z).
    wrist.attrib.pop('quat',None);wrist.set('xyaxes','0 0 1 -1 0 0')
    wrist.remove(wrist.find('joint'))
    # Slide axes are in the wrist frame: local Z = -world Y (outward pull).
    el(wrist,'joint',name='fixture_pull',type='slide',axis='0 0 1',range='-.015 .21',damping='5')
    used=set()
    for body in wrist.iter('body'):
        for g in list(body.findall('geom')):
            if g.get('type')=='box':body.remove(g);continue
            if g.get('mesh'):
                used.add(g.get('mesh'))
                if g.get('mesh').startswith('gripper_'):
                    contact=copy.deepcopy(g);contact.set('name',g.get('mesh')+'_contact')
                    contact.set('contype','2');contact.set('conaffinity','1');contact.set('group','3')
                    contact.set('rgba','0 .8 .4 0');contact.set('friction','.6 .005 .0001')
                    contact.set('solref','.004 1');body.append(contact)
    for name in used:
        mesh=copy.deepcopy(src.find(f"asset/mesh[@name='{name}']"));mesh.set('file',str((SOURCE.parent/mesh.get('file')).resolve()));asset.append(mesh)
    world.append(wrist)
    eq.append(copy.deepcopy(src.find("equality/joint[@name='left_gripper_mimic']")))
    eq.find('joint').set('solref','.003 1')
    el(act,'position',name='fixture_motor',joint='fixture_pull',kp='800',kv='30',forcerange=vec([-force_limit,force_limit]))
    el(act,'position',name='left_claw_motor',joint='left_gripper_joint',kp='12',kv='.4',ctrlrange='0 .4',forcerange=vec([-grip_torque,grip_torque]))
    drawer=el(world,'body',name='passive_drawer',pos='0 0 0')
    el(drawer,'inertial',pos='0 .1 .10',mass='1.5',diaginertia='.02 .02 .02')
    el(drawer,'joint',name='CAD_drawer_slide',type='slide',axis='0 -1 0',range='0 .23',frictionloss=str(friction),damping='3')
    def box(parent,name,pos,size,color):el(parent,'geom',name=name,type='box',pos=vec(pos),size=vec(size),rgba=color,solref='.004 1',friction='.6 .005 .0001')
    # 86 mm input front panel from the CAD. The rest is an explicit test proxy.
    box(drawer,'front',[0,.0015,.15],[.043,.0015,.024],'.75 .77 .8 1')
    box(drawer,'tray',[0,.10,.123],[.040,.10,.002],'.7 .7 .7 1')
    outer=clear_width/2+.008
    for sign in (-1,1):box(drawer,'pull_leg_'+str(sign),[sign*(clear_width/2+.004),-.0165,.15],[.004,.0165,.005],'.25 .58 .62 1')
    box(drawer,'pull_bar',[0,-.030,.15],[outer,.003,.005],'.25 .58 .62 1')
    box(world,'support',[0,.1,.045],[.16,.24,.04],'.5 .6 .65 1')
    el(world,'light',pos='0 -1 1',dir='0 1 -1')
    return root

def run(clear_width=.064,friction=2.,force_limit=12.,grip_torque=.5,render=False):
    root=build(clear_width,friction,force_limit,grip_torque);m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    for name,value in [('left_gripper_joint',.3),('left_gripper_idler_joint',-.3)]:d.qpos[m.joint(name).qposadr[0]]=value
    mujoco.mj_forward(m,d)
    drawqa=m.joint('CAD_drawer_slide').qposadr[0];fixtureqa=m.joint('fixture_pull').qposadr[0]
    phase='CLOSE';events=[];phase_start=0.;dwell=0.;target=0.;max_force=0.;max_open=0.;peak_slip=0.;reference=None;blocked_dwell=0.
    jaw_names=['gripper_r_mirrored_contact','gripper_l_mirrored_contact'];jaws=[m.geom(n).id for n in jaw_names];bar=m.geom('pull_bar').id
    force_samples=[];terminal_t=None;stop_drawer=None;stop_drift=0.
    for i in range(12000):
        t=d.time
        if phase=='OPEN':target=min(.12,.035*(t-phase_start))
        elif phase=='CLOSE_DRAWER':target=max(0,.12-.035*(t-phase_start))
        d.ctrl[:]=[target,0 if phase not in ('NO_GRASP','RELEASE','COMPLETE') else .3]
        mujoco.mj_step(m,d)
        assert np.isfinite(d.qpos).all()
        forces=[0.,0.]
        for ci,c in enumerate(d.contact):
            for k,g in enumerate(jaws):
                if {int(c.geom1),int(c.geom2)}=={g,bar}:
                    f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);forces[k]+=max(0,float(f[0]))
        effort=abs(float(d.actuator_force[0]));max_force=max(max_force,effort);max_open=max(max_open,float(d.qpos[drawqa]))
        if phase=='CLOSE':
            dwell=dwell+m.opt.timestep if min(forces)>.05 else 0.
            if dwell>.15:
                phase='OPEN';phase_start=t;reference=float(d.qpos[fixtureqa]-d.qpos[drawqa]);events.append([t,phase])
            elif t>2:phase='NO_GRASP';events.append([t,phase])
        elif phase in ('OPEN','CLOSE_DRAWER'):
            peak_slip=max(peak_slip,abs(float(d.qpos[fixtureqa]-d.qpos[drawqa])-reference))
            blocked_dwell=blocked_dwell+m.opt.timestep if effort>force_limit*.98 and abs(d.qvel[m.joint('CAD_drawer_slide').dofadr[0]])<.002 else 0.
            if blocked_dwell>.15:phase='FORCE_STOP';target=float(d.qpos[fixtureqa]);events.append([t,phase])
            elif peak_slip>.004:phase='GRASP_LOST';target=float(d.qpos[fixtureqa]);events.append([t,phase])
            elif t-phase_start>4.:
                if phase=='OPEN' and d.qpos[drawqa]>.115:phase='CLOSE_DRAWER';phase_start=t;events.append([t,phase])
                elif phase=='CLOSE_DRAWER' and d.qpos[drawqa]<.004:phase='RELEASE';phase_start=t;events.append([t,phase])
                else:phase='TRAVEL_FAILED';target=float(d.qpos[fixtureqa]);events.append([t,phase])
        elif phase=='RELEASE' and t-phase_start>.5:phase='COMPLETE';events.append([t,phase])
        if i%100==0:force_samples.append(dict(t=t,phase=phase,drawer_m=float(d.qpos[drawqa]),fixture_m=float(d.qpos[fixtureqa]),jaw_N=forces,fixture_N=float(d.actuator_force[0])))
        if phase in ('FORCE_STOP','GRASP_LOST','TRAVEL_FAILED','NO_GRASP','COMPLETE'):
            if terminal_t is None:
                terminal_t=t;stop_drawer=float(d.qpos[drawqa]);target=float(d.qpos[fixtureqa])
            stop_drift=max(stop_drift,abs(float(d.qpos[drawqa])-stop_drawer))
            # Physics continues after the fault: do not hide inertia by freezing time.
            if t-terminal_t>.5:break
    if render:
        from PIL import Image,ImageDraw,ImageFont
        with mujoco.Renderer(m,height=720,width=960) as renderer:
            cam=mujoco.MjvCamera();cam.lookat[:]=[0,-.06,.13];cam.distance=.52;cam.azimuth=140;cam.elevation=-25
            renderer.update_scene(d,cam);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
            draw.rectangle([0,0,960,55],fill='#10283c');draw.text((12,12),'Passive drawer / left-claw TEST FIXTURE: '+phase,fill='white',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',22));im.save(ROOT/'drawer_contact_bench.png')
    return dict(passed=phase=='COMPLETE',phase=phase,clear_width_m=clear_width,friction_N=friction,grip_torque_limit_Nm=grip_torque,force_limit_N=force_limit,max_fixture_force_N=max_force,max_open_m=max_open,peak_relative_slip_m=peak_slip,final_drawer_m=float(d.qpos[drawqa]),post_stop_drawer_drift_m=stop_drift,post_stop_simulation_s=d.time-terminal_t if terminal_t is not None else 0,events=events,trace=force_samples,
        assumptions=['Cartesian test fixture, NOT Nori arm trajectory.','1.5 kg drawer, friction and force thresholds unmeasured.','Left finger convex hull contacts conservative; coupled-finger self-contact excluded; detailed claw contact and clearance unvalidated.','No drawer actuator, weld, mocap or post-initialization pose assignments.'])

if __name__=='__main__':
    cases=[run(clear_width=.044),run(render=True),run(friction=30,force_limit=1)]
    report=dict(transfer_ready=False,cases=cases)
    (ROOT/'drawer_contact_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps([{k:v for k,v in result.items() if k!='trace'} for result in cases],indent=2))
