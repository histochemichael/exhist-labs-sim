"""Wheel-driven Nori development bench, not a validated hardware controller.

No base pose commands after initialization. Original arm effort caps are retained.
The lift is explicitly BRAKED for this isolated mobility test; brake capability,
10 kg chassis inertia and rolling-ball caster proxies are assumptions.
"""
import copy,json,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from build_scene import ROOT,el,vec
from gripper_physics import SOURCE,apply_nori

def build():
    root=ET.parse(SOURCE).getroot();root.set('model','Nori wheel dynamics / lift-braked development bench')
    for mesh in root.find('asset').findall('mesh'):mesh.set('file',str((SOURCE.parent/mesh.get('file')).resolve()))
    el(root,'option',timestep='.002',integrator='implicitfast',cone='elliptic',iterations='80')
    world=root.find('worldbody');base=ET.Element('body',name='mobile_chassis',pos='0 0 .002')
    el(base,'freejoint',name='mobile_free')
    el(base,'inertial',pos='-.13 0 .12',mass='10',diaginertia='.2 .2 .15')
    for child in list(world):
        if child.tag=='light' or child.get('name')=='checkerboard_floor':continue
        world.remove(child);base.append(child)
    # Original rear sphere shapes retained, but allowed to roll passively.
    for side,y in [('left',.075),('right',-.075)]:
        body=el(base,'body',name=side+'_ball_caster',pos=vec([-.273275,y,.0254]))
        el(body,'joint',name=side+'_caster_rotation',type='ball',damping='.0005')
        el(body,'inertial',pos='0 0 0',mass='.05',diaginertia='.0000129 .0000129 .0000129')
        for g in list(base.findall('geom')):
            p=np.fromstring(g.get('pos','0 0 0'),sep=' ')
            if np.allclose(p,[-.273275,y,.0254],atol=1e-6):
                base.remove(g);g.set('pos','0 0 0');g.set('friction','1 .005 .0001');body.append(g)
    world.append(base);apply_nori(root)
    # This explicit fixture brake isolates wheel testing from unverified 1 N lift import.
    el(root.find('equality'),'joint',name='TEST_FIXTURE_lift_brake',joint1='lift_extension_joint',polycoef='.25 0 0 0 0')
    actuator=root.find('actuator')
    for side in ['left','right']:
        el(actuator,'velocity',name=side+'_wheel_motor',joint=side+'_wheel_joint',kv='2',ctrlrange='-5 5',forcerange='-1.5 1.5')
    for joint in base.iter('joint'):
        name=joint.get('name','')
        if name.endswith('_joint') and any(word in name for word in ['shoulder','bicep','elbow','forearm','wrist']):
            el(actuator,'position',name='hold_'+name,joint=name,kp='120',kv='8',forcerange='-4 4')
    el(actuator,'position',name='hold_left_gripper',joint='left_gripper_joint',kp='30',kv='2',forcerange='-4 4')
    root.find('visual/global').set('offwidth','1280');root.find('visual/global').set('offheight','900')
    return root

def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi

class Drive:
    def __init__(self,model,data):
        self.m=model;self.d=data;self.home=data.qpos.copy();self.radius=.0762;self.track=.30
        self.holds=[]
        for ai in range(model.nu):
            name=model.actuator(ai).name
            if name.startswith('hold_'):
                ji=int(model.actuator_trnid[ai,0]);self.holds.append((ai,int(model.jnt_qposadr[ji]),int(model.jnt_dofadr[ji]),30 if name=='hold_left_gripper' else 120))
        self.wheels=[model.actuator(s+'_wheel_motor').id for s in ['left','right']]
        self.targets=[(.45,0,math.pi/2),(.45,.35,math.pi/2)];self.index=0;self.phase='SETTLE';self.events=[];self.fault=None
    def pose(self):
        p=self.d.xpos[self.m.body('mobile_chassis').id];r=self.d.xmat[self.m.body('mobile_chassis').id].reshape(3,3)
        return p,math.atan2(r[1,0],r[0,0]),math.acos(float(np.clip(r[2,2],-1,1)))
    def update(self):
        for ai,qa,va,kp in self.holds:self.d.ctrl[ai]=self.home[qa]+self.d.qfrc_bias[va]/kp
        p,yaw,tilt=self.pose();v=omega=0.
        if tilt>math.radians(12):self.fault='TILT_FAULT'
        if not np.isfinite(self.d.qpos).all() or not np.isfinite(self.d.qvel).all():self.fault='NONFINITE_FAULT'
        if self.fault:self.phase=self.fault
        elif self.d.time>=2 and self.index<len(self.targets):
            x,y,heading=self.targets[self.index];dx=x-p[0];dy=y-p[1];distance=math.hypot(dx,dy)
            error=wrap(math.atan2(dy,dx)-yaw)
            if distance>.012:
                self.phase='TURN_TO_PATH' if abs(error)>.12 else 'DRIVE'
                omega=float(np.clip(2*error,-.65,.65))
                if self.phase=='DRIVE':v=min(.12,1.2*distance)*max(0,math.cos(error))
            else:
                error=wrap(heading-yaw);omega=float(np.clip(2*error,-.65,.65));self.phase='FINAL_HEADING'
                if abs(error)<.025:
                    self.events.append(dict(t=self.d.time,target=self.index,position=p.tolist(),yaw=yaw))
                    self.index+=1;self.phase='COMPLETE' if self.index==len(self.targets) else 'TURN_TO_PATH'
        self.d.ctrl[self.wheels]=[(v-omega*self.track/2)/self.radius,(v+omega*self.track/2)/self.radius]

def initialize(m):
    d=mujoco.MjData(m)
    for name,value in [('lift_extension_joint',.25),('lift_middle_joint',.125)]:d.qpos[m.joint(name).qposadr[0]]=value
    mujoco.mj_forward(m,d)
    return d,Drive(m,d)

def run(render=False,motors_enabled=True,duration_s=40,save=True):
    root=build();m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    d,drive=initialize(m);trace=[];max_tilt=0.;max_lateral=0.;efforts=np.zeros(m.nu);completed_at=None
    if not motors_enabled:m.actuator_gainprm[drive.wheels,:]=0;m.actuator_biasprm[drive.wheels,:]=0
    for i in range(round(duration_s/m.opt.timestep)):
        drive.update();mujoco.mj_step(m,d);p,yaw,tilt=drive.pose()
        assert np.isfinite(d.qpos).all()
        max_tilt=max(max_tilt,tilt);efforts=np.maximum(efforts,np.abs(d.actuator_force))
        if d.time>2:max_lateral=max(max_lateral,abs(-math.sin(yaw)*d.qvel[0]+math.cos(yaw)*d.qvel[1]))
        if i%50==0:trace.append(dict(t=d.time,xyz=p.tolist(),yaw=yaw,phase=drive.phase,wheel_q=[float(d.qpos[m.joint(s+'_wheel_joint').qposadr[0]]) for s in ['left','right']]))
        if drive.phase=='COMPLETE':
            if completed_at is None:completed_at=d.time
            if d.time-completed_at<1:continue
            if render:
                from PIL import Image,ImageDraw,ImageFont
                with mujoco.Renderer(m,height=900,width=1280) as renderer:
                    cam=mujoco.MjvCamera();cam.lookat[:]=[.3,.15,.65];cam.distance=2.6;cam.azimuth=140;cam.elevation=-20
                    renderer.update_scene(d,cam);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
                    draw.rectangle([0,0,1280,64],fill='#10283c');draw.text((18,16),'Wheel-driven Nori test | lift braked | no base pose commands',fill='white',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',25))
                    im.save(ROOT/'wheel_driven_nori.png')
            break
    p,yaw,tilt=drive.pose()
    result=dict(passed=drive.phase=='COMPLETE' and max_tilt<math.radians(12),phase=drive.phase,seconds=d.time,
        pose=p.tolist(),yaw_rad=yaw,max_tilt_deg=math.degrees(max_tilt),max_lateral_velocity_mps=max_lateral,
        actuator_peak_effort={m.actuator(ai).name:dict(value=float(efforts[ai]),unit='N' if m.jnt_type[m.actuator_trnid[ai,0]]==mujoco.mjtJoint.mjJNT_SLIDE else 'Nm') for ai in range(m.nu)},
        motors_enabled=motors_enabled,final_linear_speed_mps=float(np.linalg.norm(d.qvel[:3])),final_yaw_speed_radps=float(d.qvel[5]),
        events=drive.events,base_pose_commands_after_initialization=0,
        limitations=['Isolated mobility test, not lab navigation or a carrier transfer.',
            'Lift held by explicit test brake; brake capability unverified.',
            '10 kg chassis inertia, ball-caster dynamics and 1.5 Nm wheel limits are assumptions.',
            'Arm servos retain imported 4 Nm caps; collision/actuator calibration is incomplete.'])
    result['passed']=result['passed'] and result['final_linear_speed_mps']<.005 and abs(result['final_yaw_speed_radps'])<.01
    if save:
        (ROOT/'mobile_base_validation.json').write_text(json.dumps(result,indent=2));(ROOT/'mobile_base_trace.json').write_text(json.dumps(trace,indent=2))
        ET.indent(root);ET.ElementTree(root).write(ROOT/'mobile_base_bench.xml',encoding='utf-8',xml_declaration=True)
    return result

def view():
    import time,mujoco.viewer
    m=mujoco.MjModel.from_xml_string(ET.tostring(build(),encoding='unicode'));d,drive=initialize(m)
    with mujoco.viewer.launch_passive(m,d) as viewer:
        viewer.cam.lookat[:]=[.3,.15,.65];viewer.cam.distance=2.6;viewer.cam.azimuth=140;viewer.cam.elevation=-20
        print('WHEEL-DRIVEN DEVELOPMENT TEST ONLY: lift braked, assumed wheel/caster/chassis dynamics. No lab transfers.')
        while viewer.is_running():
            start=time.monotonic();drive.update();mujoco.mj_step(m,d);viewer.sync()
            time.sleep(max(0,m.opt.timestep-(time.monotonic()-start)))

if __name__=='__main__':
    import sys
    if '--view' in sys.argv:view()
    else:
        result=run(render=True);print(json.dumps(result,indent=2));assert result['passed']
