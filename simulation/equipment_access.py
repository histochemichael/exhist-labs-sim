"""Contact-driven access development with actual Nori arm joints.

The chassis is fixed at a dock to isolate hand/equipment dynamics. No door
actuator, weld, mocap or live pose assignment. CAD joints/handles retained.
Masses, friction, motor/lift limits and collision proxies remain provisional.
"""
import copy,json,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from build_scene import ROOT,el,vec
from gripper_physics import SOURCE
from pose_control import multistart,solve_pose,rotation_error
from front_docking import apply_provisional_lift,add_head_cameras,PARAMETERS
import mobile_base_bench

ARM=['left_'+s+'_joint' for s in ('shoulder_pitch','shoulder_roll','bicep_yaw','elbow_pitch','forearm_yaw','wrist_pitch','wrist_roll')]
SITE='access_claw_center'
KINDS=('quincy','s60','st_load','st_unload','cv_load','ts_drawer')
MOTION_PHASES=('APPROACH','OPEN','CLOSE','RETREAT')

def rz(a):return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])

def add_box(parent,name,center,half,quat=None):
    return el(parent,'geom',name=name,type='box',pos=vec(center),size=vec(np.maximum(half,.0005)),
              **({'quat':vec(quat)} if quat is not None else {}),rgba='0 .6 .8 0',group='3',
              contype='1',conaffinity='2',density='0',friction='.6 .005 .0001',solref='.004 1')

def build(kind='quincy',jammed=False,grasp_depth=None,grip_force=None):
    if kind not in KINDS:raise ValueError(kind)
    if grasp_depth is None:grasp_depth=.116 if kind=='s60' else .114
    if grip_force is None:grip_force=1.5
    root=mobile_base_bench.build();root.set('model','Nori equipment access / fixed-dock contact development')
    root.find('option').set('timestep','.00025');root.find('option').set('noslip_iterations','0')
    world=root.find('worldbody');robot=world.find("body[@name='mobile_chassis']")
    robot.remove(robot.find('freejoint'));robot.set('pos','0 0 .002');robot.set('quat','.707106781 0 0 .707106781')
    apply_provisional_lift(root);add_head_cameras(root)
    # The imported telescopic tubes are solid box proxies, not hollow tubes.
    # Isolate ONLY the fixed inner tube, so it does not collide with its own
    # enclosing sliding stage. Keep all external tube/robot collisions enabled.
    housing=el(robot,'body',name='access_fixed_lift_tube',pos='-.113523 0 .428928')
    for g in list(robot.findall('geom')):
        if g.get('type')=='box' and np.allclose(np.fromstring(g.get('pos','0 0 0'),sep=' '),[-.113523,0,.428928]) and abs(float(g.get('size').split()[2])-.2575)<1e-5:
            robot.remove(g);g.set('pos','0 0 0');housing.append(g)
    for other in ('lift_middle_link','lift_top_link'):
        el(root.find('contact'),'exclude',name='access_nested_fixed_'+other,body1='access_fixed_lift_tube',body2=other)
    mimic=root.find("equality/joint[@name='left_gripper_mimic']")
    mimic.set('solref','.002 1');mimic.set('solimp','.999 .9999 .0001')
    for j in robot.iter('joint'):
        if j.get('range'):
            j.set('limited','true');j.set('solreflimit','.002 1');j.set('solimplimit','.999 .9999 .0001')
    # Retain actual left finger mesh shape instead of the original solid boxes.
    wrist=robot.find(".//body[@name='left_wrist_roll_link']")
    el(wrist,'site',name=SITE,pos=vec([.0025,.00652,-grasp_depth]),size='.002',rgba='0 0 0 0')
    for body in wrist.findall('body'):
        for g in list(body.findall('geom')):
            if g.get('type')=='box':body.remove(g)
            elif g.get('mesh','').startswith('gripper_'):
                c=copy.deepcopy(g);c.set('name',g.get('mesh')+'_access_contact');c.set('contype','2');c.set('conaffinity','1');c.set('group','3');c.set('rgba','0 .8 .3 0');c.set('friction','.6 .005 .0001');c.set('solref','.004 1');body.append(c)
    from left_claw_contacts import replace
    replace(root,wrist)
    act=root.find('actuator')
    for n in ARM:
        a=act.find(f"position[@name='hold_{n}']");a.set('kp','250');a.set('kv','12')
    grip=act.find("position[@name='hold_left_gripper']");grip.set('kp','30');grip.set('kv','.4');grip.set('forcerange',vec([-grip_force,grip_force]))
    src=ET.parse(ROOT/'exhist_operational.xml').getroot()
    existing={e.get('name') for e in root.find('asset')}
    for e in src.find('asset'):
        if e.get('name') in existing:continue
        e=copy.deepcopy(e)
        if e.get('file'):e.set('file',str((ROOT/e.get('file')).resolve()))
        root.find('asset').append(e)
    isdoor=kind in ('quincy','s60')
    machine_name={'quincy':'quincy_1','s60':'imaging_a_s60_pc'}.get(kind,'routine_a_workstation')
    machine=copy.deepcopy(src.find(f"worldbody/body[@name='{machine_name}']"));world.append(machine)
    from drawer_pulls import upgrade_pulls
    upgrade_pulls(root)
    for g in machine.iter('geom'):g.set('contype','0');g.set('conaffinity','0')
    if isdoor:
        moving=machine.find(f"body[@name='{machine_name}_passive_door']");joint=moving.find('joint');joint_name=joint.get('name')
        data=json.loads((ROOT/'assets'/f'{kind}.json').read_text());rot=np.eye(3) if kind=='quincy' else np.array([[1,0,0],[0,0,-1],[0,1,0]])
        whole=np.concatenate([np.array(p['vertices']).reshape(-1,3)@rot.T for p in data['parts']]);offset=np.r_[(whole.min(0)[:2]+whole.max(0)[:2])/2,whole.min(0)[2]]
        pivot=np.fromstring(moving.get('pos'),sep=' ');reference=float(joint.get('ref','0'));R=rz(reference) if kind=='quincy' else np.eye(3)
        handle_part=next(p for p in data['parts'] if ('Vertical thermoplastic handle' in p['name'] if kind=='quincy' else p['name'].startswith('cassette_door') and p['name'].endswith('/handle')))
        handle_vertices=np.array(handle_part['vertices']).reshape(-1,3)@rot.T-offset-pivot
        center=(handle_vertices.min(0)+handle_vertices.max(0))/2
        for i,part in enumerate(data['parts']):
            moving_part=part['name'].startswith('03 Door' if kind=='quincy' else 'cassette_door')
            if not moving_part:continue
            v=(np.array(part['vertices']).reshape(-1,3)@rot.T-offset-pivot)@R
            if not (part is handle_part or any(n in part['name'] for n in ('outer panel','mounting foot','sliding_cover'))):continue
            q=np.zeros(4);mujoco.mju_mat2Quat(q,R.ravel())
            add_box(moving,'access_handle' if part is handle_part else f'access_panel_{i}',R@((v.min(0)+v.max(0))/2),(v.max(0)-v.min(0))/2,q)
        if kind=='quincy':
            from quincy_offset_pull import add
            center=add(moving,center,R)
        el(moving,'site',name='access_handle_frame',pos=vec(center),size='.002',rgba='0 0 0 0')
        joint.set('frictionloss',str(10 if jammed else (.05 if kind=='quincy' else 2.)));joint.set('damping','.3' if kind=='quincy' else '3')
        desired=math.radians(70) if kind=='quincy' else -.20
        orientation=np.array([[1,0,0],[0,0,-1],[0,1,0]],float)
    else:
        cadname={'st_load':'LOAD_DRAWER_OPEN_mm','st_unload':'UNLOAD_DRAWER_OPEN_mm','cv_load':'EST_DRAWER_SLIDER','ts_drawer':'TS5025_DRAWER_OPEN_mm'}[kind]
        row=next(r for r in json.loads((ROOT/'machine_articulation.json').read_text())['joint_map'] if r['station']=='routine_a' and r['cad_name']==cadname)
        moving=machine.find(f".//body[@name='{row['body']}']");joint=moving.find('joint');joint_name=joint.get('name')
        pull=moving.find(f"site[@name='routine_a_{cadname}_pull_grasp']");center=np.fromstring(pull.get('pos'),sep=' ')
        el(moving,'site',name='access_handle_frame',pos=vec(center),size='.002',rgba='0 0 0 0')
        for g in moving.findall('geom'):
            if '_pull_' in g.get('name',''):
                g.set('contype','1');g.set('conaffinity','2');g.set('friction','.6 .005 .0001');g.set('solref','.004 1')
                if g.get('name').endswith('_bar'):g.set('name','access_handle')
        joint.set('frictionloss',str(30 if jammed else 2));joint.set('damping','3')
        moving.find('inertial').set('mass','1.5')
        desired=-.18;orientation=np.array([[0,-1,0],[0,0,-1],[1,0,0]],float)
        # CAD panel bounds rather than a whole-machine filled box.
        from drawer_pulls import designs
        r=next(r for r in designs() if r['cad_joint']==cadname)
        add_box(moving,'access_drawer_front',center+[0,.0315,0],[.043 if kind=='cv_load' else .12,.0015,.024])
    joint.set('solreflimit','.004 1');joint.set('solimplimit','.98 .999 .001')
    # Every other joint on the instrument stays at its CAD zero configuration.
    for body in machine.iter('body'):
        for j in list(body.findall('joint')):
            if j.get('name')!=joint_name:body.remove(j)
    machine.set('pos','0 .55 .8')
    # Handle-centered fixed docking, leaving enough stand-off for the hinge sweep.
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    d.qpos[m.joint(joint_name).qposadr[0]]=0;mujoco.mj_forward(m,d)
    h=d.site_xpos[m.site('access_handle_frame').id].copy()
    planfile=ROOT/f'access_{kind}_plan.json';plan=json.loads(planfile.read_text()) if planfile.exists() else {}
    if 'path' not in plan:plan={}
    robot.set('pos',vec([h[0]+plan.get('lateral',.17),h[1]-plan.get('stand',.38),.002]))
    yaw=plan.get('yaw',math.pi/2);robot.set('quat',vec([math.cos(yaw/2),0,0,math.sin(yaw/2)]))
    if plan.get('mobile'):el(robot,'freejoint',name='access_mobile_free')
    if plan:orientation=np.array(plan['orientation'])
    # Bench placement follows the real equipment base, not the handle.
    el(world,'geom',name='access_bench',type='box',pos='0 .65 .78',size='.95 .44 .02',rgba='.55 .68 .73 1')
    meta=dict(kind=kind,joint=joint_name,machine=machine_name,open_target=desired,orientation=orientation.tolist(),
              handle_closed_world=h.tolist(),lift_target=float(plan['path'][0][-1] if plan else np.clip(h[2]-.52,.43,.68)),jammed=jammed,grasp_depth_m=grasp_depth,grip_torque_limit_Nm=grip_force,plan=plan)
    return root,meta

class Access:
    def __init__(self,kind='quincy',jammed=False,miss=False,grasp_depth=None,grip_force=None):
        self.root,self.meta=build(kind,jammed,grasp_depth,grip_force);self.m=mujoco.MjModel.from_xml_string(ET.tostring(self.root,encoding='unicode'));self.d=mujoco.MjData(self.m);m=self.m;d=self.d
        self.qa=np.array([m.joint(n).qposadr[0] for n in ARM]);self.va=np.array([m.joint(n).dofadr[0] for n in ARM]);self.ai=np.array([m.actuator('hold_'+n).id for n in ARM]);self.grip=m.actuator('hold_left_gripper').id
        self.ji=m.joint(self.meta['joint']).id;self.jq=m.jnt_qposadr[self.ji];self.jv=m.jnt_dofadr[self.ji];self.sid=m.site(SITE).id;self.hsid=m.site('access_handle_frame').id
        d.qpos[self.jq]=0;d.qpos[m.joint('lift_extension_joint').qposadr[0]]=self.meta['lift_target'];d.qpos[m.joint('lift_middle_joint').qposadr[0]]=self.meta['lift_target']/2
        for side in ('left','right'):
            for name,value in zip(ARM,[0,1.25 if side=='left' else -1.25,0,-.45,0,0,0]):
                n=name.replace('left_',side+'_');d.qpos[m.joint(n).qposadr[0]]=value
        for n,v in [('left_gripper_joint',.3),('left_gripper_idler_joint',-.3)]:d.qpos[m.joint(n).qposadr[0]]=v
        mujoco.mj_forward(m,d);self.home=d.qpos.copy();self.closed=d.site_xpos[self.hsid].copy();self.R=np.array(self.meta['orientation']);self.miss=miss
        self.axis=d.xaxis[self.ji].copy();self.anchor=d.xanchor[self.ji].copy()
        self.pre=self.closed+np.array(self.meta['plan'].get('approach_delta_m',[0,-self.meta['plan'].get('approach_standoff_m',.02 if kind=='s60' else .05),0]))+[.06 if miss else 0,0,0]
        self.plan=self.meta['plan'];self.names=ARM+['lift_extension_joint'];self.qall=np.array([m.joint(n).qposadr[0] for n in self.names])
        if self.plan:
            from access_planning import fit
            d.qpos[self.qall]=self.plan['path'][0]
            solution,pe,re=fit(m,d.qpos,SITE,self.names,self.pre,self.R,iterations=300)
            q=solution[:7];d.qpos[self.qall[-1]]=solution[-1];d.qpos[m.joint('lift_middle_joint').qposadr[0]]=solution[-1]/2
        else:q,pe,re=multistart(m,d.qpos,SITE,ARM,self.pre,self.R,attempts=14)
        if pe>.002 or re>.02:raise RuntimeError(('Access approach unreachable',kind,pe,re))
        d.qpos[self.qa]=q;mujoco.mj_forward(m,d);self.qtarget=q.copy();self.home=d.qpos.copy()
        self.phase='SETTLE';self.start=0.;self.events=[];self.trace=[];self.duration=1.;self.frompos=self.pre.copy();self.target=self.pre.copy();self.dwell=0.;self.lost=0.;self.max_open=0.;self.max_torque=0.;self.slip=0.;self.bilateral=0;self.heldsamples=0;self.reference=None;self.fault_contacts=[];self.terminal=None
        self.handle=m.geom('access_handle').id
        self.jaws={i:side for i in range(m.ngeom) for side,n in enumerate(('gripper_r_mirrored','gripper_l_mirrored')) if (m.geom(i).name or '').startswith(n+'_access_contact')}
        self.geom_names=[m.geom(i).name or '' for i in range(m.ngeom)]
        self.forbidden={i for i,n in enumerate(self.geom_names) if n in ('access_bench','access_drawer_front') or n.startswith('access_panel_')}
        self.robot_geoms={i for i in range(m.ngeom) if m.body(m.geom_bodyid[i]).name.startswith(('left_','right_','lehisto_','lift_'))}
        self.holds=[]
        for ai in range(m.nu):
            n=m.actuator(ai).name
            if n.startswith('hold_') and ai not in self.ai and ai!=self.grip:
                ji=m.actuator_trnid[ai,0];self.holds.append((ai,m.jnt_qposadr[ji],m.jnt_dofadr[ji]))
        self.steps=0
        self.filtered_forces=np.zeros(2)
        self.blocked_dwell=0.;self.stage_rotation=self.R.copy()
        self.lift_command=float(d.qpos[self.qall[-1]]);self.max_lift_force=0.;self.phase_q=0.
        self.commanded_wrench=np.zeros(3);self.max_task_force=0.
        self.base_id=m.body('mobile_chassis').id;self.base_origin=d.xpos[self.base_id].copy();self.max_tilt=0.;self.max_base_error=0.
        self.wheels=[m.actuator(s+'_wheel_motor').id for s in ('left','right')]
        self.limited=np.flatnonzero(m.jnt_limited);self.max_joint_violation=0.;self.max_mimic_error=0.
        self.grip_command=.3;self.force_dwell=0.;self.pull_force_dwell=0.;self.max_measured_pull=0.;self.closed_dwell=0.
        self.hold_seconds=.5;self.release_clear_dwell=0.

    def drive_base(self):
        if not self.plan.get('mobile'):return
        m=self.m;d=self.d;cfg=self.plan['mobile'];q=float(d.qpos[self.jq]);turn=cfg['yaw_gain']*q
        goal=self.base_origin+np.array([cfg['radius_m']*(1-math.cos(turn)),-cfg['radius_m']*math.sin(turn),0.])
        matrix=d.xmat[self.base_id].reshape(3,3);yaw=math.atan2(matrix[1,0],matrix[0,0]);tilt=math.acos(float(np.clip(matrix[2,2],-1,1)))
        self.max_tilt=max(self.max_tilt,tilt)
        if tilt>math.radians(12) and self.terminal is None:self.change('TILT_FAULT')
        delta=goal-d.xpos[self.base_id];along=float(delta[:2]@np.array([math.cos(yaw),math.sin(yaw)]));cross=float(delta[:2]@np.array([-math.sin(yaw),math.cos(yaw)]))
        self.max_base_error=max(self.max_base_error,float(np.linalg.norm(delta[:2])))
        rate=cfg['yaw_gain']*float(d.qvel[self.jv]);err=mobile_base_bench.wrap(math.pi/2+turn-yaw)
        velocity=float(np.clip(-cfg['radius_m']*rate+1.8*along,-.05,.05));omega=float(np.clip(rate+4*err+np.sign(velocity)*2*cross,-.2,.2))
        if self.terminal is not None:velocity=omega=0.
        d.ctrl[self.wheels]=[(velocity-omega*.15)/.0762,(velocity+omega*.15)/.0762]

    def change(self,phase,target=None,duration=1.):
        self.phase=phase;self.start=self.d.time;self.duration=duration;self.frompos=self.d.site_xpos[self.sid].copy();self.target=self.frompos.copy() if target is None else np.array(target);self.dwell=0.;self.lost=0.
        self.events.append(dict(time=self.d.time,phase=phase,door_q=float(self.d.qpos[self.jq])))
        self.phase_q=float(self.d.qpos[self.jq])
        self.force_dwell=0.
        self.stage_rotation=self.d.site_xmat[self.sid].reshape(3,3).copy() if phase in ('GRIP','HOLD','RELEASE','COMPLETE') else self.R.copy();self.blocked_dwell=0.
        self.release_clear_dwell=0.
        # Preserve the established motor stance, including its elastic load
        # deflection. Replacing it by measured joints unloads the hand suddenly.
        # This latches COMMANDS only; all bodies still integrate under physics.
        if phase in ('COMPLETE','NO_GRASP','CONTACT_LOST','GRASP_LOST','FORCE_STOP','JAM_STOP','TRACKING_TIMEOUT','COLLISION','IK_FAULT','TILT_FAULT','CONSTRAINT_FAULT'):self.terminal=self.d.time

    def desired(self,q,orientation_q=None):
        p=self.anchor+rz(q)@(self.closed-self.anchor) if self.meta['kind']=='quincy' else self.closed+self.axis*q
        r=rz((q if orientation_q is None else orientation_q)*self.plan.get('follow',1.))@self.R if self.meta['kind']=='quincy' else self.R
        if self.reference is not None:p-=r@self.reference
        return p,r

    def step(self):
        m=self.m;d=self.d;dt=m.opt.timestep;elapsed=d.time-self.start
        u=float(np.clip(elapsed/self.duration,0,1));u=u*u*(3-2*u);R=self.stage_rotation;motion_error=0.
        if self.phase in ('OPEN','CLOSE'):
            end=self.meta['open_target'] if self.phase=='OPEN' else 0.
            q=self.phase_q+(end-self.phase_q)*u
            actual=float(d.qpos[self.jq]);lead=.006 if self.meta['kind']=='quincy' else .004
            motion_error=q-actual
            q=actual+np.clip(q-actual,-lead,lead)
            target,R=self.desired(q,actual)
        elif self.phase=='GRIP':
            # Hold the acquisition pose. Chasing a low-resistance door while
            # closing the claw feeds squeezing-induced motion back into the arm.
            target=self.target.copy();R=self.stage_rotation
        else:target=(1-u)*self.frompos+u*self.target
        if self.terminal is None and self.phase in MOTION_PHASES and self.steps%max(1,round(.02/dt))==0:
            if self.plan:
                from access_planning import fit
                # Anchor redundant IK to the whole-stroke branch. Re-seeding
                # from contact-disturbed live joints lets the lift/elbow drift
                # into a high-effort posture even while the tip error stays low.
                path=np.asarray(self.plan['path']);fraction=float(np.clip(d.qpos[self.jq]/self.meta['open_target'],0,1))
                index=fraction*(len(path)-1);low=min(int(index),len(path)-2);blend=index-low
                seed=d.qpos.copy();seed[self.qall]=(1-blend)*path[low]+blend*path[low+1]
                solution,pe,re=fit(m,seed,SITE,self.names,target,R,iterations=80)
            else:solution,pe,re=solve_pose(m,d.qpos,SITE,ARM,target,R,iterations=80)
            if pe>.004 or re>.04:self.change('IK_FAULT')
            else:
                self.qtarget+=np.clip(solution[:7]-self.qtarget,-.015,.015)
                if self.plan:self.lift_command+=np.clip(solution[-1]-self.lift_command,-.0008,.0008)
        for ai,qa,va in self.holds:d.ctrl[ai]=self.home[qa]+d.qfrc_bias[va]/120
        # Joint-space impedance alone produced less than the drawer's 2 N breakaway
        # force. Supply a bounded task wrench THROUGH THE ARM MOTORS, never the door.
        # Original 4 Nm arm caps and provisional 12 N pull-force ceiling remain.
        self.commanded_wrench[:]=0
        if self.phase in ('OPEN','CLOSE') and min(self.filtered_forces)>.10:
            direction=np.cross(self.axis,d.site_xpos[self.hsid]-self.anchor) if self.meta['kind']=='quincy' else self.axis.copy()
            direction/=np.linalg.norm(direction)
            # Ramp from measured trajectory error; an immediate full pull could
            # accelerate a light drawer ahead of the closing fingers.
            limit=3.
            scale=.004 if self.meta['kind']=='quincy' else .001
            self.commanded_wrench=direction*limit*math.tanh(motion_error/scale)
        jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jacSite(m,d,jp,jr,self.sid)
        task_torque=jp[:,self.va].T@self.commanded_wrench
        self.max_task_force=max(self.max_task_force,float(np.linalg.norm(self.commanded_wrench)))
        d.ctrl[self.ai]=self.qtarget+(d.qfrc_bias[self.va]+task_torque)/250
        li=m.actuator('provisional_lift_motor').id;lq=m.joint('lift_extension_joint').dofadr[0];mq=m.joint('lift_middle_joint').dofadr[0]
        progress=np.clip(abs(d.qpos[self.jq]/self.meta['open_target']),0,1)
        if not self.plan:
            wanted=min(.69,self.meta['lift_target']+.12*progress)
            self.lift_command+=np.clip(wanted-self.lift_command,-.04*dt,.04*dt)
        d.ctrl[li]=self.lift_command+(d.qfrc_bias[lq]+.5*d.qfrc_bias[mq])/PARAMETERS['nori_lift']['kp_N_per_m']
        gripping=self.phase in ('GRIP','OPEN','CLOSE','HOLD','FORCE_STOP','JAM_STOP','GRASP_LOST','CONTACT_LOST')
        # Effort-limited closure: a zero-angle position target supplies almost no
        # holding torque at a thin handle near the fully-closed finger position.
        # This is an arm/claw command; no equipment force or attachment is added.
        gq=m.joint('left_gripper_joint').qposadr[0]
        effort=self.meta['grip_torque_limit_Nm']
        if self.phase=='GRIP':effort*=min(1.,elapsed/.8)
        if gripping:
            self.grip_command=d.qpos[gq]-effort/30
            # Empty closure has no handle to oppose motor torque. Slow into a
            # 5 mrad software stop before the physical zero-angle joint stop.
            # Loaded nominal grasps remain above 38 mrad in the checked cycles.
            if d.qpos[gq]<.02:self.grip_command=max(.005,self.grip_command)
        else:
            self.grip_command+=np.clip(.3-self.grip_command,-.25*dt,.25*dt)
        d.ctrl[self.grip]=self.grip_command
        self.drive_base();mujoco.mj_step(m,d);self.steps+=1
        values=d.qpos[m.jnt_qposadr[self.limited]];limits=m.jnt_range[self.limited]
        violations=np.maximum(np.maximum(limits[:,0]-values,values-limits[:,1]),0.)
        self.max_joint_violation=max(self.max_joint_violation,float(violations.max(initial=0.)))
        mimic_error=abs(float(d.qpos[gq]+d.qpos[m.joint('left_gripper_idler_joint').qposadr[0]]));self.max_mimic_error=max(self.max_mimic_error,mimic_error)
        tolerance=np.where(m.jnt_type[self.limited]==mujoco.mjtJoint.mjJNT_SLIDE,.001,.005)
        if self.terminal is None and (np.any(violations>tolerance) or mimic_error>.005):self.change('CONSTRAINT_FAULT')
        self.max_lift_force=max(self.max_lift_force,abs(float(d.actuator_force[li])))
        forces=[0.,0.];unexpected=[];handle_force=np.zeros(3)
        for ci,c in enumerate(d.contact):
            g1,g2=int(c.geom1),int(c.geom2)
            jaw=g2 if g1==self.handle else g1 if g2==self.handle else -1
            if jaw in self.jaws:
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);forces[self.jaws[jaw]]+=max(0,float(f[0]))
                handle_force+=(1 if g2==self.handle else -1)*(np.asarray(c.frame).reshape(3,3).T@f[:3])
            if c.dist<-.002 and ((g1 in self.forbidden and g2 in self.robot_geoms) or (g2 in self.forbidden and g1 in self.robot_geoms)):
                unexpected.append([self.geom_names[g1],self.geom_names[g2]])
        self.max_open=max(self.max_open,abs(float(d.qpos[self.jq])));self.max_torque=max(self.max_torque,float(np.max(np.abs(d.actuator_force[self.ai]))))
        # 20 ms force-sensor filter; zero-contact cases still cannot acquire a grasp.
        self.filtered_forces+=(np.asarray(forces)-self.filtered_forces)*(1-math.exp(-dt/.02))
        # Never retreat merely because an opening timer expired. The fingers
        # must be visibly open AND both measured handle contacts clear for 100 ms.
        release_clear=self.phase=='RELEASE' and d.qpos[gq]>.28 and max(self.filtered_forces)<.02
        self.release_clear_dwell=self.release_clear_dwell+dt if release_clear else 0.
        direction=np.cross(self.axis,d.site_xpos[self.hsid]-self.anchor) if self.meta['kind']=='quincy' else self.axis.copy()
        direction/=np.linalg.norm(direction)
        measured_pull=abs(float(handle_force@direction))
        closed_tolerance=.002 if self.meta['kind']=='quincy' else .0005
        seated=self.phase=='CLOSE' and elapsed>self.duration*.75 and abs(d.qpos[self.jq])<closed_tolerance and abs(d.qvel[self.jv])<.002 and min(self.filtered_forces)>.1
        self.closed_dwell=self.closed_dwell+dt if seated else 0.
        # A mechanically seated door must not continue being driven simply to
        # finish a timed trajectory. Require observed closed position and dwell;
        # the same force away from closed remains a fault, never success.
        if self.closed_dwell>.03 and self.terminal is None:
            self.change('RELEASE',d.site_xpos[self.sid].copy(),1.5)
        if self.phase in ('OPEN','CLOSE'):
            self.max_measured_pull=max(self.max_measured_pull,measured_pull)
            self.pull_force_dwell=self.pull_force_dwell+dt if measured_pull>12. else 0.
            if self.pull_force_dwell>.03 and self.terminal is None:self.change('FORCE_STOP')
        else:self.pull_force_dwell=0.
        if unexpected and self.terminal is None:self.fault_contacts=unexpected;self.change('COLLISION')
        error=float(np.linalg.norm(target-d.site_xpos[self.sid]));at=elapsed>=self.duration and error<.003
        if self.terminal is None:
            if self.phase=='SETTLE' and at:self.change('APPROACH',self.closed+[.06 if self.miss else 0,0,0],3)
            elif self.phase=='APPROACH' and at:self.change('GRIP',self.target,.3)
            elif self.phase=='GRIP':
                aligned=np.linalg.norm(d.site_xpos[self.hsid]-d.site_xpos[self.sid])<.003
                self.dwell=self.dwell+dt if aligned and elapsed>.8 and min(self.filtered_forces)>.10 and abs(d.qvel[self.jv])<(.02 if self.meta['kind']=='quincy' else .005) else 0
                if self.dwell>.15:
                    self.reference=d.site_xmat[self.sid].reshape(3,3).T@(d.site_xpos[self.hsid]-d.site_xpos[self.sid]);self.change('OPEN',duration=20)
                elif elapsed>4:self.change('NO_GRASP')
            elif self.phase in ('OPEN','CLOSE','HOLD'):
                both=min(forces)>.02;self.heldsamples+=1;self.bilateral+=int(both);self.lost=0 if both else self.lost+dt
                relative=d.site_xmat[self.sid].reshape(3,3).T@(d.site_xpos[self.hsid]-d.site_xpos[self.sid]);self.slip=max(self.slip,float(np.linalg.norm(relative-self.reference)))
                trying=self.phase in ('OPEN','CLOSE') and abs(self.meta['open_target']*(u if self.phase=='OPEN' else 1-u)-d.qpos[self.jq])>(.02 if self.meta['kind']=='quincy' else .005)
                self.blocked_dwell=self.blocked_dwell+dt if trying and abs(d.qvel[self.jv])<.0005 and min(self.filtered_forces)>.1 else 0
                self.force_dwell=self.force_dwell+dt if np.max(np.abs(d.actuator_force[self.ai]))>3.95 and abs(d.qvel[self.jv])<.001 else 0.
                if self.lost>.10:self.change('CONTACT_LOST')
                elif self.slip>.006:self.change('GRASP_LOST')
                elif self.blocked_dwell>.7:self.change('JAM_STOP')
                elif self.force_dwell>.2:self.change('FORCE_STOP')
                elif self.phase=='OPEN' and at:
                    if abs(d.qpos[self.jq]-self.meta['open_target'])<(.03 if self.meta['kind']=='quincy' else .004):self.change('HOLD',target,duration=self.hold_seconds)
                elif self.phase=='HOLD' and elapsed>self.duration:self.change('CLOSE',duration=20)
                elif self.phase=='CLOSE' and at and abs(d.qpos[self.jq])<(.015 if self.meta['kind']=='quincy' else .003):self.change('RELEASE',target,1.5)
            elif self.phase=='RELEASE' and elapsed>1.5 and self.release_clear_dwell>=.1:self.change('RETREAT',self.pre,3)
            elif self.phase=='RETREAT' and at:self.change('COMPLETE')
            if d.time-self.start>self.duration+(20 if self.phase in ('OPEN','CLOSE') else 5) and self.terminal is None:self.change('TRACKING_TIMEOUT')
        if self.steps%max(1,round(.1/dt))==0:self.trace.append(dict(t=d.time,phase=self.phase,door_q=float(d.qpos[self.jq]),jaw_N=forces,position_error_m=error,arm_torque_Nm=d.actuator_force[self.ai].tolist(),commanded_task_force_N=self.commanded_wrench.tolist()))
        return self.terminal is not None and d.time-self.terminal>.5

    def result(self):
        return dict(kind=self.meta['kind'],phase=self.phase,passed=self.phase=='COMPLETE',max_open=self.max_open,open_target=self.meta['open_target'],final_q=float(self.d.qpos[self.jq]),max_arm_torque_Nm=self.max_torque,max_commanded_task_force_N=self.max_task_force,relative_slip_m=self.slip,bilateral_contact_fraction=self.bilateral/max(1,self.heldsamples),events=self.events,trace=self.trace,fault_contacts=self.fault_contacts,lift_target_m=self.meta['lift_target'],max_lift_force_N=self.max_lift_force,grip_torque_limit_Nm=self.meta['grip_torque_limit_Nm'],grasp_depth_m=self.meta['grasp_depth_m'],max_joint_limit_violation=self.max_joint_violation,max_left_claw_mimic_error_rad=self.max_mimic_error,handle_interface='PROPOSED stand-off round pull; mounting/thermal/load validation pending' if self.meta['kind']=='quincy' else 'v02 proposed drawer pull' if self.meta['kind'] not in ('quincy','s60') else 'OEM CAD handle',
                    max_measured_handle_pull_N=self.max_measured_pull,measured_pull_stop_N=12.,scope='Actual seven-joint left arm, fixed dock, passive CAD equipment joint. Provisional dynamics and conservative finger mesh contacts; not mobile/lab transfer or hardware validation.')

def run(kind='quincy',jammed=False,miss=False,render=False,grasp_depth=None,grip_force=None):
    a=Access(kind,jammed,miss,grasp_depth,grip_force)
    for _ in range(round(100/a.m.opt.timestep)):
        if a.step():break
    result=a.result();suffix='_jammed' if jammed else '_miss' if miss else ''
    (ROOT/f'access_{kind}{suffix}_validation.json').write_text(json.dumps(result,indent=2))
    if render:
        from PIL import Image,ImageDraw,ImageFont
        with mujoco.Renderer(a.m,height=900,width=1280) as renderer:
            c=mujoco.MjvCamera();c.lookat[:]=a.closed+[0,0,-.18];c.distance=1.65;c.azimuth=125;c.elevation=-20
            opt=mujoco.MjvOption();opt.sitegroup[:]=0;renderer.update_scene(a.d,c,scene_option=opt);im=Image.fromarray(renderer.render());dr=ImageDraw.Draw(im)
            dr.rectangle([0,0,1280,70],fill='#10283c');dr.text((15,12),f'Nori access DEVELOPMENT | {kind} | {a.phase}',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',27),fill='white');dr.text((15,43),'Actual arm + passive equipment joint | fixed dock | provisional dynamics',fill='#ffd190');im.save(ROOT/f'access_{kind}.png')
    return result

def view(kind='quincy'):
    """Inspect one contact attempt, then hold the terminal diagnostic pose."""
    import time
    import mujoco.viewer
    a=Access(kind);finished=False
    with mujoco.viewer.launch_passive(a.m,a.d) as viewer:
        viewer.cam.lookat[:]=a.closed+[0,0,-.18];viewer.cam.distance=1.65;viewer.cam.azimuth=125;viewer.cam.elevation=-20
        viewer.opt.sitegroup[:]=0
        while viewer.is_running():
            start=time.monotonic()
            for _ in range(round(.016/a.m.opt.timestep)):
                done=a.step()
                if done and not finished:
                        finished=True
                        (ROOT/f'access_{kind}_validation.json').write_text(json.dumps(a.result(),indent=2))
            viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                f'ACCESS DEVELOPMENT | {kind} | {a.phase}\nFixed dock; passive equipment joint; provisional dynamics\nFull-lab transfers remain guarded.', '')])
            viewer.sync();time.sleep(max(0,.016-(time.monotonic()-start)))

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--kind',choices=KINDS,default='quincy');parser.add_argument('--jammed',action='store_true');parser.add_argument('--miss',action='store_true');parser.add_argument('--view',action='store_true');args=parser.parse_args()
    if args.view:view(args.kind)
    else:
        r=run(args.kind,args.jammed,args.miss,render=True);print(json.dumps({k:v for k,v in r.items() if k!='trace'},indent=2))
