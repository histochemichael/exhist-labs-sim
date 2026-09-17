"""End-to-end first-pass workflow REVIEW, not a contact-physics success claim.

Leica internals replay the actual three-minute CAD choreography. Robot payloads
are attached to their groove frame during transit. Access and carrier exchange
remain explicitly provisional kinematic abstractions; physical benches are separate.
"""
import argparse,json,math,time
from collections import deque
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from run_lab import ROOT,camera
from workflow import Lab,CAPACITY
from pose_control import rotation_error
from first_pass_machines import FilmMachines
from folder_finish import FolderLab,FolderMotion,FOLDER_CENTERS,FOLDER_Y,FOLDER_Z,BENCH_X,BENCH_Y
from nori_posture import BENCH_VIEW_LIFT_M

JOINTS=['lift_extension_joint']+['right_'+s+'_joint' for s in ('shoulder_pitch','shoulder_roll','bicep_yaw','elbow_pitch','forearm_yaw','wrist_pitch','wrist_roll')]
LEFT_JOINTS=['lift_extension_joint']+[n.replace('right_','left_') for n in JOINTS[1:]]
KINDS=('leica_rack','rack24','output_magazine','scanner_cassette','slide_folder')
HEIGHTS=dict(leica_rack=.099,rack24=.0925,output_magazine=.1115,scanner_cassette=.1315,slide_folder=.0045)
GRASP_ROTATION=np.array([[-1,0,0],[0,0,1],[0,1,0]],float)
FOLDER_ROTATION=np.array([[0,1,0],[0,0,1],[1,0,0]],float)
# Keep Nori's shoulder center above the 800 mm bench, not a low-lift reach-up pose.
# This is a review posture preference within the original 0..700 mm lift range.
BENCH_LIFT_MIN=.43

def grasp_offset(kind):
    if kind=='slide_folder':return np.array([-.098,-.10,HEIGHTS[kind]])
    if kind=='scanner_cassette':return np.array([0,-.038,HEIGHTS[kind]])
    return np.array([0,0,HEIGHTS[kind]])
def carrier_side(kind):return 'left' if kind in ('scanner_cassette','slide_folder','output_magazine') else 'right'
def carrier_rotation(kind):return yaw_matrix(math.pi/2) if kind=='rack24' else np.eye(3)
def grasp_rotation(kind):return np.diag([1.,-1.,-1.]) if carrier_side(kind)=='left' else GRASP_ROTATION
def yaw_matrix(angle):return np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])

def smooth(u):
    u=np.clip(u,0,1);return u*u*(3-2*u)

def kinematics(m,d):
    d.qpos[m.joint('lift_middle_joint').qposadr[0]]=.5*d.qpos[m.joint('lift_extension_joint').qposadr[0]]
    mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d);mujoco.mj_camlight(m,d)

def ik(m,d,site,names,target,rotation=None,attempts=None,iterations=170):
    qa=np.array([m.joint(n).qposadr[0] for n in names]);va=np.array([m.joint(n).dofadr[0] for n in names])
    sid=m.site(site).id;jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);best=None;score=float('inf')
    rng=np.random.default_rng(26)
    lift_min=float(np.clip(np.asarray(target)[2]-.48,.30,BENCH_LIFT_MIN))
    lift_max=.7
    if 'lift_extension_joint' in names:
        qi=m.joint('lift_extension_joint').qposadr[0]
        d.qpos[qi]=np.clip(d.qpos[qi],lift_min,lift_max)
    for attempt in range(attempts if attempts is not None else (12 if rotation is not None else 1)):
        if attempt:
            for name,q in zip(names,qa):d.qpos[q]=rng.uniform(*m.jnt_range[m.joint(name).id])
        if 'lift_extension_joint' in names:d.qpos[m.joint('lift_extension_joint').qposadr[0]]=np.clip(d.qpos[m.joint('lift_extension_joint').qposadr[0]],lift_min,lift_max)
        for _ in range(iterations):
            kinematics(m,d);ep=np.asarray(target)-d.site_xpos[sid]
            er=rotation_error(rotation,d.site_xmat[sid].reshape(3,3)) if rotation is not None else np.zeros(3)
            metric=np.linalg.norm(ep)+.1*np.linalg.norm(er)
            if metric<score:score=metric;best=(d.qpos[qa].copy(),float(np.linalg.norm(ep)),float(np.linalg.norm(er)))
            if np.linalg.norm(ep)<.0004 and np.linalg.norm(er)<.005:break
            mujoco.mj_jacSite(m,d,jp,jr,sid)
            if 'lift_extension_joint' in names:
                full=m.joint('lift_extension_joint').dofadr[0];half=m.joint('lift_middle_joint').dofadr[0]
                jp[:,full]+=.5*jp[:,half];jr[:,full]+=.5*jr[:,half]
            j=np.vstack((jp[:,va],.1*jr[:,va])) if rotation is not None else jp[:,va]
            e=np.r_[ep,.1*er] if rotation is not None else ep
            dq=j.T@np.linalg.solve(j@j.T+np.eye(len(e))*1e-5,e);d.qpos[qa]+=np.clip(dq,-.1,.1)
            for name,q in zip(names,qa):
                lo,hi=m.jnt_range[m.joint(name).id]
                if name=='lift_extension_joint':lo=max(lo,lift_min);hi=min(hi,lift_max)
                d.qpos[q]=np.clip(d.qpos[q],lo,hi)
        if best[1]<.0004 and best[2]<.005:break
    d.qpos[qa]=best[0];return best

class Scene:
    def __init__(self,guarded=True):
        self.layout=json.loads((ROOT/'layout.json').read_text())
        self.m=mujoco.MjModel.from_xml_path(str(ROOT/'exhist_first_pass.xml'));self.d=mujoco.MjData(self.m)
        self.lab=FolderLab(self.layout['stations'])
        self.guarded=guarded;self.interaction_hold=None;self._rest_at={}
        # Allow the first individual coverslipping sequence to remain legible.
        for job in self.lab.jobs:
            for step in job.route:
                if step.name=='stain':step.seconds=55
                if step.name=='coverslip':step.seconds=70
                if step.name=='special_stain':step.seconds=28
        self.machines=FilmMachines(self.m,self.d)
        self.q={self.m.joint(i).name:int(self.m.jnt_qposadr[i]) for i in range(self.m.njnt)}
        self.qa=np.array([self.q[n] for n in JOINTS]);self.left_qa=np.array([self.q[n] for n in LEFT_JOINTS]);self.clock=0;self.rx=self.lab.robot_x
        self.robot_pose=np.array([self.rx,-.8,math.pi/2]);self.previous_pose=self.robot_pose.copy()
        self.d.qpos[self.q['lift_extension_joint']]=BENCH_VIEW_LIFT_M;self.d.qpos[self.q['lift_middle_joint']]=BENCH_VIEW_LIFT_M/2
        self.d.qpos[self.q['base_yaw']]=math.pi/2
        self.d.mocap_pos[:]=[0,0,-10]
        self.held=None;self.custody=[];self.transfers={};self.max_heading_error=0.;self.max_attachment_error=0.
        self.plans={};scratch=mujoco.MjData(self.m);scratch.qpos[:]=self.d.qpos
        # Retain the previous IK seed/branch. This change raises the idle view,
        # not the existing task-specific reach trajectories or grasp frames.
        scratch.qpos[self.q['lift_extension_joint']]=BENCH_LIFT_MIN
        scratch.qpos[self.q['lift_middle_joint']]=BENCH_LIFT_MIN/2
        # Rotate the equipment-front bench's gripper orientation with Nori's base,
        # not only its target point. Jaw closure is across the rack handle.
        for kind in KINDS:
            rot=grasp_rotation(kind)
            side=carrier_side(kind);names=LEFT_JOINTS if side=='left' else JOINTS
            site='nori_left_carrier_grasp' if side=='left' else 'nori_right_handle_groove'
            z=.805+HEIGHTS[kind];poses=[];errors=[]
            for target in ([(-.2 if side=='left' else .2),-.8,z],[(-.2 if side=='left' else .2),-.8,z+.07],[(-.2 if side=='left' else .2),-.8,z+.07]):
                q,pe,re=ik(self.m,scratch,site,names,target,rot)
                poses.append(q);errors.append([pe,re])
            if max(p[0] for p in errors)>.004 or max(p[1] for p in errors)>.04:
                raise RuntimeError('First-pass arm pose unreachable: '+str((kind,errors)))
            self.plans[kind]=np.array(poses)
            print('Nori poses ready:',kind,flush=True)
        self.robot_plans={}
        self.height_plans={}
        for name,station,slot in [('sorter_1','sorting',0),('sorter_2','sorting',1),('special_lehisto','special',0),('sendout_lehisto_1','sendout',0),('sendout_lehisto_2','sendout',1)]:
            names=[name+'_carriage']+[name+'_j'+str(i) for i in range(1,6)]
            qs=np.array([self.q[n] for n in names]);base=self.m.body(name).pos
            poses=[]
            for offset in ([-.18,-.12,.22],[-.08,-.18,.18],[-.02,-.10,.23]):
                p,_,_=ik(self.m,scratch,name+'_tcp',names,base+offset);poses.append(p)
            self.robot_plans[name]=(station,slot,qs,np.array([*poses,poses[0]]))
            print('LeHisto poses ready:',name,flush=True)
        self.rest_arms()
        self.folders=FolderMotion(self)
        from special_rack_motion import SpecialRackMotion
        self.special=SpecialRackMotion(self)
        self.sync()

    def reset(self):
        mujoco.mj_resetData(self.m,self.d);self.lab=FolderLab(self.layout['stations'])
        for job in self.lab.jobs:
            for step in job.route:
                if step.name=='stain':step.seconds=55
                if step.name=='coverslip':step.seconds=70
                if step.name=='special_stain':step.seconds=28
        self.robot_pose=np.array([self.lab.robot_x,-.8,math.pi/2]);self.previous_pose=self.robot_pose.copy()
        self.interaction_hold=None;self._rest_at.clear()
        self.transfers.clear();self.custody.clear();self.held=None
        self.machines.last.clear();self.machines.seen.clear();self.machines.rack_trace.clear();self.machines.slide_counts.clear()
        self.max_heading_error=0.;self.d.qpos[self.qa]=self.plans['leica_rack'][2];self.sync()
        self.folders.approaches.clear()

    def stage(self,station,slot=0,kind='leica_rack'):
        if station=='finished_bench':return np.array([BENCH_X[slot],BENCH_Y,.83])
        if station=='sendout':return np.array([FOLDER_CENTERS[slot]+(0 if kind=='slide_folder' else .275),FOLDER_Y,FOLDER_Z])
        if station in ('routine_a','routine_b'):
            base=np.asarray(self.machines.meta['bases'][station]);offset=np.asarray(self.machines.meta['offset_m'])
            # Same CAD poses as the film: open ST drawer or indexed CV output.
            local=[-.173,-.08,.2295] if kind=='output_magazine' else [.888955,-.43495,.134314]
            return base-offset+local
        # Front buffer lanes are provisional until each instrument insertion is validated.
        if station=='sorting':
            return np.array([self.lab.x[station]+(-.46 if slot%2==0 else .46)+(.15 if slot>=2 else 0),.34,.8])
        if station=='special':
            from special_rack_motion import bath_point
            finished=any('special_stain' in j.trace for j in self.lab.jobs if j.protocol=='SPECIAL')
            return bath_point(8 if finished else 4)
        return np.array([self.lab.x[station]+(slot-(CAPACITY[station]-1)/2)*.22,.22,.8])

    def dock(self,port,side='right'):
        heading=-math.pi/2 if port[1]<-2 else math.pi/2
        offset=yaw_matrix(heading-math.pi/2)@np.array([-.2 if side=='left' else .2,.2,0])
        return np.array([port[0]-offset[0],port[1]-offset[1],heading])

    def route(self,a,b,u):
        # Stop/turn, back into aisle, travel forward, turn to face equipment, dock.
        heading=0. if b[0]>=a[0] else math.pi
        way=[a,np.array([a[0],-.8,a[2]]),np.array([a[0],-.8,heading]),
             np.array([b[0],-.8,heading]),np.array([b[0],-.8,b[2]]),b]
        times=[0,.18,.3,.72,.84,1.]
        for i in range(5):
            if u<=times[i+1]:
                f=smooth((u-times[i])/(times[i+1]-times[i]));return (1-f)*way[i]+f*way[i+1]
        return b.copy()

    def put(self,job,pos,quat=None,kind=None):
        kind=kind or job.carrier;mid=int(self.m.body_mocapid[self.m.body(job.id+'_'+kind).id])
        self.d.mocap_pos[mid]=pos
        if quat is None:
            quat=np.zeros(4);mujoco.mju_mat2Quat(quat,carrier_rotation(kind).ravel())
        self.d.mocap_quat[mid]=quat

    def rest_arms(self,except_side=None):
        # Source zero pose is a T pose. Relax the unused arm downward instead.
        for side in ('left','right'):
            if side==except_side:continue
            names=LEFT_JOINTS if side=='left' else JOINTS
            qa=[self.q[n] for n in names[1:]];target=np.array([0,1.25 if side=='left' else -1.25,0,-.45,0,0,0])
            previous=self._rest_at.get(side);dt=0 if previous is None else max(0,self.lab.time-previous)
            self.d.qpos[qa]=target if previous is None else self.d.qpos[qa]+np.clip(target-self.d.qpos[qa],-.9*dt,.9*dt)
            self._rest_at[side]=self.lab.time

    def set_carrier_jaw(self,kind,closed):
        if carrier_side(kind)=='left':
            self.d.qpos[self.q['left_gripper_joint']]=.25 if closed else .65
            self.d.qpos[self.q['left_gripper_idler_joint']]=-.25 if closed else -.65
        else:
            grip=-.0355 if closed else 0.
            for side in ('a','b'):self.d.qpos[self.q['lehisto_jaw_'+side+'_slide']]=grip
            self.d.qpos[self.q['lehisto_pinion_joint']]=np.clip(grip/.0072,*self.m.jnt_range[self.m.joint('lehisto_pinion_joint').id])

    def height_plan(self,kind,height):
        """Solve the actual port height; never add lift travel after solving IK."""
        if abs(height-.805)<1e-8:return self.plans[kind]
        key=(kind,round(float(height),6))
        if key not in self.height_plans:
            scratch=mujoco.MjData(self.m);scratch.qpos[self.q['base_yaw']]=math.pi/2
            side=carrier_side(kind);names=LEFT_JOINTS if side=='left' else JOINTS
            qa=self.left_qa if side=='left' else self.qa;scratch.qpos[qa]=self.plans[kind][0]
            site='nori_left_carrier_grasp' if side=='left' else 'nori_right_handle_groove';poses=[]
            for dz in (0,.07,.07):
                q,pe,re=ik(self.m,scratch,site,names,[(-.2 if side=='left' else .2),-.8,height+HEIGHTS[kind]+dz],grasp_rotation(kind))
                if pe>.004 or re>.04:raise RuntimeError(('Equipment-height pose unreachable',key,pe,re))
                poses.append(q)
            self.height_plans[key]=np.array(poses)
        return self.height_plans[key]

    def update_robot(self):
        tr=self.lab.transport
        if not tr:return
        key=(tr['job'],tr['source'],tr['target'],tr['slot'])
        job=next(j for j in self.lab.jobs if j.id==tr['job'])
        if key not in self.transfers:
            # Give every trip explicit turn/dock time even for adjacent equipment.
            tr['approach']=max(tr['approach'],5.);tr['travel']=max(tr['travel'],6.)
            tr['total']=tr['approach']+tr['travel']+4
            self.transfers[key]=self.robot_pose.copy()
        a=tr['approach'];v=tr['travel'];t=tr['elapsed'];kind=job.carrier
        side=carrier_side(kind);qa=self.left_qa if side=='left' else self.qa
        self.rest_arms(except_side=side)
        source=self.stage(tr['source'],tr['source_slot'],kind);dest=self.stage(tr['target'],tr['slot'],kind)
        source_rot=yaw_matrix(self.dock(source)[2]-math.pi/2);dest_rot=yaw_matrix(self.dock(dest)[2]-math.pi/2)
        offset=grasp_offset(kind)
        source_dock=self.dock(source+source_rot@np.array([offset[0],offset[1],0]),side)
        dest_dock=self.dock(dest+dest_rot@np.array([offset[0],offset[1],0]),side)
        pick,up,carry=self.height_plan(kind,source[2])
        dest_pick,dest_up,dest_carry=self.height_plan(kind,dest[2])
        if t<a:
            self.robot_pose=self.route(self.transfers[key],source_dock,t/a)
            rest=carry.copy();rest[1:]=[0,1.25 if side=='left' else -1.25,0,-.45,0,0,0]
            u=smooth((t/a-.8)/.2);q=(1-u)*rest+u*carry
        elif t<a+2:
            self.robot_pose=source_dock;u=(t-a)/2
            if u<.35:q=(1-smooth(u/.35))*carry+smooth(u/.35)*pick
            else:q=(1-smooth((u-.35)/.65))*pick+smooth((u-.35)/.65)*carry
        elif t<a+2+v:
            u=smooth((t-a-2)/v)
            self.robot_pose=self.route(source_dock,dest_dock,(t-a-2)/v);q=(1-u)*carry+u*dest_carry
        else:
            self.robot_pose=dest_dock;u=(t-a-2-v)/2
            if u<.65:q=(1-smooth(u/.65))*dest_carry+smooth(u/.65)*dest_pick
            else:q=(1-smooth((u-.65)/.35))*dest_pick+smooth((u-.65)/.35)*dest_carry
        self.d.qpos[qa]=q
        # Every endpoint already includes the port height within CAD lift limits.
        self.d.qpos[self.q['base_x']]=self.robot_pose[0]
        self.d.qpos[self.q['base_y']]=self.robot_pose[1]+1 # source base origin is y=-1
        self.d.qpos[self.q['base_yaw']]=self.robot_pose[2]
        delta=self.robot_pose-self.previous_pose
        forward=np.dot(delta[:2],[math.cos(self.robot_pose[2]),math.sin(self.robot_pose[2])])
        for name,sign in [('left_wheel_joint',-1),('right_wheel_joint',1)]:
            self.d.qpos[self.q[name]]+=(forward+sign*.145*delta[2])/.075
        self.previous_pose=self.robot_pose.copy();self.rx=self.robot_pose[0]
        grasping=a+.7<=t<a+2+v+1.3
        self.set_carrier_jaw(kind,grasping)
        kinematics(self.m,self.d)
        sid=self.m.site('nori_left_carrier_grasp' if side=='left' else 'nori_right_handle_groove').id
        if grasping:
            r=self.d.site_xmat[sid].reshape(3,3)
            # Fixed relative transform: the visual payload cannot drift off the groove.
            # This is an attachment, NOT evidence of bilateral contact or retention.
            desired=grasp_rotation(kind)
            relative=desired.T@(-grasp_offset(kind))
            pos=self.d.site_xpos[sid]+r@relative
            rot=r@desired.T@carrier_rotation(kind);quat=np.zeros(4);mujoco.mju_mat2Quat(quat,rot.ravel())
            self.put(job,pos,quat)
            if self.held!=job.id:self.custody.append(dict(time=self.lab.time,batch=job.id,event='visual_groove_attach',source=tr['source']));self.held=job.id
        elif t<a+.7:self.put(job,source)
        else:
            quat=np.zeros(4);mujoco.mju_mat2Quat(quat,(dest_rot@carrier_rotation(kind)).ravel());self.put(job,dest,quat)
            if self.held:self.custody.append(dict(time=self.lab.time,batch=job.id,event='visual_supported_release',target=tr['target']));self.held=None
        if (a<=t<a+2) or t>=a+2+v:
            wanted=source_dock[2] if t<a+2 else dest_dock[2]
            self.max_heading_error=max(self.max_heading_error,abs(self.robot_pose[2]-wanted))
        # Passive doors NEVER advance from a workflow timer. The guarded viewer
        # stops at unvalidated access; storyboard mode leaves doors closed.

    def sync(self):
        self.d.mocap_pos[:]=[0,0,-10]
        self.rest_arms()
        self.d.qpos[self.q['base_x']]=self.robot_pose[0]
        self.d.qpos[self.q['base_y']]=self.robot_pose[1]+1
        self.d.qpos[self.q['base_yaw']]=self.robot_pose[2]
        for name in ('quincy_1_door_joint','quincy_2_door_joint','quincy_3_door_joint','imaging_a_s60_pc_door_joint','imaging_b_s60_pc_door_joint'):
            self.d.qpos[self.q[name]]=0.
        for job in self.lab.jobs:
            if job.holds in ('routine_a','routine_b') and not (self.lab.transport and self.lab.transport['job']==job.id):continue
            station=job.location if job.location!='nori' else 'sorting'
            slot=job.slot if job.state!='WAITING' else self.lab.jobs.index(job)
            self.put(job,self.stage(station,slot,job.carrier))
            if station=='finished_bench':self.put(job,self.stage(station,slot,job.carrier),[0,0,0,1])
        for name,(station,slot,qs,poses) in self.robot_plans.items():
            job=next((j for j in self.lab.jobs if j.holds==station and j.slot==slot and j.state=='PROCESSING'),None)
            if job and job.route[job.index].name in ('package','close_folder'):
                self.d.qpos[qs]=poses[0];job=None
            if job:
                elapsed=job.route[job.index].seconds-job.remaining;u=(elapsed%5)/5*3;i=min(int(u),2);f=smooth(u-i)
                self.d.qpos[qs]=(1-f)*poses[i]+f*poses[i+1]
                jaw=-.012 if .8<u<2.4 else 0
                for side in ('left','right'):self.d.qpos[self.q[name+'_jaw_'+side]]=jaw
                self.d.qpos[self.q[name+'_pinion']]=jaw/.0072
                self.d.qpos[self.q[name+'_screw']]=self.d.qpos[self.q[name+'_carriage']]*2*math.pi/.008
        self.update_robot();self.folders.update();self.special.update();self.machines.update(self.lab);kinematics(self.m,self.d)
        # A representative slide illustrates the active LeHisto task, with explicit scope in report.
        # Do not create an unsupported "representative" slide on an empty hand.
        active=None
        if active:
            name,job=active;sid=self.m.site(name+'_slide_groove').id;mid=self.m.body_mocapid[self.m.body('first_pass_slide').id]
            rot=self.d.site_xmat[sid].reshape(3,3)
            # Original lower groove, not the old provisional jaw-center TCP.
            reference=np.array([[1,0,0],[0,0,-1],[0,1,0]],float)
            orient=rot@reference.T
            self.d.mocap_pos[mid]=self.d.site_xpos[sid]+orient@np.array([0,0,-.029])
            mujoco.mju_mat2Quat(self.d.mocap_quat[mid],orient.ravel())
        kinematics(self.m,self.d)

    def tick(self,dt):
        if self.interaction_hold:return
        while dt>1e-8:
            part=min(dt,.05)
            if self.guarded and self.lab.transport:
                from interaction_guards import transfer_blockers,hold_time
                tr=self.lab.transport;j=next(j for j in self.lab.jobs if j.id==tr['job'])
                reasons=transfer_blockers(tr,j.carrier);stop=hold_time(tr,j.carrier)
                if reasons and tr['elapsed']+part>=stop:
                    remainder=max(0,stop-tr['elapsed'])
                    if remainder:self.lab.tick(remainder)
                    self.interaction_hold=dict(batch=j.id,source=tr['source'],target=tr['target'],reasons=reasons)
                    self.lab.paused=True;self.lab.event(j,'interaction_hold',reasons=reasons);break
            self.lab.tick(part);dt-=part
            # Configure transfer durations before a larger tick can overrun one.
            if self.lab.transport and self.lab.transport['elapsed']==0:self.sync()
        self.sync()

    def status(self,speed=1):
        state='INTERACTION HOLD' if self.interaction_hold else 'STORYBOARD COMPLETE' if self.lab.completed==36 else 'PAUSED' if self.lab.paused else 'RUNNING'
        return '\n'.join([f'ExHist FIRST PASS | {state} | {self.lab.completed}/36 slides | {self.lab.time:.1f}s | {speed:g}x',
                          'KINEMATIC REVIEW - not validated grasp/contact physics',
                          *([] if not self.interaction_hold else self.interaction_hold['reasons']),
                          *[f'{j.id} {j.protocol}: {j.route[j.index].name if j.index<len(j.route) else "packaged"} / {j.location}' for j in self.lab.jobs],
                          self.machines.status()])

    def save(self,filename='first_pass_live.json'):
        report=self.lab.report();report.update(mode='FIRST_PASS_KINEMATIC_WORKFLOW',physical_success=False,
            guarded=self.guarded,interaction_hold=self.interaction_hold,full_lab_transfers_validated=False,
            source_video=self.machines.meta['source'],machine_rack_trace=self.machines.rack_trace,
            observed_coverslipped_counts=self.machines.slide_counts,
            visual_custody=self.custody,equipment_facing_error_deg=math.degrees(self.max_heading_error),
            folders=[dict(batch=j.id,slides_seated=j.folder_loaded,closed=j.folder_closed,location=j.location,slot=j.slot) for j in self.lab.jobs],
            limitations=['Playback is not force-driven simulation; unvalidated access stops the guarded viewer. Passive doors never move on a timer.',
                        'External handoffs use front buffers; incubator/scanner insertion and drawer contact remain unvalidated.',
                        'Special rack to Leica rack and output magazine to scanner cassette exchanges are workflow abstractions.',
                        'Special LeHisto has five reach-checked bath positions; six other jars are spare positions, not cleared active baths. Contact and insertion remain unvalidated.',
                        'Nori fills the real folder slots from a provisional flat-slide staging point, closes both passive flaps, and carries the closed folder to the finished bench; crease/contact forces remain unvalidated.',
                        'Machine internals use the actual v3 film motion, with the batch-specific slide count.',
                        'Original 28-slot Leica rack reserves center slots 15 and 16.',
                        'Legacy machine joint coordinates remain at source pose: visible machine motion uses film rigid-body transforms, not a force-driven joint graph.'])
        (ROOT/filename).write_text(json.dumps(report,indent=2))
        return report

def view(speed,folder_review=False,storyboard=False):
    import mujoco.viewer
    scene=Scene(guarded=not (storyboard or folder_review));keys=deque();follow=False;head=None;saved=False
    if folder_review:
        while not any(j.state=='PROCESSING' and j.route[j.index].name=='package' for j in scene.lab.jobs):scene.tick(.1)
        print('Folder review ready at',round(scene.lab.time,1),'demo seconds',flush=True)
    with mujoco.viewer.launch_passive(scene.m,scene.d,key_callback=keys.append) as v:
        print('First-pass viewer ready; 6 = send-out, 9 = finished folders',flush=True)
        c=camera(2.85 if folder_review else -6.5,True);v.cam.lookat[:]=c.lookat;v.cam.distance=2.5 if folder_review else 3.6;v.cam.azimuth=90;v.cam.elevation=-38
        last=time.monotonic()
        while v.is_running():
            now=time.monotonic();dt=min(.12,now-last)*speed;last=now
            with v.lock():
                while keys:
                    k=keys.popleft()
                    if k==32 and not scene.interaction_hold:scene.lab.paused=not scene.lab.paused
                    elif k==88:scene.machines.toggle_xray()
                    elif k==84:follow=not follow;head=None
                    elif k in (86,66,78,77):head={86:'nori_head_left',66:'nori_head_right',78:'nori_head_wide',77:'nori_head_interaction'}[k]
                    elif k==70:scene.lab.inject_scan_fault=True
                    elif k==67:scene.lab.recover()
                    elif k in (61,334):speed=min(16,speed*2)
                    elif k in (45,333):speed=max(.25,speed/2)
                    elif k==82:scene.reset();saved=False
                    elif 49<=k<=57 or k==79:
                        c=camera(scene.layout['stations'][k-49]['x'],True) if 49<=k<=56 else camera()
                        if k==57:c.lookat[:]=[-2.8,-3.79,.93];c.distance=2.7;c.azimuth=-90;c.elevation=-40
                        v.cam.lookat[:]=c.lookat;v.cam.distance=c.distance;v.cam.azimuth=c.azimuth;v.cam.elevation=c.elevation;head=None;follow=False
                if scene.lab.completed<36:scene.tick(dt)
                if head:v.cam.type=mujoco.mjtCamera.mjCAMERA_FIXED;v.cam.fixedcamid=scene.m.camera(head).id
                else:
                    v.cam.type=mujoco.mjtCamera.mjCAMERA_FREE
                    if follow:v.cam.lookat[:]=[scene.rx,0,1];v.cam.distance=2.5
            v.set_texts([(100,mujoco.mjtGridPos.mjGRID_TOPLEFT,scene.status(speed),''),
                         (100,mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,'SPACE pause | R restart | X cutaway | 1-8 station | 9 finished folders | O overview\nV/B/N head left/right/wide | M interaction camera | T follow | +/- speed | F scan fault | C recover','')])
            v.sync();time.sleep(.01)
            if scene.lab.completed==36 and not saved:scene.save();saved=True
    scene.save()

def check(render=False,storyboard=False):
    scene=Scene(guarded=not storyboard);shots=set();previous=None;maximum_jump=0.
    renderer=mujoco.Renderer(scene.m,height=720,width=1280) if render else None
    for step in range(18000):
        scene.tick(.1)
        if step%500==0:print('Workflow',round(scene.lab.time,1),scene.lab.completed,flush=True)
        assert np.isfinite(scene.d.qpos).all() and np.isfinite(scene.d.mocap_pos).all()
        if render:
            modes={r['mode'] for r in scene.machines.rack_trace}
            mode=next((k for k in ('stain','transfer','cv','empty') if k in modes and k not in shots),None)
            if mode:
                c=camera(-2.65,True);c.lookat[:]=[-2.65,.6,1.13];c.distance=2.2;c.elevation=-42
                renderer.update_scene(scene.d,camera=c);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
                draw.rectangle((0,0,1280,74),fill='#10283c');draw.text((18,12),'ExHist first pass | '+mode+' | kinematic CAD playback',fill='white',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',24))
                im.save(ROOT/f'first_pass_{mode}.png');shots.add(mode)
        if scene.lab.completed==36 or scene.interaction_hold:break
    if renderer:renderer.close()
    if scene.interaction_hold:
        print(json.dumps(scene.save('guarded_review_validation.json')['interaction_hold'],indent=2));return
    assert scene.lab.completed==36,'Workflow did not finish'
    report=scene.save('first_pass_validation.json')
    assert scene.max_heading_error<1e-6
    assert all(j.state=='DONE' for j in scene.lab.jobs)
    print(json.dumps(dict(completed_slides=36,seconds=scene.lab.time,transfers=len(scene.custody)//2,machine_phases=len(scene.machines.rack_trace),equipment_facing_error_deg=report['equipment_facing_error_deg'],physical_success=False)))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--render',action='store_true');p.add_argument('--speed',type=float,default=1);p.add_argument('--folder-review',action='store_true')
    p.add_argument('--storyboard',action='store_true',help='Explicitly inspect unvalidated candidate transfers; not physical operation.')
    args=p.parse_args()
    if args.check or args.render:check(args.render,args.storyboard)
    else:view(args.speed,args.folder_review,args.storyboard)
