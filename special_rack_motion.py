"""Reach-checked five-bath rack choreography; contact physics remains separate."""
import json,math
import numpy as np
import mujoco
from build_scene import ROOT
from jar_riser import RACK_SEAT_Z
from lehisto_loaded_reach import profile,row_position,audit_scene

ACTIVE_JARS=tuple(profile()['layout']['active_jars'])
RACK_Z=RACK_SEAT_Z
def bath_point(index):return np.array([*row_position(index),RACK_Z])

class SpecialRackMotion:
    def __init__(self,scene):
        from first_pass import ik,GRASP_ROTATION,yaw_matrix
        self.s=scene;self.poses={};self.errors=[]
        self.loaded_reach_audit=audit_scene(scene.m,scene.d)
        self.names=['special_lehisto_carriage']+['special_lehisto_j'+str(i) for i in range(1,6)]
        self.qa=np.array([scene.q[n] for n in self.names])
        a=math.pi/6;rx=np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
        self.rotation=rx@yaw_matrix(math.pi)@GRASP_ROTATION
        scratch=mujoco.MjData(scene.m);scratch.qpos[:]=scene.d.qpos
        for jar in ACTIVE_JARS:
            poses=[]
            for height in np.linspace(0,.11,21):
                q,pe,re=ik(scene.m,scratch,'special_lehisto_handle_groove',self.names,bath_point(jar)+[0,0,.0925+height],self.rotation,attempts=10)
                self.errors.append([jar,float(height),pe,re])
                if pe>.001 or re>.01:raise RuntimeError('Special rack pose outside tolerance: '+str(self.errors[-1]))
                poses.append(q)
            self.poses[jar]=np.array(poses)
        (ROOT/'special_rack_reach.json').write_text(json.dumps(dict(active_jars=ACTIVE_JARS,spare_jars=[1,2,3,9,10,11],poses=self.errors,rail_limits_m=[-.06,.21],approach_tilt_deg=30,scope='CAD-joint reach only; jar contacts and loaded-slide retention unvalidated.'),indent=2))

    def update(self):
        from first_pass import smooth,kinematics,carrier_rotation
        s=self.s
        job=next((j for j in s.lab.jobs if j.holds=='special' and j.state=='PROCESSING' and j.route[j.index].name=='special_stain'),None)
        if job is None:return
        elapsed=job.route[job.index].seconds-job.remaining
        def vertical(jar,u):
            f=np.clip(u,0,1)*20;i=min(19,int(f));return (1-f+i)*self.poses[jar][i]+(f-i)*self.poses[jar][i+1]
        if elapsed<2:
            rest=s.robot_plans['special_lehisto'][3][0]
            s.d.qpos[self.qa]=(1-smooth(elapsed/2))*rest+smooth(elapsed/2)*self.poses[4][0]
            s.put(job,bath_point(4));closed=False
        elif elapsed<26:
            segment=min(3,int((elapsed-2)/6));u=(elapsed-2-segment*6)/6;jar=ACTIVE_JARS[segment];nxt=ACTIVE_JARS[segment+1]
            if u<1/3:q=vertical(jar,smooth(u*3))
            elif u<2/3:q=(1-smooth(u*3-1))*self.poses[jar][-1]+smooth(u*3-1)*self.poses[nxt][-1]
            else:q=vertical(nxt,1-smooth(u*3-2))
            s.d.qpos[self.qa]=q;closed=True
        else:
            s.d.qpos[self.qa]=self.poses[8][0];s.put(job,bath_point(8));closed=False
        for side in ('left','right'):s.d.qpos[s.q['special_lehisto_jaw_'+side]]=-.0355 if closed else -.03
        pinion='special_lehisto_pinion'
        s.d.qpos[s.q[pinion]]=np.clip((-.0355 if closed else -.03)/.0072,*s.m.jnt_range[s.m.joint(pinion).id])
        s.d.qpos[s.q['special_lehisto_screw']]=s.d.qpos[s.q['special_lehisto_carriage']]*2*math.pi/.008
        kinematics(s.m,s.d)
        if closed:
            sid=s.m.site('special_lehisto_handle_groove').id;r=s.d.site_xmat[sid].reshape(3,3)@self.rotation.T
            quat=np.zeros(4);mujoco.mju_mat2Quat(quat,(r@carrier_rotation('rack24')).ravel())
            s.put(job,s.d.site_xpos[sid]-r@np.array([0,0,.0925]),quat)
