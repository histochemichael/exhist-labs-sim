"""Joint-space motion trials driven by lab tasks, not physical grasp control."""
import json, math
import numpy as np
import mujoco
from run_lab import ROOT

def solve(m,d,site,names,target,iterations=150):
    ids=[m.joint(n).id for n in names];qs=m.jnt_qposadr[ids];ds=m.jnt_dofadr[ids]
    sid=m.site(site).id;jac=np.zeros((3,m.nv));rot=np.zeros_like(jac)
    for _ in range(iterations):
        mujoco.mj_forward(m,d);err=np.asarray(target)-d.site_xpos[sid]
        if np.linalg.norm(err)<.001:break
        mujoco.mj_jacSite(m,d,jac,rot,sid);a=jac[:,ds]
        change=a.T@np.linalg.solve(a@a.T+np.eye(3)*.0001,err)
        d.qpos[qs]+=np.clip(change,-.08,.08)
        for i,q in zip(ids,qs):
            if m.jnt_limited[i]:d.qpos[q]=np.clip(d.qpos[q],*m.jnt_range[i])
    mujoco.mj_forward(m,d)
    return d.qpos[qs].copy(),float(np.linalg.norm(np.asarray(target)-d.site_xpos[sid]))

class Motion:
    def __init__(self,m,d,lab):
        self.m=m;self.d=d;self.plans={};self.elapsed={};self.prev_time=lab.time
        scratch=mujoco.MjData(m);scratch.qpos[:]=d.qpos
        self.report={"mode":"CAD-joint task-synchronized motion trials; position IK only","robots":{}}
        for name,station,slot in [("sorter_1","sorting",0),("sorter_2","sorting",1),
            ("special_lehisto","special",0),("sendout_lehisto_1","sendout",0),("sendout_lehisto_2","sendout",1)]:
            names=[name+"_carriage"]+[name+"_j"+str(i) for i in range(1,6)]
            qs=[m.joint(n).qposadr[0] for n in names]
            home=scratch.qpos[qs].copy();base=m.body(name).pos
            targets=[base+[-.24,-.15,.24],base+[-.08,-.19,.20],base+[-.04,-.12,.24]]
            poses=[home];errors=[]
            for target in targets:
                q,e=solve(m,scratch,name+"_tcp",names,target);poses.append(q);errors.append(e)
            poses.append(home)
            self.plans[name]=(station,slot,np.asarray(qs),np.array(poses))
            self.elapsed[name]=0.
            self.report["robots"][name]={"target_errors_m":errors,"all_targets_reached":max(errors)<.005}
            scratch.qpos[qs]=home
        # Nori arm poses include an aisle carry pose and a front-buffer reach.
        self.nori_names=["right_"+s+"_joint" for s in ["shoulder_pitch","shoulder_roll","bicep_yaw","elbow_pitch","forearm_yaw","wrist_pitch","wrist_roll"]]
        self.nori_qs=np.array([m.joint(n).qposadr[0] for n in self.nori_names])
        x=lab.robot_x
        scratch.qpos[self.nori_qs]=0
        carry,e2=solve(m,scratch,"nori_right_tcp",self.nori_names,[x+.1,-.8,.98],400)
        rng=np.random.default_rng(41);e=100.;reach=None;best_score=100.;path_depth=1.
        for attempt in range(60):
            scratch.qpos[m.joint("base_y").qposadr[0]]=.95
            if attempt:
                for n in self.nori_names:
                    j=m.joint(n);scratch.qpos[j.qposadr[0]]=rng.uniform(*m.jnt_range[j.id])
            candidate,error=solve(m,scratch,"nori_right_tcp",self.nori_names,[x,.12,1.0],400)
            depth=0.
            for u in np.linspace(0,1,41):
                scratch.qpos[self.nori_qs]=(1-u)*carry+u*candidate
                scratch.qpos[m.joint("base_y").qposadr[0]]=.95*u
                mujoco.mj_forward(m,scratch)
                for c in scratch.contact:
                    a=m.geom(c.geom1).name or "";b=m.geom(c.geom2).name or ""
                    if any(p in a+b for p in ["_top","_leg_","_envelope"]):depth=max(depth,-float(c.dist))
            score=error+10*depth
            if score<best_score:e=error;reach=candidate;best_score=score;path_depth=depth
            if e<.001 and path_depth<.0001:break
        scratch.qpos[m.joint("base_y").qposadr[0]]=0
        scratch.qpos[self.nori_qs]=0
        self.reach=reach;self.carry=carry
        self.report["robots"]["nori"]={"reach_error_m":e,"carry_error_m":e2,"reach_valid":e<.005,
             "dock_base_y_offset_m":.95,"tcp":"provisional parallel-gripper jaw-center point",
             "sampled_bench_path_penetration_m":path_depth,"candidates_tested":attempt+1}
        self.report["limits"]=["Position-only IK, not orientation/grasp validation.",
            "LeHisto targets are local motion-trial poses, not certified rack insertion poses.",
            "Visual LeHisto CAD lacks contact colliders; Nori uses imported coarse colliders."]
        (ROOT/"robot_motion_plan.json").write_text(json.dumps(self.report,indent=2))
    def update(self,lab):
        dt=max(0,lab.time-self.prev_time);self.prev_time=lab.time
        for name,(station,slot,qs,poses) in self.plans.items():
            active=any(j.state=="PROCESSING" and j.holds==station and j.slot==slot for j in lab.jobs)
            if active:self.elapsed[name]+=dt
            # Hold last pose while inactive; no unsignalled robot jumps.
            phase=self.elapsed[name]%8/2;i=min(int(phase),3);u=phase-i;u=u*u*(3-2*u)
            self.d.qpos[qs]=(1-u)*poses[i]+u*poses[i+1]
            jaw=-.012*(.5-.5*math.cos(self.elapsed[name]*math.pi))
            for suffix in ["jaw_left","jaw_right"]:self.d.qpos[self.m.joint(name+"_"+suffix).qposadr[0]]=jaw
            self.d.qpos[self.m.joint(name+"_pinion").qposadr[0]]=jaw/.0072
            carriage=self.d.qpos[self.m.joint(name+"_carriage").qposadr[0]]
            self.d.qpos[self.m.joint(name+"_screw").qposadr[0]]=carriage*2*math.pi/.008
        tr=lab.transport;amount=0.
        if tr:
            t=tr["elapsed"];a=tr["approach"];b=a+2+tr["travel"]
            if a<=t<a+2:amount=math.sin(math.pi*(t-a)/2)**2
            elif b<=t<b+2:amount=math.sin(math.pi*(t-b)/2)**2
        self.d.qpos[self.m.joint("base_y").qposadr[0]]=.95*amount
        self.d.qpos[self.nori_qs]=(1-amount)*self.carry+amount*self.reach
        grip=-.025*(1-amount)
        self.d.qpos[self.m.joint("lehisto_jaw_a_slide").qposadr[0]]=grip
        self.d.qpos[self.m.joint("lehisto_jaw_b_slide").qposadr[0]]=grip
        self.d.qpos[self.m.joint("lehisto_pinion_joint").qposadr[0]]=grip/.0072
        self.d.ctrl[self.m.actuator("lehisto_grip").id]=grip
