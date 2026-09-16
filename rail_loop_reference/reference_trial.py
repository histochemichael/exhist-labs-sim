"""Contact-driven reference trial; commands actuators only after initialization."""
import argparse,json,time
from collections import deque
from pathlib import Path
import numpy as np
import mujoco,placo
from build_scene import ROOT,GRASP,SEAT,JAR
ARM=['rail_travel','base_link_to_link1','link1_to_link2','link2_to_link3','link3_to_link4','link4_to_link5']

class IK:
    def __init__(self):
        self.robot=placo.RobotWrapper(str(ROOT/'source/robot/so101_rail.urdf'))
        self.solver=placo.KinematicsSolver(self.robot);self.solver.mask_fbase(True);self.solver.enable_joint_limits(True);self.solver.dt=.02
        T=np.eye(4);T[:3,:3]=np.array([[1,0,0],[0,0,-1],[0,1,0]]);T[:3,3]=GRASP
        self.task=self.solver.add_frame_task('tcp_link',T);self.task.configure('grasp','soft',10.,1.)
        self.solver.add_regularization_task(1e-5);self.solver.mask_dof('rail_travel')
        self.T=T
    def target(self,xyz,iterations=5):
        self.T[:3,3]=np.asarray(xyz)-np.array([0,0,.0110015]);self.task.T_world_frame=self.T
        for _ in range(iterations):self.robot.update_kinematics();self.solver.solve(True)
        self.robot.update_kinematics()
        return np.array([self.robot.get_joint(n) for n in ARM])

def run(offset=0,close=True,dt=.001,video=False,name='reference',grasp_height_m=.002,rack_offset=(0.,0.),mass_scale=1.,friction_scale=1.,hold_seconds=1.):
    m=mujoco.MjModel.from_xml_path(str(ROOT/'scene.xml'));m.opt.timestep=dt;d=mujoco.MjData(m)
    rack_joint=m.joint('rack_free').qposadr[0]
    d.qpos[rack_joint:rack_joint+2]+=rack_offset
    rack_id=m.body('rack').id
    m.body_mass[rack_id]*=mass_scale;m.body_inertia[rack_id]*=mass_scale
    for gi in range(m.ngeom):
        if (m.geom(gi).name or '').startswith(('groove_', 'rack_')):m.geom_friction[gi,0]*=friction_scale
    ik=IK();target=GRASP+[0,offset,.055];q=ik.target(target,300)
    aids=[m.actuator('rail' if n=='rail_travel' else n).id for n in ARM]
    qadr=[m.joint(n).qposadr[0] for n in ARM]
    d.qpos[qadr]=q;d.qpos[m.joint('leadscrew_rot').qposadr[0]]=q[0]*2*np.pi/.008;d.ctrl[aids]=q
    mujoco.mj_forward(m,d)
    initial_ik_error=float(np.linalg.norm(d.site('sg_handle_tcp').xpos-target))
    if initial_ik_error>.003:raise RuntimeError(f'Initial IK residual {initial_ik_error}')
    grip=m.actuator('sg_grip').id;bid=m.body('rack').id
    jaw_ids={m.body(n).id for n in ('sg_jaw_left','sg_jaw_right')}
    rack_geoms={m.geom(n).id for n in ('rack_bar','rack_neck','rack_lower')}
    jar_geoms={i for i in range(m.ngeom) if m.geom(i).name and m.geom(i).name.startswith('jar_') and m.geom(i).name!='jar_visual'}
    trace=[];close_ready=0.;events=[];max_jaw_rim_force=0.;bilateral=0;held=0;peak=SEAT;max_rim_force=0;bad_penetration=0;phase='SETTLE';phase_t=0;failure=None;baseline=None;peak_lift=0;max_slip=0
    rr=None;writer=None;dual=None
    if video:
        import imageio.v2 as imageio
        from camera_observations import DualCameraRecorder
        dual=DualCameraRecorder(m,ROOT/name)
        rr=mujoco.Renderer(m,720,960);writer=imageio.get_writer(str(ROOT/f'{name}.mp4'),fps=25,codec='libx264',quality=7)
    cam=mujoco.MjvCamera();cam.lookat[:]=[.01,-.15,.08];cam.distance=.6;cam.azimuth=145;cam.elevation=-22
    rim_dwell=0.; contact_window=deque(maxlen=max(1,int(.2/dt))); bias=np.zeros(3); grasp=GRASP.copy(); tick=0;next_control=0;next_log=0;next_frame=0
    try:
        while d.time<40:
            elapsed=d.time-phase_t
            z={'SETTLE':.055,'APPROACH':.055*(1-min(elapsed/2,1)),'CLOSE':0,'PROBE':.005*min(elapsed,1),'LIFT':.005+.105*min(elapsed/3,1),'HOLD':.11,'LOWER':.11*(1-min(elapsed/4,1)),'RELEASE':0,'RETREAT':.055*min(elapsed/2,1)}[phase]
            if d.time>=next_control:
                desired=grasp+[0,offset,z]
                bias=np.clip(bias+.08*(desired-d.site('sg_handle_tcp').xpos),-.010,.010)
                q=ik.target(desired+bias)
                d.ctrl[aids]=q;d.ctrl[grip]=.0355 if close and phase in ('CLOSE','PROBE','LIFT','HOLD','LOWER') else 0
                next_control+=.02
            mujoco.mj_step(m,d)
            if not np.isfinite(d.qpos).all():raise RuntimeError('Nonfinite physics state')
            forces={b:0. for b in jaw_ids};rim=0.;jaw_rim=0.;support=0.
            for ci,c in enumerate(d.contact):
                b1=int(m.geom_bodyid[c.geom1]);b2=int(m.geom_bodyid[c.geom2])
                if (b1 in jaw_ids and c.geom2 in jar_geoms) or (b2 in jaw_ids and c.geom1 in jar_geoms):
                    jf=np.zeros(6);mujoco.mj_contactForce(m,d,ci,jf);jaw_rim+=max(0,float(jf[0]))
                if c.geom1 not in rack_geoms and c.geom2 not in rack_geoms:continue
                other=c.geom2 if c.geom1 in rack_geoms else c.geom1
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);force=max(0,float(f[0]))
                body=int(m.geom_bodyid[other])
                if body in forces:forces[body]+=force
                if other in jar_geoms and m.geom(other).name!='jar_floor':rim+=force
                if (m.geom(other).name or '').startswith('riser_support'):support+=force
                bad_penetration=max(bad_penetration,max(0,-float(c.dist)))
            max_jaw_rim_force=max(max_jaw_rim_force,jaw_rim)
            both=min(forces.values())>.02;contact_window.append(both)
            jaw_speed=max(abs(float(d.joint(n).qvel[0])) for n in ('sg_jaw_left_joint','sg_jaw_right_joint'))
            ready=both and jaw_speed<.001 and jaw_rim<.5
            close_ready=close_ready+dt if phase=='CLOSE' and ready else 0.
            bilateral=bilateral+dt if both else 0
            rack_z=float(d.xpos[bid,2]);peak=max(peak,rack_z);max_rim_force=max(max_rim_force,rim)
            if phase in ('LIFT','HOLD') and baseline is not None:
                max_slip=max(max_slip,float(np.linalg.norm((d.xpos[bid]-d.site('sg_handle_tcp').xpos)-baseline)))
            rim_dwell=rim_dwell+dt if max(rim,jaw_rim)>15 else 0.
            if rim_dwell>.02:failure='sustained_rim_contact';break
            if phase in ('LIFT','HOLD') and max_slip>.005:failure='grasp_slipped';break
            if d.time>=next_log:
                trace.append(dict(t=float(d.time),phase=phase,rack_xyz=d.xpos[bid].tolist(),tcp=d.site('sg_handle_tcp').xpos.tolist(),jaw_force_N=list(forces.values()),rim_force_N=rim,support_force_N=support,ctrl=d.ctrl.tolist(),qpos=d.qpos.tolist(),qvel=d.qvel.tolist(),jaw_rim_force_N=jaw_rim,actuator_force=d.actuator_force.tolist(),grasp_ready_s=close_ready))
                next_log+=.04
            if video and d.time>=next_frame:
                rr.update_scene(d,cam);writer.append_data(rr.render());dual.append(d,phase);next_frame+=.04
            duration={'SETTLE':1,'APPROACH':2.8,'CLOSE':8,'PROBE':1.5,'LIFT':3.5,'HOLD':hold_seconds,'LOWER':4.8,'RELEASE':1.5,'RETREAT':2.5}[phase]
            if phase=='CLOSE' and elapsed>duration and close_ready<.3:failure='grasp_not_settled';break
            if phase=='PROBE' and elapsed>duration and (np.mean(contact_window)<.95 or rack_z-SEAT<.002):failure='failed_grasp_probe';break
            if phase=='LIFT' and elapsed>duration and rack_z-SEAT<.085:failure='failed_to_clear_rim';break
            if phase=='LOWER' and elapsed>duration and (abs(rack_z-SEAT)>.003 or support<.1):failure='not_supported_on_riser';break
            if phase=='RETREAT' and elapsed>duration:break
            if elapsed>duration or (phase=='CLOSE' and elapsed>=2 and close_ready>=.3):
                if phase=='SETTLE':grasp=d.site('rack_grasp').xpos.copy()+[0,0,grasp_height_m]
                if phase=='CLOSE':baseline=d.xpos[bid].copy()-d.site('sg_handle_tcp').xpos
                events.append(dict(phase=phase,end_time_s=float(d.time),duration_s=elapsed))
                phases=['SETTLE','APPROACH','CLOSE','PROBE','LIFT','HOLD','LOWER','RELEASE','RETREAT'];phase=phases[phases.index(phase)+1];phase_t=float(d.time)
            tick+=1
        seated=bool(abs(float(d.xpos[bid,2])-SEAT)<.003)
        stable=float(np.max(np.abs(d.qvel[m.joint('rack_free').dofadr[0]:m.joint('rack_free').dofadr[0]+6])))<.02
        passed=bool(failure is None and phase=='RETREAT' and peak-SEAT>.085 and seated and stable and support>.1 and np.linalg.norm(d.xpos[bid,:2]-(JAR[:2]+rack_offset))<.003 and d.xmat[bid].reshape(3,3)[2,2]>.995 and d.ctrl[grip]==0 and np.linalg.norm(d.site('sg_handle_tcp').xpos-d.site('rack_grasp').xpos)>.04)
        result=dict(passed=passed,failure=failure,phase=phase,peak_lift_m=peak-SEAT,final_rack_xyz=d.xpos[bid].tolist(),seated=seated,stable=stable,max_rack_penetration_m=bad_penetration,max_rim_force_N=max_rim_force,max_slip_m=max_slip,initial_ik_error_m=initial_ik_error,dt=dt,offset_m=offset,close=close,scope='Reference actuator trajectory, not a learned policy. Fixed slide load; provisional physics.')
        hold=[r for r in trace if r['phase']=='HOLD'][-12:]
        final=trace[-12:]
        result['final_rack_peak_to_peak_mm']=(np.ptp([r['rack_xyz'] for r in final],axis=0)*1000).tolist()
        result['final_rack_velocity']=d.qvel[m.joint('rack_free').dofadr[0]:m.joint('rack_free').dofadr[0]+6].tolist()
        result['final_support_force_N']=support
        result.update(handle_width_m=float(m.geom('rack_bar').size[1]*2),multiccd=bool(m.opt.enableflags & int(mujoco.mjtEnableBit.mjENBL_MULTICCD)),grasp_height_m=grasp_height_m,mass_scale=mass_scale,friction_scale=friction_scale,rack_offset_m=list(rack_offset),max_jaw_rim_force_N=max_jaw_rim_force,phase_events=events)
        if hold:
            result['hold_tcp_peak_to_peak_mm']=(np.ptp([r['tcp'] for r in hold],axis=0)*1000).tolist()
            result['hold_rack_peak_to_peak_mm']=(np.ptp([r['rack_xyz'] for r in hold],axis=0)*1000).tolist()
        (ROOT/f'{name}.json').write_text(json.dumps(result,indent=2));(ROOT/f'{name}_trace.json').write_text(json.dumps(trace))
        if video:
            rr.update_scene(d,cam)
            from PIL import Image
            Image.fromarray(rr.render()).save(ROOT/f'{name}.png')
        print(json.dumps(result,indent=2));return result
    finally:
        if dual:dual.close()
        if writer:writer.close()
        if rr:rr.close()
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--video',action='store_true');ap.add_argument('--offset-mm',type=float,default=0);ap.add_argument('--no-close',action='store_true');ap.add_argument('--dt',type=float,default=.001);ap.add_argument('--name',default='reference');a=ap.parse_args()
    run(a.offset_mm/1000,not a.no_close,a.dt,a.video,a.name)
