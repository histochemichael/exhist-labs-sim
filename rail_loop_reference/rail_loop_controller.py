"""Continuous contact-driven rack transfers; no payload resets between containers."""
import argparse,json
from collections import deque
import numpy as np
import mujoco
from build_scene import ROOT,SEAT
from reference_trial import IK,ARM
from multi_container_scene import ORIGIN,rz

class YawIK(IK):
    def target_yaw(self,xyz,yaw,iterations=5):
        self.T[:3,:3]=rz(yaw)@np.array([[1,0,0],[0,0,-1],[0,1,0]])
        return self.target(xyz,iterations)

def smooth(x):
    u=np.clip(x,0.,1.);return u*u*(3-2*u)


def run(hops=8,video=False,name='multi_transfer',dt=.001,scene_path=None,layout_path=None,route_ids=None,initial_fk_correction=False):
    layout=json.loads((layout_path or ROOT/'multi_container_layout.json').read_text());stations=layout['stations']
    by_id={s['id']:s for s in stations}
    route=[by_id[i] for i in route_ids] if route_ids is not None else stations
    if not 1<=hops<len(route):raise ValueError('hops must fit the selected route')
    m=mujoco.MjModel.from_xml_path(str(scene_path or ROOT/'multi_container.xml'));m.opt.timestep=dt;d=mujoco.MjData(m)
    aids=[m.actuator('rail' if n=='rail_travel' else n).id for n in ARM]
    qadr=[m.joint(n).qposadr[0] for n in ARM];grip=m.actuator('sg_grip').id
    ik=RailIK(route[0]['rail_m']);initial=np.array([*route[0]['position_m'][:2],SEAT+.0925+.055])
    q=ik.target_yaw(initial,route[0]['angle_deg'],300);d.qpos[qadr]=q
    d.qpos[m.joint('leadscrew_rot').qposadr[0]]=q[0]*2*np.pi/.008;d.ctrl[aids]=q
    mujoco.mj_forward(m,d)
    if initial_fk_correction:
        correction=np.zeros(3)
        for _ in range(3):
            correction=np.clip(correction+initial-d.site('sg_handle_tcp').xpos,-.010,.010)
            q=ik.target_yaw(initial+correction,route[0]['angle_deg'],100)
            d.qpos[qadr]=q;d.ctrl[aids]=q;mujoco.mj_forward(m,d)
    if np.linalg.norm(d.site('sg_handle_tcp').xpos-initial)>.003:raise RuntimeError('Initial IK residual too large')
    jaws={m.body(n).id for n in ('sg_jaw_left','sg_jaw_right')}
    rack=m.body('rack').id;rack_geoms={m.geom(n).id for n in ['rack_bar','rack_neck','rack_lower']}
    jar_geoms={};supports={}
    for station in stations:
        jar_id=m.body(station['jar_body']).id
        riser_id=m.body(station['riser_body']).id
        for gi in range(m.ngeom):
            if m.geom_bodyid[gi]==jar_id and m.geom_contype[gi]:jar_geoms[gi]=station['id']
            if m.geom_bodyid[gi]==riser_id and m.geom_contype[gi]:supports[gi]=station['id']
    robot_bodies={m.body(n).id for n in ('slide_gripper_base','sg_jaw_left','sg_jaw_right')}
    for bi in range(m.nbody):
        if m.body(bi).name.startswith(('link','base_link')):robot_bodies.add(bi)
    phase='SETTLE';phase_t=0.;hop=0;failure=None;trace=[];completed=[];events=[]
    next_control=0.;next_log=0.;next_frame=0.;bias=np.zeros(3);ready_s=0.;rim_dwell=0.;contact_window=deque(maxlen=max(1,int(.2/dt)))
    source_grasp=initial-[0,0,.055];dest_grasp=source_grasp.copy();approach_start=initial.copy();baseline=None
    minimum_rail_clearance=float('inf');max_rail_contact=0.
    max_slip=0.;max_rim=0.;max_jaw_rim=0.;max_arm_rim=0.;max_penetration=0.;transfer_min_clearance=float('inf');peak=SEAT
    rr=writer=dual=None
    cam=mujoco.MjvCamera();cam.lookat[:]=[0,.035,.05];cam.distance=1.0;cam.azimuth=135;cam.elevation=-43
    if video:
        import imageio.v2 as imageio
        from camera_observations import DualCameraRecorder
        dual=DualCameraRecorder(m,ROOT/name,fps=15);rr=mujoco.Renderer(m,720,960)
        writer=imageio.get_writer(str(ROOT/f'{name}.mp4'),fps=15,codec='libx264',quality=7)
    try:
        while d.time<35*hops+5:
            source=route[hop];dest=route[hop+1];elapsed=d.time-phase_t
            transfer_s=max(4.,abs(dest['angle_deg']-source['angle_deg'])/8.,1.5*abs(dest['rail_m']-source['rail_m'])/.02)
            durations={'SETTLE':1.,'APPROACH':3.,'CLOSE':8.,'PROBE':1.5,'LIFT':3.5,'TRANSFER':transfer_s,'HOVER':1.,'LOWER':4.8,'RELEASE':1.5,'RETREAT':2.5,'VERIFY':.8}
            yaw=source['angle_deg'];desired=source_grasp.copy();planned_rail=source['rail_m']
            if phase=='SETTLE':desired=initial
            elif phase=='APPROACH':desired=approach_start+(source_grasp-approach_start)*smooth(elapsed/2.4)
            elif phase=='PROBE':desired=source_grasp+[0,0,.005*smooth(elapsed/1.)]
            elif phase=='LIFT':desired=source_grasp+[0,0,.005+.105*smooth(elapsed/3.)]
            elif phase=='TRANSFER':
                u=smooth(elapsed/transfer_s)
                desired,yaw,planned_rail=trajectory(source,dest,source_grasp,dest_grasp,u)
            elif phase in ('HOVER','LOWER','RELEASE','RETREAT','VERIFY'):
                yaw=dest['angle_deg'];desired=dest_grasp.copy();planned_rail=dest['rail_m']
                if phase=='HOVER':desired[2]+=.11
                elif phase=='LOWER':desired[2]+=.11*(1-smooth(elapsed/4.))
                elif phase=='RETREAT':desired[2]+=.055*smooth(elapsed/2.)
                elif phase=='VERIFY':desired[2]+=.055
            if d.time>=next_control:
                bias=np.clip(bias+.08*(desired-d.site('sg_handle_tcp').xpos),-.010,.010)
                ik.rail=planned_rail
                d.ctrl[aids]=ik.target_yaw(desired+bias,yaw)
                grip_target=.0355 if phase in ('CLOSE','PROBE','LIFT','TRANSFER','HOVER','LOWER') else 0.
                d.ctrl[grip]+=float(np.clip(grip_target-d.ctrl[grip],-.0006,.0006))
                next_control+=.02
            mujoco.mj_step(m,d)
            if not np.isfinite(d.qpos).all():raise RuntimeError('Nonfinite physics state')
            forces={j:0. for j in jaws};support={s['id']:0. for s in stations};rim=jaw_rim=arm_rim=0.
            for ci,c in enumerate(d.contact):
                g1,g2=int(c.geom1),int(c.geom2);b1,b2=int(m.geom_bodyid[g1]),int(m.geom_bodyid[g2])
                relevant=g1 in rack_geoms or g2 in rack_geoms or ((b1 in robot_bodies and g2 in jar_geoms) or (b2 in robot_bodies and g1 in jar_geoms))
                if not relevant:continue
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);force=max(0.,float(f[0]))
                if (b1 in robot_bodies and g2 in jar_geoms) or (b2 in robot_bodies and g1 in jar_geoms):arm_rim+=force
                if (b1 in jaws and g2 in jar_geoms) or (b2 in jaws and g1 in jar_geoms):jaw_rim+=force
                if g1 in rack_geoms or g2 in rack_geoms:
                    other=g2 if g1 in rack_geoms else g1;body=int(m.geom_bodyid[other])
                    if body in jaws:forces[body]+=force
                    if other in supports:support[supports[other]]+=force
                    if other in jar_geoms:rim+=force
                    max_penetration=max(max_penetration,max(0.,-float(c.dist)))
            rail_force=0.
            guard=m.geom('rail_rack_guard').id
            for ci,c in enumerate(d.contact):
                if guard in (c.geom1,c.geom2):
                    f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);rail_force+=max(0.,float(f[0]))
            max_rail_contact=max(max_rail_contact,rail_force)
            if rail_force>.5:failure='rack_rail_collision';break
            both=min(forces.values())>.02;contact_window.append(both)
            speed=max(abs(float(d.joint(n).qvel[0])) for n in ('sg_jaw_left_joint','sg_jaw_right_joint'))
            ready_s=ready_s+dt if phase=='CLOSE' and both and speed<.001 and jaw_rim<.5 else 0.
            max_arm_rim=max(max_arm_rim,arm_rim);max_rim=max(max_rim,rim);max_jaw_rim=max(max_jaw_rim,jaw_rim)
            rim_dwell=rim_dwell+dt if max(rim,jaw_rim,arm_rim)>15 else 0.
            if rim_dwell>.02:failure='sustained_container_contact';break
            local=d.body('slide_gripper_base').xmat.reshape(3,3).T@(d.body('rack').xpos-d.body('slide_gripper_base').xpos)
            slip=0. if baseline is None else float(np.linalg.norm(local-baseline))
            if phase in ('LIFT','TRANSFER','HOVER'):
                max_slip=max(max_slip,slip)
                if slip>.005:failure='grasp_slipped';break
            bottom=float(d.geom('rack_lower').xpos[2]-np.abs(d.geom('rack_lower').xmat.reshape(3,3)[2])@m.geom('rack_lower').size)
            rim_top=max(d.body(s['jar_body']).xpos[2]+.0918 for s in stations)
            if phase=='TRANSFER':
                size=np.abs(d.geom('rack_lower').xmat.reshape(3,3))@m.geom('rack_lower').size
                xyz=d.geom('rack_lower').xpos;lo,hi=np.array(RAIL_BOUNDS)
                if np.all(xyz[:2]+size[:2]>lo[:2]) and np.all(xyz[:2]-size[:2]<hi[:2]):
                    clearance=bottom-hi[2];minimum_rail_clearance=min(minimum_rail_clearance,clearance)
                    if clearance<.015:failure='rail_overflight_clearance';break
                transfer_min_clearance=min(transfer_min_clearance,bottom-rim_top)
                if bottom-rim_top<.015 or rim>.5 or jaw_rim>.5 or arm_rim>.5:failure='transfer_clearance';break
            peak=max(peak,float(d.body('rack').xpos[2]))
            if d.time>=next_log:
                trace.append(dict(t=float(d.time),phase=phase,source=source['id'],destination=dest['id'],rack_xyz=d.body('rack').xpos.tolist(),tcp=d.site('sg_handle_tcp').xpos.tolist(),planned_rail_m=planned_rail,rail_contact_N=rail_force,qpos=d.qpos.tolist(),qvel=d.qvel.tolist(),ctrl=d.ctrl.tolist(),support_N=support,jaw_force_N=list(forces.values()),rack_container_force_N=rim,jaw_container_force_N=jaw_rim,arm_container_force_N=arm_rim,slip_m=slip,bottom_clearance_m=bottom-rim_top,actuator_force=d.actuator_force.tolist()))
                next_log+=.04
            if video and d.time>=next_frame:
                from PIL import Image,ImageDraw
                rr.update_scene(d,cam);frame=Image.fromarray(rr.render());draw=ImageDraw.Draw(frame)
                draw.rectangle((0,0,960,28),fill='black');draw.text((8,8),f'SIM ONLY | Container {source["id"]} -> {dest["id"]} | {phase} | completed {len(completed)}/{hops}',fill='white')
                writer.append_data(np.asarray(frame));dual.append(d,f'{source["id"]}_to_{dest["id"]}_{phase}');next_frame+=1/15
            finish=elapsed>durations[phase]
            if phase=='CLOSE':
                if finish and ready_s<.3:failure='grasp_not_settled';break
                finish=finish or (elapsed>=2 and ready_s>=.3)
            if phase=='PROBE' and finish and (np.mean(contact_window)<.95 or d.body('rack').xpos[2]-SEAT<.002):failure='failed_grasp_probe';break
            if phase=='LIFT' and finish and bottom-rim_top<.015:failure='failed_to_clear_rims';break
            expected_z=float(d.body(dest['riser_body']).xpos[2]+.0254)
            if phase=='LOWER' and finish and (abs(d.body('rack').xpos[2]-expected_z)>.003 or support[dest['id']]<.1):failure='not_supported_in_destination';break
            if phase=='VERIFY' and finish:
                velocity=d.joint('rack_free').qvel
                jar=d.body(dest['jar_body']);relative=jar.xmat.reshape(3,3).T@(d.body('rack').xpos-jar.xpos)
                ok=abs(d.body('rack').xpos[2]-expected_z)<.003 and np.linalg.norm(relative[:2])<.003 and np.max(np.abs(velocity))<.02 and support[dest['id']]>.1 and d.body('rack').xmat.reshape(3,3)[2,2]>.995 and d.ctrl[grip]==0 and np.linalg.norm(d.site('sg_handle_tcp').xpos-d.site('rack_grasp').xpos)>.04
                if not ok:failure='destination_verification_failed';break
                completed.append(dict(source=source['id'],destination=dest['id'],time_s=float(d.time),rack_xyz=d.body('rack').xpos.tolist(),support_force_N=support[dest['id']],lateral_error_m=float(np.linalg.norm(relative[:2]))))
                print(f'Completed {source["id"]} -> {dest["id"]}',flush=True)
                if len(completed)==hops:break
                hop+=1;phase='APPROACH';phase_t=float(d.time);baseline=None;peak=SEAT
                source=route[hop];dest=route[hop+1]
                source_grasp=d.site('rack_grasp').xpos.copy()+[0,0,.002];approach_start=d.site('sg_handle_tcp').xpos.copy()
                dest_grasp=np.array([*d.body(dest['jar_body']).xpos[:2],d.body(dest['riser_body']).xpos[2]+.0254+.0925+.002])
                continue
            if finish:
                if phase=='SETTLE':
                    source_grasp=d.site('rack_grasp').xpos.copy()+[0,0,.002];approach_start=d.site('sg_handle_tcp').xpos.copy()
                    dest_grasp=np.array([*d.body(dest['jar_body']).xpos[:2],d.body(dest['riser_body']).xpos[2]+.0254+.0925+.002])
                if phase=='CLOSE':baseline=local.copy()
                events.append(dict(phase=phase,source=source['id'],destination=dest['id'],end_time_s=float(d.time)))
                phases=['SETTLE','APPROACH','CLOSE','PROBE','LIFT','TRANSFER','HOVER','LOWER','RELEASE','RETREAT','VERIFY']
                phase=phases[phases.index(phase)+1];phase_t=float(d.time)
        passed=failure is None and len(completed)==hops
        result=dict(minimum_rail_clearance_m=None if not np.isfinite(minimum_rail_clearance) else minimum_rail_clearance,max_rail_contact_N=max_rail_contact,passed=passed,failure=failure,phase=phase,requested_hops=hops,completed_transfers=completed,max_slip_m=max_slip,max_rack_container_force_N=max_rim,max_jaw_container_force_N=max_jaw_rim,max_arm_container_force_N=max_arm_rim,max_penetration_m=max_penetration,minimum_transfer_clearance_m=None if not np.isfinite(transfer_min_clearance) else transfer_min_clearance,dt=dt,events=events,simulated_seconds=float(d.time),scope='Offline continuous actuator reference; no payload attachments or per-hop resets; not a learned policy.')
        (ROOT/f'{name}.json').write_text(json.dumps(result,indent=2));(ROOT/f'{name}_trace.json').write_text(json.dumps(trace))
        if video:
            from PIL import Image
            rr.update_scene(d,cam);Image.fromarray(rr.render()).save(ROOT/f'{name}.png')
        print(json.dumps(result,indent=2));return result
    finally:
        if dual:dual.close()
        if writer:writer.close()
        if rr:rr.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--hops',type=int,default=8);parser.add_argument('--video',action='store_true');parser.add_argument('--name',default='multi_transfer');parser.add_argument('--dt',type=float,default=.001)
    a=parser.parse_args();result=run(a.hops,a.video,a.name,a.dt)
    raise SystemExit(0 if result["passed"] else 1)