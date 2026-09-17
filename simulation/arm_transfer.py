"""Development of actual Nori-arm contact transfer. Not full-lab readiness."""
import json,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import mobile_base_bench as mobile
from build_scene import ROOT,el,vec
from pose_control import multistart,solve_pose,rotation_error

JOINTS=['right_'+s+'_joint' for s in ('shoulder_pitch','shoulder_roll','bicep_yaw','elbow_pitch','forearm_yaw','wrist_pitch','wrist_roll')]
SITE='nori_right_slide_groove'
ROTATION=np.array([[1,0,0],[0,0,-1],[0,1,0]],float)
PICK=np.array([-.20,-.48,.8665]);PLACE=PICK+np.array([.10,0,0])

def geometry(front=False):
    if not front:return PICK.copy(),PLACE.copy(),ROTATION.copy()
    rz=np.array([[0,-1,0],[1,0,0],[0,0,1]],float)
    return np.array([.20,-.20,.8665]),np.array([.20,-.10,.8665]),rz@ROTATION

def build(timestep=.001,grip_force=.8,slide_offset=0,front=False):
    PICK,PLACE,ROTATION=geometry(front)
    root=mobile.build();root.set('model','Nori actual-arm slide transfer DEVELOPMENT')
    root.find('option').set('timestep',str(timestep))
    root.find('option').set('noslip_iterations','20')
    world=root.find('worldbody')
    el(world,'geom',name='transfer_table',type='box',pos='.29 0 .78' if front else '-.15 -.48 .78',size='.11 .23 .02' if front else '.22 .12 .02',rgba='.55 .67 .7 1')
    # Free microscope slide. Initial support is a table, not a parent/weld.
    p=PICK-[0,0,.029]+[slide_offset,0,0]
    payload=el(world,'body',name='transfer_slide',pos=vec(p));el(payload,'freejoint',name='transfer_slide_free')
    if front:payload.set('quat','.707106781 0 0 .707106781')
    el(payload,'geom',name='transfer_slide_contact',type='box',size='.0125 .0005 .0375',mass='.0046875',rgba='.5 .85 .95 1',condim='4',friction='.6 .005 .0001',solref='.004 1',solimp='.95 .99 .001')
    # Open guide slots support the vertical slide against toppling before pickup.
    for name,point in [('source',PICK),('destination',PLACE)]:
        for side in (-1,1):
            guide=[point[0]+side*.0017,point[1],.810] if front else [point[0],point[1]+side*.0017,.810]
            el(world,'geom',name=f'{name}_guide_{side}',type='box',pos=vec(guide),size='.001 .015 .010' if front else '.015 .001 .010',rgba='.35 .45 .5 1',solref='.004 1')
            if front:
                # Provisional locating slot: 25.6 x 1.4 mm clear, 20 mm high.
                # End stops resist sideways release forces; no payload constraints.
                el(world,'geom',name=f'{name}_endstop_{side}',type='box',pos=vec([point[0],point[1]+side*.0133,.810]),size='.0027 .0005 .010',rgba='.35 .45 .5 1',solref='.004 1')
    # Hold the lift only as in the isolated mobility model. No new arm or payload constraints.
    for actuator in root.find('actuator'):
        if actuator.get('name','').startswith('hold_right_'):actuator.set('kp','250');actuator.set('kv','12')
    actuator=root.find("actuator/position[@name='lehisto_grip']")
    actuator.set('forcerange',vec([-grip_force,grip_force]) if grip_force else '-1e-12 1e-12')
    for equality in root.find('equality'):
        if equality.get('name','').startswith('lehisto_'):equality.set('solref','.002 1');equality.set('solimp','.99 .999 .0001')
    if front:
        from front_docking import apply_provisional_lift,add_head_cameras
        apply_provisional_lift(root);add_head_cameras(root)
    return root

def plan(model,data,front=False):
    PICK,PLACE,ROTATION=geometry(front)
    qa=np.array([model.joint(n).qposadr[0] for n in JOINTS]);seed=data.qpos.copy()
    targets=[PICK+[0,0,.04],PICK,PICK+[0,0,.04],PLACE+[0,0,.04],PLACE,PLACE+[0,0,.04]]
    rng=np.random.default_rng(26);scratch=mujoco.MjData(model);best=None;score=float('inf')
    for attempt in range(40):
        seed=data.qpos.copy();seed[qa]=[rng.uniform(*model.jnt_range[model.joint(n).id]) for n in JOINTS]
        seed[qa]=solve_pose(model,seed,SITE,JOINTS,PICK,ROTATION)[0]
        poses=[];errors=[];depth=0.
        for target in targets:
            q,pe,re=solve_pose(model,seed,SITE,JOINTS,target,ROTATION)
            previous=poses[-1] if poses else q
            for u in np.linspace(0,1,11):
                scratch.qpos[:]=seed;scratch.qpos[qa]=previous*(1-u)+q*u;mujoco.mj_forward(model,scratch)
                for contact in scratch.contact:
                    if 'transfer_table' in [model.geom(contact.geom1).name,model.geom(contact.geom2).name] and model.geom('transfer_slide_contact').id not in (contact.geom1,contact.geom2):depth=max(depth,-float(contact.dist))
            poses.append(q);errors.append(dict(position_m=pe,angle_rad=re,sampled_table_penetration_m=depth));seed[qa]=q
        candidate=max(e['position_m']+.1*e['angle_rad'] for e in errors)+10*depth
        if candidate<score:best=(poses,errors);score=candidate
        if all(e['position_m']<.0003 and e['angle_rad']<.003 for e in errors) and depth<.0001:break
    return best

def run(render=False,timestep=.001,grip_force=.8,slide_offset=0,save=True,front=False):
    PICK,PLACE,ROTATION=geometry(front)
    root=build(timestep,grip_force,slide_offset,front);m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d,drive=mobile.initialize(m)
    poses,errors=plan(m,d,front)
    if any(e['position_m']>.001 or e['angle_rad']>.01 or e['sampled_table_penetration_m']>.0001 for e in errors):return dict(passed=False,phase='IK_REJECTED',errors=errors)
    qa=np.array([m.joint(n).qposadr[0] for n in JOINTS]);d.qpos[qa]=poses[0];mujoco.mj_forward(m,d);drive.home=d.qpos.copy()
    ai=np.array([m.actuator('hold_'+n).id for n in JOINTS]);va=np.array([m.joint(n).dofadr[0] for n in JOINTS]);qtarget=poses[0].copy()
    sid=m.site(SITE).id;bid=m.body('transfer_slide').id;pg=m.geom('transfer_slide_contact').id;grip=m.actuator('lehisto_grip').id
    jaws={g:('a' if m.geom(g).name.startswith('nori_groove_a_') else 'b') for g in range(m.ngeom) if m.geom(g).name.startswith('nori_groove_')}
    phase='SETTLE';phase_start=0.;events=[];trace=[];dwell=0.;grasp_relative=None;slip=0.;peak_lift=0.;max_torque=0.;fault_contacts=[]
    target=PICK+[0,0,.04];origin=target.copy();goal=target.copy();duration=2.;support_dwell=0.;lost_dwell=0.;holding_samples=0;bilateral_samples=0
    camera_rows=[];camera_images=[];camera_renderer=mujoco.Renderer(m,height=240,width=320) if front else None
    lift_peak=0.;heading_peak=0.;dock_dwell=0.;release_q=0.
    def transition(next_phase,new_goal,seconds):
        nonlocal phase,phase_start,origin,goal,duration,dwell,release_q
        events.append(dict(t=d.time,phase=next_phase));phase=next_phase;phase_start=d.time
        origin=target.copy();goal=np.array(new_goal);duration=seconds;dwell=0.
        if next_phase=='RELEASE':release_q=float(d.qpos[m.joint('lehisto_jaw_a_slide').qposadr[0]])
        if front and next_phase in ('APPROACH','LIFT','CARRY','RELEASE','COMPLETE'):
            from PIL import Image,ImageDraw
            row=[];counts={};finger_counts={}
            for name in ['nori_head_left','nori_head_right','nori_head_wide']:
                camera_renderer.enable_segmentation_rendering();camera_renderer.update_scene(d,camera=name)
                labels=camera_renderer.render();pixels=int(np.count_nonzero((labels[:,:,0]==pg)&(labels[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))))
                finger_counts[name]=[int(np.count_nonzero((labels[:,:,0]==m.geom(f'lehisto_part_{part}').id)&(labels[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM)))) for part in (14,15)]
                counts[name]=pixels;camera_renderer.disable_segmentation_rendering();camera_renderer.update_scene(d,camera=name)
                frame=Image.fromarray(camera_renderer.render());draw=ImageDraw.Draw(frame);draw.rectangle([0,0,320,32],fill='#10283c');draw.text((5,5),next_phase+' / '+name+' / '+str(pixels)+' px',fill='white');row.append(frame)
            camera_images.append(row);camera_rows.append(dict(t=d.time,phase=next_phase,slide_pixels=counts,finger_pixels=finger_counts,visible_any=max(counts.values())>=10,
                interaction_visible_any=any(counts[n]>=10 and min(finger_counts[n])>=8 for n in counts)))
    for i in range(round(55/timestep)):
        t=d.time;elapsed=t-phase_start;u=min(1,elapsed/duration);u=u*u*(3-2*u);target=origin+(goal-origin)*u
        if i%max(1,round(.02/timestep))==0:
            solution,pe,re=solve_pose(m,d.qpos,SITE,JOINTS,target,ROTATION,iterations=70)
            if pe>.003 or re>.03:
                transition('IK_TRACKING_FAULT',d.site_xpos[sid],1);break
            qtarget+=np.clip(solution-qtarget,-.012,.012)
        for hold,q,v,kp in drive.holds:
            if hold not in ai:d.ctrl[hold]=drive.home[q]+d.qfrc_bias[v]/kp
        d.ctrl[drive.wheels]=0
        if front:
            from front_docking import PARAMETERS,facing_error
            lq=m.joint('lift_extension_joint').dofadr[0];mq=m.joint('lift_middle_joint').dofadr[0]
            d.ctrl[m.actuator('provisional_lift_motor').id]=.25+(d.qfrc_bias[lq]+.5*d.qfrc_bias[mq])/PARAMETERS['nori_lift']['kp_N_per_m']
        d.ctrl[ai]=qtarget+d.qfrc_bias[va]/250
        d.ctrl[grip]=-.0355 if phase in ('CLOSE','LIFT','CARRY','LOWER','SUPPORT') else 0
        if front and phase=='RELEASE':
            fraction=min(1,elapsed/1.5);fraction=fraction*fraction*(3-2*fraction)
            d.ctrl[grip]=release_q+.005*fraction
        elif front and phase=='RETREAT':d.ctrl[grip]=release_q+.005
        mujoco.mj_step(m,d)
        if front:
            bp,yaw,tilt=drive.pose();angle=abs(facing_error(bp[:2],yaw,[.29,0]));heading_peak=max(heading_peak,angle)
            from front_docking import docking_ready
            ready=docking_ready(bp[:2],yaw,[.29,0],float(np.linalg.norm(d.qvel[:3])),float(d.qvel[5]))
            dock_dwell=dock_dwell+timestep if ready else 0.
            lift_peak=max(lift_peak,abs(float(d.actuator_force[m.actuator('provisional_lift_motor').id])))
            if angle>math.radians(3):transition('NOT_FACING_EQUIPMENT',target,1);break
            if tilt>math.radians(12):transition('TILT_FAULT',target,1);break
        if not np.isfinite(d.qpos).all():transition('NONFINITE_FAULT',target,1);break
        forces={'a':0.,'b':0.};supported=False;unexpected=[]
        for ci,c in enumerate(d.contact):
            other=int(c.geom2) if c.geom1==pg else int(c.geom1) if c.geom2==pg else -1
            if other in jaws:
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);forces[jaws[other]]+=max(0,float(f[0]))
            if other>=0 and m.geom(other).name=='transfer_table':supported=True
            pair=[m.geom(c.geom1).name or '',m.geom(c.geom2).name or '']
            if c.dist<-.001 and 'transfer_table' in pair and other<0:unexpected.append(pair)
        if unexpected:fault_contacts=unexpected;transition('TABLE_COLLISION',target,1);break
        max_torque=max(max_torque,float(np.max(np.abs(d.actuator_force[ai]))))
        peak_lift=max(peak_lift,float(d.xpos[bid,2]-(PICK[2]-.029)))
        pe=float(np.linalg.norm(target-d.site_xpos[sid]));re=float(np.linalg.norm(rotation_error(ROTATION,d.site_xmat[sid].reshape(3,3))))
        at_target=pe<.0015 and re<.02 and t-phase_start>=duration
        if phase=='SETTLE' and at_target and (not front or dock_dwell>=.3):transition('APPROACH',PICK,4)
        elif phase=='APPROACH' and at_target:
            if np.linalg.norm(d.xpos[bid]-(PICK-[0,0,.029]))>.0015:transition('PAYLOAD_POSE_REJECTED',target,1);break
            transition('CLOSE',PICK,.2)
        elif phase=='CLOSE':
            dwell=dwell+m.opt.timestep if min(forces.values())>.005 else 0
            if dwell>.15:
                grasp_relative=d.site_xmat[sid].reshape(3,3).T@(d.xpos[bid]-d.site_xpos[sid]);transition('LIFT',PICK+[0,0,.04],4)
            elif elapsed>3:transition('NO_GRASP',target,1);break
        elif phase in ('LIFT','CARRY','LOWER','SUPPORT'):
            holding_samples+=1;both=min(forces.values())>.005;bilateral_samples+=int(both)
            lost_dwell=0 if both else lost_dwell+timestep
            if lost_dwell>.06:transition('CONTACT_LOST',target,1);break
            relative=d.site_xmat[sid].reshape(3,3).T@(d.xpos[bid]-d.site_xpos[sid]);slip=max(slip,float(np.linalg.norm(relative-grasp_relative)))
            if slip>.002:transition('GRASP_LOST',target,1);break
            if phase=='LIFT' and at_target and peak_lift>.03:transition('CARRY',PLACE+[0,0,.04],6)
            elif phase=='CARRY' and at_target:transition('LOWER',PLACE,4)
            elif phase=='LOWER' and at_target:transition('SUPPORT',PLACE,.5)
            elif phase=='SUPPORT':
                support_dwell=support_dwell+m.opt.timestep if supported and np.linalg.norm(d.qvel[m.joint('transfer_slide_free').dofadr[0]:][:3])<.005 else 0
                if support_dwell>.25:transition('RELEASE',PLACE,1)
                elif elapsed>3:transition('NO_SUPPORT',target,1);break
        elif phase=='RELEASE' and elapsed>(2 if front else 1):transition('RETREAT',PLACE+[0,0,.04],4)
        elif phase=='RETREAT' and at_target:
            final_error=np.linalg.norm(d.xpos[bid]-(PLACE-[0,0,.029]))
            final_speed=np.linalg.norm(d.qvel[m.joint('transfer_slide_free').dofadr[0]:][:3])
            if supported and final_error<.0015 and final_speed<.005:transition('COMPLETE',target,1)
            else:transition('PLACEMENT_FAILED',target,1)
            break
        if elapsed>duration+5 and phase not in ('CLOSE','SUPPORT'):transition('TRACKING_TIMEOUT',target,1);break
        if i%100==0:trace.append(dict(t=t,phase=phase,position_error_m=pe,angle_error_rad=re,slide=d.xpos[bid].tolist(),groove=d.site_xpos[sid].tolist(),forces_N=forces,jaw_q=[float(d.qpos[m.joint('lehisto_jaw_'+s+'_slide').qposadr[0]]) for s in ('a','b')],torque_Nm=d.actuator_force[ai].tolist()))
    if camera_renderer:camera_renderer.close()
    result=dict(passed=phase=='COMPLETE',phase=phase,timestep_s=timestep,grip_force_limit_N=grip_force,slide_offset_m=slide_offset,events=events,plan_errors=errors,peak_lift_m=peak_lift,relative_slip_m=slip,bilateral_contact_fraction=bilateral_samples/max(1,holding_samples),max_arm_torque_Nm=max_torque,final_slide_position_m=d.xpos[bid].tolist(),final_position_error_m=float(np.linalg.norm(d.xpos[bid]-(PLACE-[0,0,.029]))),fault_contacts=fault_contacts,trace=trace,
        head_camera_samples=camera_rows,front_facing=front,max_heading_error_deg=math.degrees(heading_peak),maximum_provisional_lift_force_N=lift_peak,
        scope='Actual seven-joint Nori arm; passive slide; development guide slots; '+('provisional actuated lift and head cameras' if front else 'braked lift')+'; not full lab validation.')
    if front:result['camera_check_passed']=bool(camera_rows) and all(r['interaction_visible_any'] for r in camera_rows)
    if save:(ROOT/('front_transfer_validation.json' if front else 'arm_transfer_validation.json')).write_text(json.dumps(result,indent=2))
    if front and camera_images and save:
        from PIL import Image
        montage=Image.new('RGB',(960,240*len(camera_images)))
        for r,row in enumerate(camera_images):
            for c,frame in enumerate(row):montage.paste(frame,(320*c,240*r))
        montage.save(ROOT/'head_camera_interactions.png')
    if render:
        from PIL import Image,ImageDraw,ImageFont
        with mujoco.Renderer(m,height=900,width=1280) as renderer:
            cam=mujoco.MjvCamera();cam.lookat[:]=[.15,-.05,.8] if front else [-.15,-.35,.8];cam.distance=1.4;cam.azimuth=120;cam.elevation=-25
            renderer.update_scene(d,cam);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle([0,0,1280,60],fill='#10283c');draw.text((15,15),'Actual Nori-arm contact transfer DEVELOPMENT: '+phase,fill='white',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',24));im.save(ROOT/('front_transfer.png' if front else 'arm_transfer.png'))
    return result

if __name__=='__main__':
    import sys
    result=run(render=True,front='--front' in sys.argv);print(json.dumps({k:v for k,v in result.items() if k!='trace'},indent=2))
