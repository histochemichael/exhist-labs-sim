"""Video-rate geometry regression checks. Deliberately NOT physics validation."""
import json,hashlib
import numpy as np
import mujoco
from promo_demo import MODEL,OUT,FPS,RACK_GRIP,GRIP_R,RIGHT,kinematics,exact_grip_playback
from promo_clearance import Clearance
from promo_geometry import VesselClearance,BACK_RACK
import xml.etree.ElementTree as ET

def run():
    m=mujoco.MjModel.from_xml_path(str(MODEL));d=mujoco.MjData(m);s=np.load(OUT/'promo_states.npz');r=json.loads((OUT/'promo_manifest.json').read_text())
    s=dict(s) # Cache compressed NPZ arrays once, rather than decompress per frame.
    action=next(e['start'] for e in r['events'] if e['label']=='FADE TO WHITE');c=Clearance(m,d)
    mid=m.body('B01_leica_rack').mocapid[0];probe=m.geom('promo_handle_beam_probe').id;ft=np.zeros(6)
    faces=[[g for g in range(m.ngeom) if m.geom(g).name.startswith(f'nori_groove_{side}_handle_')] for side in ('a','b')]
    clear=[];grip=[];pose=[];vessel=[];tilts=[];v=VesselClearance(m,d)
    for n in range(1800):
        t=n/30;st=t*action/54.5 if t<54.5 else action+t-54.5;f=st*FPS;i=min(int(f),len(s['qpos'])-1);j=min(i+1,len(s['qpos'])-1);u=np.clip(f-i,0,1)
        d.qpos[:]=(1-u)*s['qpos'][i]+u*s['qpos'][j];d.mocap_pos[mid]=(1-u)*s['rack'][i]+u*s['rack'][j];rq=(1-u)*s['rack_quat'][i]+u*s['rack_quat'][j];d.mocap_quat[mid]=rq/np.linalg.norm(rq)
        exact_grip_playback(m,d,s,f);door=d.qpos[m.joint('base_x').qposadr[0]]<-4
        rack_rotation=d.xmat[m.body('B01_leica_rack').id].reshape(3,3)
        tilts.append(float(np.rad2deg(np.arccos(np.clip(rack_rotation[2,2],-1,1)))))
        clear.append([float(c.distances(side,door).min()) for side in ('left','right')])
        if str(s['label'][i]) in ('LOWER INTO STAINER CONTAINER','RELEASE RACK IN CONTAINER','RIGHT GRIPPER CLEAR','OPEN JAWS ABOVE CONTAINER'):
            vessel.append(v.gaps()[0])
        if s['custody'][i]=='right':
            grip.append([float(min(mujoco.mj_geomDistance(m,d,g,probe,.1,ft) for g in group)) for group in faces])
            p=d.xpos[m.body('B01_leica_rack').id];R=d.xmat[m.body('B01_leica_rack').id].reshape(3,3)
            pose.append(float(np.linalg.norm(p+R@RACK_GRIP-d.site('nori_right_handle_groove').xpos)))
    lift=s['rack'][s['label']=='STAINER LIFTS RACK',2];sort_delta=s['sort_pos'][-1]-s['sort_pos'][0]
    report=dict(video_frames_checked=1800,min_head_or_first_oven_clearance_m=np.min(clear,axis=0).tolist(),minimum_handle_face_gap_m=np.min(grip,axis=0).tolist(),maximum_handle_face_gap_m=np.max(grip,axis=0).tolist(),max_held_grasp_frame_error_m=max(pose),stainer_rack_lift_m=float(lift[-1]-lift[0]),sorted_slide_displacements_m=sort_delta.tolist(),joint_limit_violations=r['joint_limit_violations'],physical_success=False,scope='All video frames; selected Nori arm geometry versus head and first oven door. CAD top-bar versus both handle-groove faces. Not full collision, force, slip or hardware validation.')
    assert np.min(clear)>.003,report
    assert np.min(grip)>-.0001 and np.max(grip)<.00015,report
    assert max(pose)<1e-7 and lift[-1]-lift[0]>.100,report
    assert np.allclose(sort_delta,np.tile([.19,0,0],(4,1)),atol=1e-6),report
    assert not r['joint_limit_violations'],report
    assert max(tilts)<1.,('Loaded rack tilted',max(tilts))
    loaded=(s['custody'][:-1]=='right')&(s['custody'][1:]=='right')
    ids=[m.joint(name).qposadr[0] for name in RIGHT]
    step=float(np.max(abs(np.diff(s['qpos'][:,ids],axis=0)[loaded])))
    assert step<.30,('Loaded arm branch switch',step)
    report['max_loaded_arm_joint_step_rad']=step
    report['maximum_rack_tilt_deg']=max(tilts)
    report['gripper_tilt_deg']=r['gripper_tilt_deg']
    assert r['gripper_tilt_deg']==0
    tree=ET.parse(MODEL).getroot()
    for k in (1,2,3):
        oven=tree.find(f".//body[@name='quincy_{k}']")
        assert not any(g.get('mesh')=='passive_quincy_9' for g in oven.iter('geom'))
        bar=oven.find(f".//geom[@name='promo_oven_{k}_pull_bar']")
        assert bar is not None and np.max(np.fromstring(bar.get('rgba'),sep=' ')[:3])<.07
    oj=[j for j in range(m.njnt) if 'quincy_1' in m.joint(j).name and m.jnt_type[j]==mujoco.mjtJoint.mjJNT_HINGE]
    assert len(oj)==1
    departure=next(e['start'] for e in r['events'] if e['label']=='OVEN DEPART')
    closed_error=float(np.abs(s['qpos'][round(departure*FPS):,m.jnt_qposadr[oj[0]]]).max())
    assert closed_error<1e-6,closed_error
    assert min(vessel)>.0005,min(vessel)
    rear=s['rack'][s['custody']=='rear_bath'];rear_error=float(np.linalg.norm(rear-BACK_RACK,axis=1).max())
    assert rear_error<.0001,rear_error
    solid=json.loads((OUT/'promo_solid_audit_step1.json').read_text())
    assert solid['frames_checked']==1800 and not solid['intersections']
    assert solid['scene_sha256']==hashlib.sha256(MODEL.read_bytes()).hexdigest()
    assert solid['states_sha256']==hashlib.sha256((OUT/'promo_states.npz').read_bytes()).hexdigest()
    report.update(min_container_clearance_m=min(vessel),oven_closed_error_rad=closed_error,final_rear_container_position_error_m=rear_error,handles='one black pull per oven; original handles removed',full_frame_solid_intersections=0)
    report['scope']='1800 video-frame geometric checks of the oven/stainer interaction corridor; CAD solids plus conservative hulls for non-manifold robot links. Intentional claw/handle and fork/rack contacts excluded. Staged playback, not force, slip, balance, continuous-time or hardware certification.'
    report['scoped_geometry_checks_passed']=True
    (OUT/'promo_postflight.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':run()
