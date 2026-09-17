"""Loaded LeHisto admission profile; never equate radial reach with a safe path."""
import json,math,hashlib
from pathlib import Path
import numpy as np
from build_scene import ROOT
from handling_contracts import Decision

def profile():
    p=json.loads((ROOT/'lehisto_loaded_reach.json').read_text())
    lo,hi=p['tested_radius_m'];margin=p['design_margin_m']
    if not 0<=margin<(hi-lo)/2:raise ValueError('Design margin must remain inside tested bounds')
    return p

def row_position(index):
    p=profile()['layout']
    if not 1<=index<=p['count']:raise ValueError(index)
    return np.array([p['row_center_x_m']+(index-(p['count']+1)/2)*p['pitch_m'],p['row_y_m']])

def radial_check(center_world,shoulder_world,front_world=(0,-1,0),p=None):
    p=profile() if p is None else p
    delta=np.asarray(center_world,float)[:2]-np.asarray(shoulder_world,float)[:2]
    front=np.asarray(front_world,float)[:2]
    if not np.isfinite(np.r_[delta,front]).all() or np.linalg.norm(front)<1e-8:
        return dict(radius_m=None,yaw_deg=None,reasons=['invalid_reach_frame'])
    radius=float(np.linalg.norm(delta));front=front/np.linalg.norm(front)
    yaw=math.degrees(math.atan2(front[0]*delta[1]-front[1]*delta[0],float(front@delta)))
    lo,hi=p['tested_radius_m'];margin=p['design_margin_m'];reasons=[]
    if radius<lo-1e-9 or radius>hi+1e-9:reasons.append('outside_tested_loaded_radius')
    elif radius<lo+margin-1e-9 or radius>hi-margin+1e-9:reasons.append('inside_tested_bound_but_outside_design_margin')
    if abs(yaw)>p['design_yaw_limit_deg']+1e-8:reasons.append('outside_front_horseshoe')
    return dict(radius_m=radius,yaw_deg=yaw,reasons=reasons)

def transfer_check(*,robot,center_world,shoulder_world,front_world,conditions,
                   trajectory_validated=False,collision_free=None,bottom_clearance_m=None,grasp_frame_matched=False,p=None):
    p=profile() if p is None else p
    if robot!='LeHisto':return Decision(('profile_not_applicable_to_robot',))
    reasons=radial_check(center_world,shoulder_world,front_world,p)['reasons']
    for key,nominal in p['conditions'].items():
        value=conditions.get(key)
        if value is None or not np.isfinite(value) or abs(value-nominal)>1e-6:
            reasons.append('condition_requires_revalidation:'+key)
    if not trajectory_validated:reasons.append('placed_loaded_trajectory_unvalidated')
    if not grasp_frame_matched:reasons.append('source_grasp_frame_mapping_unverified')
    if collision_free is not True:reasons.append('swept_collision_check_missing_or_failed')
    if bottom_clearance_m is None or not np.isfinite(bottom_clearance_m) or bottom_clearance_m<p['minimum_transfer_clearance_m']:
        reasons.append('rack_bottom_clearance_missing_or_insufficient')
    return Decision(tuple(reasons))

def rig_frame(m,d,prefix):
    """Use actual FK shoulder anchor, including the current carriage/rig pose."""
    ji=m.joint(prefix+'_j1').id
    f=profile()['frame'];front=np.array(f['source_nominal_jar_m'])-f['source_shoulder_m'];front[2]=0;front/=np.linalg.norm(front)
    return d.xanchor[ji].copy(),d.body(prefix).xmat.reshape(3,3)@np.array(f['source_to_lab_rig_rotation'])@front

def audit_scene(m,d,output=True):
    p=profile();rows=[]
    for rig in ('sorter_1','sorter_2','special_lehisto','sendout_lehisto_1','sendout_lehisto_2'):
        shoulder,front=rig_frame(m,d,rig)
        if rig!='special_lehisto':
            rows.append(dict(rig=rig,shoulder_world_m=shoulder.tolist(),status='No matched loaded container route assigned'));continue
        rail=float(d.qpos[m.joint(rig+'_carriage').qposadr[0]])
        for i in range(1,12):
            c=d.body('special_staining_jar_'+str(i)).xpos.copy();radial=radial_check(c,shoulder,front,p)
            # The lab's CAD-derived arm is higher than the tested source arm.
            # Do not silently certify that different height or moving rail.
            conditions={**p['conditions'],'rail_m':rail,'jar_origin_relative_shoulder_z_m':float(c[2]-shoulder[2]),'payload_kg':None,'arm_cap_Nm':None}
            decision=transfer_check(robot='LeHisto',center_world=c,shoulder_world=shoulder,front_world=front,conditions=conditions,p=p)
            rows.append(dict(rig=rig,jar=i,active=i in p['layout']['active_jars'],center_world_m=c.tolist(),shoulder_world_m=shoulder.tolist(),front_world=front.tolist(),**radial,conditions=conditions,tool_approach_tilt_deg=30,transfer_allowed=decision.allowed,transfer_blockers=decision.reasons))
    source=Path(p['source_directory']);provenance={}
    for name in ('REACH_RESULTS.md','reach_verification.json','reach_screen.json','reach_transfer_layout.json','reach_transfer.json','multi_container_layout.json'):
        provenance[name]=hashlib.sha256((source/name).read_bytes()).hexdigest()
    report=dict(profile=p,rows=rows,source_sha256=provenance,source_modified=False,scope='Placement/admission audit only. Existing choreography is not loaded physics validation.')
    if output:(ROOT/'lehisto_lab_reach_audit.json').write_text(json.dumps(report,indent=2))
    return report

if __name__=='__main__':
    import mujoco
    m=mujoco.MjModel.from_xml_path(str(ROOT/'exhist_first_pass.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    r=audit_scene(m,d)
    print(json.dumps([{k:v for k,v in x.items() if k in ('rig','jar','active','radius_m','reasons','conditions')} for x in r['rows']],indent=2))
