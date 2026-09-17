"""Wheel-driven equipment-facing dock with the approved provisional lift servo."""
import json,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import mobile_base_bench as mobile
from build_scene import ROOT,el
from front_docking import add_head_cameras,apply_provisional_lift,PARAMETERS,facing_yaw,facing_error,docking_ready

def run(save=True):
    root=mobile.build();apply_provisional_lift(root);add_head_cameras(root)
    el(root.find('worldbody'),'geom',name='equipment_front',type='box',pos='.45 1 .85',size='.25 .025 .12',rgba='.35 .4 .5 1')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d,drive=mobile.initialize(m)
    target=[.45,.35];equipment=[.45,1.]
    drive.targets=[(.45,0,math.pi/2),(*target,facing_yaw(target,equipment))]
    ai=m.actuator('provisional_lift_motor').id;va=m.joint('lift_extension_joint').dofadr[0];vm=m.joint('lift_middle_joint').dofadr[0]
    dwell=0.;lift_peak=0.;stopped=None
    for i in range(20000):
        drive.update();d.ctrl[ai]=.25+(d.qfrc_bias[va]+.5*d.qfrc_bias[vm])/PARAMETERS['nori_lift']['kp_N_per_m']
        mujoco.mj_step(m,d);p,yaw,tilt=drive.pose();lift_peak=max(lift_peak,abs(float(d.actuator_force[ai])))
        ready=drive.phase=='COMPLETE' and docking_ready(p[:2],yaw,equipment,float(np.linalg.norm(d.qvel[:3])),float(d.qvel[5]))
        dwell=dwell+m.opt.timestep if ready else 0
        if dwell>=PARAMETERS['docking']['settle_s']:stopped=d.time;break
    result=dict(passed=stopped is not None,seconds=d.time,phase=drive.phase,position_m=p.tolist(),heading_error_deg=math.degrees(facing_error(p[:2],yaw,equipment)),
        speed_mps=float(np.linalg.norm(d.qvel[:3])),yaw_speed_radps=float(d.qvel[5]),settled_s=dwell,maximum_lift_force_N=lift_peak,
        base_pose_commands_after_initialization=0,lift_brake=False,scope='Standalone physical docking with provisional lift/chassis/casters; no payload or full-lab navigation.')
    if save:(ROOT/'equipment_dock_validation.json').write_text(json.dumps(result,indent=2))
    return result

if __name__=='__main__':r=run();print(json.dumps(r,indent=2));assert r['passed']
