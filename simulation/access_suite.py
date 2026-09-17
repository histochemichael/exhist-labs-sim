"""Reproducible contact cycles and state recordings, never kinematic door playback."""
import argparse,json,hashlib,time,xml.etree.ElementTree as ET
import numpy as np
from build_scene import ROOT
from equipment_access import Access,KINDS

def source_hashes():
    files=['equipment_access.py','left_claw_contacts.py','quincy_offset_pull.py','access_planning.py']+[f'access_{k}_plan.json' for k in KINDS]
    return {n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in files}

def run(kind,negative=None,record=True):
    started=time.monotonic();a=Access(kind,jammed=negative=='jammed',miss=negative=='miss');states=[];phases=[];times=[];next_frame=0.;hashes=source_hashes()
    while a.d.time<100:
        done=a.step()
        if record and a.d.time>=next_frame:
            states.append(a.d.qpos.copy());phases.append(a.phase);times.append(a.d.time);next_frame+=.2
        if done:break
    r=a.result();r['source_sha256']=hashes;r['wall_seconds']=time.monotonic()-started;r['dt']=a.m.opt.timestep
    r['model_xml_sha256']=hashlib.sha256(ET.tostring(a.root)).hexdigest()
    r['internal_proxy_exemptions']=['nested telescopic tubes','proximal claw gears versus coarse wrist-pitch gear-socket box only']
    name=f'access_{kind}'+('_'+negative if negative else '')
    r['validation_ok']=(r['passed'] and r['bilateral_contact_fraction']>.99 and r['relative_slip_m']<.006 and r['max_arm_torque_Nm']<=4.000001 and r['max_lift_force_N']<=150.000001 and r['max_left_claw_mimic_error_rad']<=.005) if not negative else (not r['passed'] and a.terminal is not None and r['max_open']<(.01 if kind=='quincy' else .005) and r['phase'] in (('NO_GRASP','TRACKING_TIMEOUT') if negative=='miss' else ('JAM_STOP','FORCE_STOP','CONTACT_LOST','GRASP_LOST')))
    if source_hashes()!=hashes:r['validation_ok']=False;r['source_changed_during_test']=True
    (ROOT/(name+'_validation.json')).write_text(json.dumps(r,indent=2))
    if record:np.savez_compressed(ROOT/(name+'_states.npz'),qpos=np.array(states),phase=np.array(phases),time=np.array(times),kind=kind)
    print(json.dumps({k:v for k,v in r.items() if k not in ('trace','events','source_sha256')}),flush=True)
    return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--kind',choices=KINDS);p.add_argument('--negative',choices=['jammed','miss']);args=p.parse_args()
    results=[run(k,args.negative) for k in ([args.kind] if args.kind else KINDS)]
    if not all(r['validation_ok'] for r in results):raise SystemExit(1)
