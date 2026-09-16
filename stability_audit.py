"""Live-physics stability measurements; never freezes bodies or rewrites qpos."""
import argparse,json,hashlib,time
import numpy as np
import mujoco
from build_scene import ROOT
from equipment_access import Access,KINDS
from access_suite import source_hashes

QUIET=('SETTLE','GRIP','HOLD','RELEASE','COMPLETE')

def span(rows,key):
    v=np.asarray([r[key] for r in rows])
    return float(np.linalg.norm(np.ptp(v,axis=0))) if len(v)>1 else 0.

def access_case(kind,tag='current',acquisition_only=False):
    start=time.monotonic();a=Access(kind);a.duration=8.;a.hold_seconds=5.;hashes=source_hashes()
    rows=[];transitions=[];last_phase=a.phase;next_sample=0.;initial_q=float(a.d.qpos[a.jq])
    right=[a.m.joint('right_'+n+'_joint').qposadr[0] for n in ('shoulder_pitch','shoulder_roll','bicep_yaw','elbow_pitch','forearm_yaw','wrist_pitch','wrist_roll')]
    max_step=0.;max_lift_step=0.;untouched_travel=0.;last_command=a.qtarget.copy();last_lift=a.lift_command
    while a.d.time<110:
        previous_phase=a.phase;a.step()
        max_step=max(max_step,float(np.max(np.abs(a.qtarget-last_command))))
        max_lift_step=max(max_lift_step,abs(a.lift_command-last_lift))
        last_command=a.qtarget.copy();last_lift=a.lift_command
        if previous_phase=='SETTLE':untouched_travel=max(untouched_travel,abs(float(a.d.qpos[a.jq])-initial_q))
        if a.phase!=last_phase:
            transitions.append(dict(t=float(a.d.time),before=last_phase,after=a.phase));last_phase=a.phase
        if a.d.time>=next_sample:
            # mj_step's Cartesian caches precede its final integration. Refresh
            # positions only for synchronized measurements, never change state.
            mujoco.mj_kinematics(a.m,a.d)
            rows.append(dict(t=float(a.d.time),age=float(a.d.time-a.start),phase=a.phase,
                tcp=a.d.site_xpos[a.sid].tolist(),rotation=a.d.site_xmat[a.sid].tolist(),
                command=a.qtarget.tolist(),lift_command=float(a.lift_command),
                right=a.d.qpos[right].tolist(),door=float(a.d.qpos[a.jq]),
                velocity=float(abs(a.d.qvel[a.jv]))))
            next_sample+=.002
        if a.terminal is not None and a.d.time-a.terminal>=10.:break
        if acquisition_only and a.phase=='OPEN':break
    phases={}
    for phase in QUIET:
        allrows=[r for r in rows if r['phase']==phase]
        # Settling is included separately; quiet-window threshold is not used to
        # conceal an acquisition jump (full-phase command spans also checked).
        quiet=[r for r in allrows if r['age']> (1. if phase in ('SETTLE','HOLD','COMPLETE') else .2)]
        phases[phase]=dict(samples=len(allrows),quiet_samples=len(quiet),
            command_span_rad=span(allrows,'command'),lift_command_span_m=span(allrows,'lift_command'),
            full_tcp_span_m=span(allrows,'tcp'),quiet_tcp_span_m=span(quiet,'tcp'),
            quiet_right_joint_span_rad=span(quiet,'right'),quiet_equipment_span=span(quiet,'door'),
            quiet_max_equipment_speed=max([r['velocity'] for r in quiet],default=0.))
    result=dict(kind=kind,tag=tag,phase=a.phase,sim_seconds=float(a.d.time),wall_seconds=time.monotonic()-start,
        source_sha256=hashes,source_unchanged=hashes==source_hashes(),sample_hz=500,
        untouched_equipment_travel=untouched_travel,equipment_unit='rad' if kind=='quincy' else 'm',
        max_arm_command_step_rad=max_step,max_lift_command_step_m=max_lift_step,phases=phases,
        access_result={k:v for k,v in a.result().items() if k not in ('trace','events')},transitions=transitions,
        scope='Fixed-dock actual arm and passive equipment; 8 s initial idle and 10 s live terminal hold. Not full-lab or hardware certification.')
    checks=dict(completed=a.phase=='COMPLETE',sources_unchanged=result['source_unchanged'],
        stationary_arm_targets=all(p['command_span_rad']<1e-12 and p['lift_command_span_m']<1e-12 for p in phases.values()),
        idle_tip_stable=all(phases[p]['quiet_tcp_span_m']<.0005 for p in ('SETTLE','HOLD','COMPLETE')),
        no_untouched_door_motion=untouched_travel<(1e-4 if kind=='quincy' else .0001),
        command_slew_bounded=max_step<=.01500001 and max_lift_step<=.000800001)
    result['checks']={k:bool(v) for k,v in checks.items()};result['passed']=all(checks.values())
    (ROOT/f'stability_{kind}_{tag}.json').write_text(json.dumps(result,indent=2))
    np.savez_compressed(ROOT/f'stability_{kind}_{tag}.npz',time=[r['t'] for r in rows],phase=[r['phase'] for r in rows],age=[r['age'] for r in rows],
        tcp=[r['tcp'] for r in rows],rotation=[r['rotation'] for r in rows],command=[r['command'] for r in rows],lift=[r['lift_command'] for r in rows],door=[r['door'] for r in rows])
    print(json.dumps(result),flush=True);return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--kind',choices=KINDS,default='quincy');p.add_argument('--tag',default='current');p.add_argument('--acquisition-only',action='store_true');args=p.parse_args()
    access_case(args.kind,args.tag,args.acquisition_only)
