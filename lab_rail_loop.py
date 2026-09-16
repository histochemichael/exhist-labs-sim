"""Live, guarded reference-control LeHisto loop inside the complete lab view.

The imported controller is unchanged except for a yield at each 25 Hz log.
No recorded payload trajectory is used to drive live physics. Other equipment
is held visual context until separate coordination/contacts are integrated.
"""
import argparse
import ast
from collections import deque
import hashlib
import json
from pathlib import Path
import time
import mujoco
import numpy as np
from build_scene import ROOT
from integrate_rail_loop import REFERENCE, MODEL


def verify_snapshot():
    manifest=json.loads((REFERENCE/'snapshot.json').read_text())
    for name,digest in manifest['sha256'].items():
        if hashlib.sha256((REFERENCE/name).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Pinned loop dependency changed: '+name)


def selected(filename,names):
    tree=ast.parse((REFERENCE/filename).read_text())
    nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
    assert {n.name for n in nodes}==set(names),(filename,names)
    return ast.Module(body=nodes,type_ignores=[])


def controller():
    verify_snapshot()
    integration=json.loads((ROOT/'rail_loop_integration.json').read_text())
    for path,key in [(MODEL,'model_sha256'),(ROOT/'exhist_operational.xml','lab_sha256')]:
        if hashlib.sha256(path.read_bytes()).hexdigest()!=integration[key]:
            raise RuntimeError('Lab scene changed; refresh with integrate_rail_loop.py before running this composition.')
    if mujoco.__version__!='3.4.0':
        raise RuntimeError('Use the validated lerobot environment (MuJoCo 3.4.0), via launch.ps1 -LeHistoLoop.')
    import placo
    layout=json.loads((REFERENCE/'rail_loop_layout.json').read_text())
    arm=['rail_travel','base_link_to_link1','link1_to_link2','link2_to_link3','link3_to_link4','link4_to_link5']
    common=dict(np=np,mujoco=mujoco,placo=placo,ROOT=REFERENCE,GRASP=np.array([.01,-.23,.0522]),ARM=arm)
    exec(compile(selected('reference_trial.py',['IK']),'pinned_IK','exec'),common)
    exec(compile(selected('multi_container_scene.py',['rz']),'pinned_rz','exec'),common)
    exec(compile(selected('rail_loop_controller.py',['YawIK']),'pinned_YawIK','exec'),common)
    common.update(RAIL_LO=-.06,RAIL_HI=.21,ORIGIN=np.array([-.000099,-.007243,0.]))
    exec(compile(selected('rail_loop.py',['RailIK','trajectory']),'pinned_RailIK','exec'),common)
    namespace=dict(common,ROOT=ROOT,SEAT=layout['seat_z'],RAIL_BOUNDS=layout['rail_bounds_m'],json=json,deque=deque)
    tree=selected('rail_loop_controller.py',['smooth','run'])
    inserts=0
    for node in ast.walk(tree):
        if isinstance(node,ast.If) and ast.unparse(node.test)=='d.time >= next_log':
            node.body+=ast.parse("yield dict(m=m,d=d,phase=phase,source=source['id'],destination=dest['id'],completed=len(completed),sample=trace[-1])").body
            inserts+=1
    assert inserts==1,'Pinned controller logging hook changed'
    ast.fix_missing_locations(tree)
    exec(compile(tree,'pinned_rail_loop_with_log_yield','exec'),namespace)
    return namespace['run'](hops=11,name='rail_loop_lab',scene_path=MODEL,
                            layout_path=REFERENCE/'rail_loop_layout.json',route_ids=layout['route'],initial_fk_correction=True)


class LiveLoop:
    def __init__(self):
        self.iterator=controller();self.current=next(self.iterator)
        self.m=self.current['m'];self.d=self.current['d'];self.result=None
        self.layout=json.loads((REFERENCE/'rail_loop_layout.json').read_text())

    def advance(self,target):
        while self.d.time<target:
            if self.result is not None:
                # Hold the last motor targets, but keep actual contact dynamics live.
                mujoco.mj_step(self.m,self.d)
                continue
            try:self.current=next(self.iterator)
            except StopIteration as end:
                self.result=end.value
        return self.current

    def close(self):self.iterator.close()

    def status(self):
        phase=self.current['phase'] if self.result is None else ('COMPLETE' if self.result['passed'] else 'STOP: '+str(self.result['failure']))
        count=self.current['completed'] if self.result is None else len(self.result['completed_transfers'])
        rail=float(self.d.joint('rail_travel').qpos[0])*1000
        return (f"ExHist Labs | LIVE LeHisto contact loop | {count}/11 transfers\n"
                f"Bath {self.current['source']} -> {self.current['destination']} | {phase} | {self.d.time:.1f}s | rail {rail:+.1f} mm\n"
                "Original gripper + 145 g rack + one-inch inserts | full 270 mm modeled stroke\n"
                "Other equipment held. Nori handoffs / room-wide interactions are not validated.")


def camera(layout,kind='station',station_x=None):
    c=mujoco.MjvCamera();R=np.array(layout['world_rotation']);t=np.array(layout['world_translation'])
    if kind=='overview':point=[0,-1.7,1.05];c.distance=16.6;c.azimuth=190;c.elevation=-38
    elif station_x is not None:point=[station_x,.45,1.0];c.distance=2.9;c.azimuth=180;c.elevation=-25
    else:point=[1.08,.54,.92];c.distance=1.6;c.azimuth=210;c.elevation=-38
    c.lookat[:]=R.T@(np.array(point)-t)
    return c


def write_validation(m,d,result,rows,hold_source='continuous live run'):
        qs=np.array([r['qpos'] for r in rows]);bad=[]
        for j in range(m.njnt):
            if not m.jnt_limited[j]:continue
            values=qs[:,m.jnt_qposadr[j]];lo,hi=m.jnt_range[j]
            tol=.0005 if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else .002
            if values.min()<lo-tol or values.max()>hi+tol:bad.append(m.joint(j).name)
        qa=m.joint('rail_travel').qposadr[0];va=m.joint('rail_travel').dofadr[0];rail=qs[:,qa]
        speed=max(abs(r['qvel'][va]) for r in rows)
        # Do not freeze the final released rack to claim stability.
        final=[]
        for _ in range(2000):
            mujoco.mj_step(m,d)
            final.append(d.body('rack').xpos.copy())
        span=float(np.linalg.norm(np.ptp(final,axis=0)))
        checks=dict(route=result['passed'],no_joint_limit_violations=not bad,
                    full_stroke=rail.min()<-.059 and rail.max()>.209,
                    rail_returns=abs(rail[-1]-rail[0])<.001,speed_within_limit=speed<=.1,
                    released_rack_stable=span<.0001)
        checks={key:bool(value) for key,value in checks.items()}
        result.update(passed=all(checks.values()),integration_checks=checks,
                      joint_limit_violations=bad,rail_actual_range_m=[float(rail.min()),float(rail.max())],
                      rail_peak_speed_m_s=speed,post_loop_hold_s=2.,released_rack_span_m=span,
                      post_loop_hold_initialization=hold_source,
                      model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
                      trace_sha256=hashlib.sha256((ROOT/'rail_loop_lab_trace.json').read_bytes()).hexdigest(),
                      reference_manifest_sha256=hashlib.sha256((REFERENCE/'snapshot.json').read_bytes()).hexdigest(),
                      scope='Fresh live reference-control test in the whole-lab view using actual bounded bench support. Other equipment is static context. No Nori handoff, loose-slide, fluid, whole-room collision or hardware validation.')
        (ROOT/'rail_loop_lab_validation.json').write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('events','completed_transfers')},indent=2))
        return result['passed']


def validate():
    loop=LiveLoop()
    try:
        while loop.result is None:loop.advance(loop.d.time+1.)
        rows=json.loads((ROOT/'rail_loop_lab_trace.json').read_text())
        return write_validation(loop.m,loop.d,loop.result,rows)
    finally:loop.close()


def recover_report(expected_model_sha256):
    """Recover an export failure, never fabricate a new route or alter its trace."""
    if hashlib.sha256(MODEL.read_bytes()).hexdigest()!=expected_model_sha256:
        raise RuntimeError('Recovery model does not match the completed physics run')
    verify_snapshot()
    result=json.loads((ROOT/'rail_loop_lab.json').read_text())
    assert result['passed'] and len(result['completed_transfers'])==11
    rows=json.loads((ROOT/'rail_loop_lab_trace.json').read_text())
    m=mujoco.MjModel.from_xml_path(str(MODEL));d=mujoco.MjData(m);last=rows[-1]
    d.qpos[:]=last['qpos'];d.qvel[:]=last['qvel'];d.ctrl[:]=last['ctrl'];d.time=last['t']
    mujoco.mj_forward(m,d)
    return write_validation(m,d,result,rows,'restored measured final state after report-export error; separate 2 s live hold')


def view(speed=1.,autostart=False):
    import mujoco.viewer
    loop=LiveLoop();keys=deque();paused=not autostart;layout=json.loads((ROOT/'layout.json').read_text())
    from rail_loop_visuals import apply as apply_visuals
    apply_visuals(loop.m)
    with mujoco.viewer.launch_passive(loop.m,loop.d,key_callback=keys.append) as viewer:
        def setcam(kind='station',station_x=None):
            c=camera(loop.layout,kind,station_x)
            viewer.cam.lookat[:]=c.lookat;viewer.cam.distance=c.distance;viewer.cam.azimuth=c.azimuth;viewer.cam.elevation=c.elevation
        setcam();viewer.opt.geomgroup[3:]=0;viewer.opt.sitegroup[:]=0
        last=time.monotonic()
        try:
            while viewer.is_running():
                now=time.monotonic();dt=min(.1,now-last);last=now
                with viewer.lock():
                    while keys:
                        k=keys.popleft()
                        if k==32:paused=not paused
                        elif k==79:setcam('overview')
                        elif k==76:setcam()
                        elif 49<=k<=56:setcam(station_x=layout['stations'][k-49]['x'])
                        elif k in (61,334):speed=min(4.,speed*2)
                        elif k in (45,333):speed=max(.25,speed/2)
                    if not paused:loop.advance(loop.d.time+dt*speed)
                viewer.set_texts([(100,mujoco.mjtGridPos.mjGRID_TOPLEFT,loop.status(),''),
                                  (100,mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,('PAUSED | ' if paused else '')+'SPACE start/pause | L LeHisto | O whole lab | 1-8 stations | +/- speed','')])
                viewer.sync();time.sleep(.01)
        finally:loop.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--validate',action='store_true');p.add_argument('--recover-report',metavar='KNOWN_MODEL_SHA256');p.add_argument('--autostart',action='store_true');p.add_argument('--speed',type=float,default=1.)
    a=p.parse_args()
    if a.recover_report:raise SystemExit(0 if recover_report(a.recover_report) else 1)
    if a.validate:raise SystemExit(0 if validate() else 1)
    view(a.speed,a.autostart)
