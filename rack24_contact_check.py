"""Actual 24-rack T-bar on the original groove fixture, with gravity.

The upstream Cartesian lift is a test fixture, not Nori. No payload attachment,
teleportation or adhesion. Rack mass and servo force are provisional sweeps.
"""
import json,math
import numpy as np
import mujoco
import xml.etree.ElementTree as ET
from groove_bench import build as groove_fixture
from build_scene import ROOT,el,vec


def build(mass=.145,force=2.,yaw_error=0):
    root=groove_fixture('handle')
    fixture=root.find(".//body[@name='actuated_test_fixture']")
    fixture.set('pos',vec([0,.0933,.025+.0925+.0961885]))
    support=root.find(".//geom[@name='support']");support.set('pos','0 0 .0125');support.set('size','.1 .1 .0125')
    payload=root.find(".//body[@name='payload']")
    for e in list(payload):payload.remove(e)
    payload.set('pos','0 0 .025');payload.set('euler',vec([0,0,yaw_error]))
    el(payload,'freejoint',name='payload_free')
    el(payload,'inertial',pos='0 0 .04',mass=mass,diaginertia='.00015 .00008 .00012')
    source=json.loads((ROOT/'assets/rack24.json').read_text())['parts'][0]
    v=np.array(source['vertices']).reshape(-1,3)
    # Rack long axis perpendicular to rail. T-bar is along fixture X.
    rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
    v=(v+[0,0,.0015])@rotation.T
    el(root.find('asset'),'mesh',name='actual_rack24',vertex=vec(v.ravel()),face=' '.join(map(str,source['triangles'])),inertia='shell')
    el(payload,'geom',name='actual_rack24_visual',type='mesh',mesh='actual_rack24',rgba='.65 .67 .68 1',contype='0',conaffinity='0',density='0')
    # Exact top-bar extents from CAD; lower envelope is conservative, not slots.
    for name,pos,size in [('actual_T_bar',[0,0,.0925],[.01,.001,.0015]),
                          ('actual_handle_neck',[0,0,.080],[.0045,.001,.011]),
                          ('rack_lower_envelope',[0,0,.033],[.016647,.044,.033])]:
        el(payload,'geom',name=name,type='box',pos=vec(pos),size=vec(size),density='0',rgba='0 0 0 0',friction='.6 .005 .0001',condim='4',solref='.004 1',solimp='.95 .99 .001')
    root.find("actuator/position[@name='jaw_servo']").set('forcerange',vec([-force,force]))
    return root


def run(mass=.145,force=2.,yaw_error=0,render=False):
    m=mujoco.MjModel.from_xml_string(ET.tostring(build(mass,force,yaw_error),encoding='unicode'));d=mujoco.MjData(m)
    bid=m.body('payload').id;fid=m.body('actuated_test_fixture').id
    contacts={m.geom('actual_T_bar').id,m.geom('actual_handle_neck').id}
    jaws={i:('a' if m.geom(i).name.startswith('groove_a_') else 'b') for i in range(m.ngeom) if m.geom(i).name.startswith('groove_')}
    phase='SETTLE';phase_t=0;dwell=0;origin=None;slip=0.;peak=0.;both_count=0;total=0;lift=0.;events=[];release_z=None
    for _ in range(10000):
        t=d.time
        if phase=='SETTLE' and t>.3:phase='CLOSE';phase_t=t
        if phase=='LIFT':lift=.04*min(1,(t-phase_t)/1.3)
        d.ctrl[:]=[lift,-.0355 if phase in ('CLOSE','LIFT','HOLD') else 0.]
        mujoco.mj_step(m,d)
        forces={'a':0.,'b':0.}
        for ci,c in enumerate(d.contact):
            other=c.geom2 if c.geom1 in contacts else c.geom1 if c.geom2 in contacts else -1
            if other in jaws:
                f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);forces[jaws[other]]+=max(0,float(f[0]))
        both=min(forces.values())>.005;peak=max(peak,float(d.xpos[bid,2]-.025))
        if phase=='CLOSE':
            dwell=dwell+1 if both else 0
            if dwell>=120:phase='LIFT';phase_t=t;origin=d.xpos[bid]-d.xpos[fid];events.append('bilateral_contact_then_lift')
            elif t-phase_t>3:phase='NO_GRASP';events.append('no_bilateral_grasp')
        elif phase in ('LIFT','HOLD'):
            slip=max(slip,float(np.linalg.norm(d.xpos[bid]-d.xpos[fid]-origin)))
            if slip>.003:phase='SLIPPED';events.append('slip_stop')
            elif phase=='LIFT' and t-phase_t>1.6:phase='HOLD';phase_t=t
            elif phase=='HOLD':
                total+=1;both_count+=both
                if t-phase_t>1.5:
                    if render:
                        from PIL import Image
                        with mujoco.Renderer(m,height=600,width=800) as rr:
                            c=mujoco.MjvCamera();c.lookat[:]=[0,0,.14];c.distance=.32;c.azimuth=130;c.elevation=-12
                            rr.update_scene(d,c);Image.fromarray(rr.render()).save(ROOT/'rack24_contact_fixture.png')
                    release_z=float(d.xpos[bid,2]);phase='RELEASE';events.append('open_jaws_drop_test')
    return dict(mass_kg=mass,force_N=force,yaw_error_deg=math.degrees(yaw_error),phase=phase,
                passed=phase=='RELEASE' and peak>.03 and both_count/max(1,total)>.99 and release_z is not None and release_z-float(d.xpos[bid,2])>.02,
                peak_lift_m=peak,relative_slip_m=slip,bilateral_hold_fraction=both_count/max(1,total),
                release_drop_m=None if release_z is None else release_z-float(d.xpos[bid,2]),events=events,
                scope='Gravity/contact-only fixture, actual CAD T-bar extents, conservative lower envelope; provisional loaded mass. Not Nori arm or loose slide retention validation.')

if __name__=='__main__':
    results=[run(force=f,render=f==2) for f in (.8,2,4)]+[run(force=.00001),run(yaw_error=math.pi/2)]
    (ROOT/'rack24_contact_check.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
    assert results[1]['passed'],'Nominal fixture lift/hold/release regression failed'
    assert not results[3]['passed'] and not results[4]['passed'],'Weak or misaligned grip must not pass'
