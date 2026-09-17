"""Contact-only checks of the ORIGINAL grooved fingers; no added pads.
Handle coupon tests the 20 mm top bar + 9 mm neck, not an entire loaded rack.
"""
import json,math
import numpy as np
import mujoco
import xml.etree.ElementTree as ET
from contact_bench import build as fixture
from gripper_physics import add_contacts,build_mass_model
from build_scene import ROOT,el,vec

def build(kind='slide',offset=0):
    root=fixture(offset=offset,padded=False)
    mount=root.find(".//body[@name='actuated_test_fixture']")
    mount.set('pos','0 .0933 '+str(.2031885 if kind=='slide' else .1711885))
    for side,index in [('a',14),('b',15)]:
        parent=root.find(f".//geom[@name='lehisto_part_{index}_contact']/..")
        parent.remove(parent.find(f"geom[@name='lehisto_part_{index}_contact']"))
        add_contacts(root,parent,index,'groove_'+side)
    masses=build_mass_model()['body_inertials']
    for name,v in masses.items():
        if name=='right_wrist_roll_link':continue
        b=root.find(f".//body[@name='{name}']");b.remove(b.find('inertial'))
        el(b,'inertial',mass=v['mass'],pos=vec(v['pos']),fullinertia=vec(v['fullinertia']))
    if kind=='handle':
        payload=root.find(".//body[@name='payload']");payload.set('pos',vec([offset,0,.075]))
        g=payload.find('geom');g.set('size','.01 .001 .00375');g.set('mass','.003');g.set('rgba','.95 .65 .25 1')
        el(payload,'geom',name='handle_neck',type='box',pos='0 0 -.01375',size='.0045 .001 .01',mass='.002',rgba='.95 .65 .25 1')
        support=root.find(".//geom[@name='support']");support.set('pos','0 0 .025625');support.set('size','.003 .01 .025625')
    return root

def run(kind,offset=0,force=.8,render=False):
    root=build(kind,offset);root.find("actuator/position[@name='jaw_servo']").set('forcerange',vec([-force,force]))
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    bid=m.body('payload').id;fid=m.body('actuated_test_fixture').id
    geoms={g for g in range(m.ngeom) if m.geom_bodyid[g]==bid}
    jaws={g:('a' if m.geom(g).name.startswith('groove_a_') else 'b') for g in range(m.ngeom) if m.geom(g).name.startswith('groove_')}
    start=.0625 if kind=='slide' else .075;phase='SETTLE';phase_start=0;dwell=0;lift=0.;grasp_local=None
    events=[];qualified=False;hold_samples=0;both_samples=0;peak=0.;slip=0.;release_z=None;snapshot=None
    for i in range(10000):
        t=d.time
        if phase=='SETTLE' and t>=.3:
            phase='CLOSE' if abs(offset)<=.002 else 'POSE_REJECTED';phase_start=t;events.append([round(t,3),phase])
        if phase=='LIFT':lift=.04*min(1,t-phase_start)
        close=-.0355 if phase in ('CLOSE','LIFT','HOLD','GRASP_LOST') else 0
        d.ctrl[:]=[lift,close];mujoco.mj_step(m,d)
        forces={'a':0.,'b':0.}
        for ci,c in enumerate(d.contact):
            other=c.geom2 if c.geom1 in geoms else c.geom1 if c.geom2 in geoms else -1
            if other in jaws:
                wrench=np.zeros(6);mujoco.mj_contactForce(m,d,ci,wrench);forces[jaws[other]]+=max(0,float(wrench[0]))
        both=min(forces.values())>.005;peak=max(peak,float(d.xpos[bid,2]-start))
        if phase=='CLOSE':
            dwell=dwell+1 if both else 0
            if dwell>=120:
                phase='LIFT';phase_start=t;grasp_local=d.xpos[bid]-d.xpos[fid];events.append([round(t,3),phase])
            elif t-phase_start>5:phase='NO_GRASP';events.append([round(t,3),phase])
        elif phase in ('LIFT','HOLD'):
            slip=max(slip,float(np.linalg.norm(d.xpos[bid]-d.xpos[fid]-grasp_local)))
            if slip>.002:phase='GRASP_LOST';events.append([round(t,3),phase]);continue
            if phase=='LIFT' and t-phase_start>1.3:
                phase='HOLD';phase_start=t;events.append([round(t,3),phase])
            elif phase=='HOLD':
                hold_samples+=1;both_samples+=int(both)
                if render and snapshot is None:
                    with mujoco.Renderer(m,height=720,width=960) as renderer:
                        cam=mujoco.MjvCamera();cam.lookat[:]=[0,0,.15];cam.distance=.38;cam.azimuth=140;cam.elevation=-12
                        renderer.update_scene(d,cam);snapshot=renderer.render().copy()
                if t-phase_start>=2:
                    qualified=both_samples/hold_samples>.99 and peak>.03
                    phase='DROP_TEST';release_z=float(d.xpos[bid,2]);events.append([round(t,3),phase])
    if snapshot is not None:
        from PIL import Image,ImageDraw,ImageFont
        im=Image.fromarray(snapshot);draw=ImageDraw.Draw(im);draw.rectangle([0,0,960,55],fill='#10283c')
        draw.text((15,14),f'Original {kind} groove | contact-only test fixture | no pads',fill='white',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',22))
        im.save(ROOT/f'groove_{kind}.png')
    return dict(kind=kind,offset_m=offset,force_limit_N=force,passed=bool(qualified),peak_lift_m=peak,
        relative_slip_m=slip,hold_bilateral_fraction=both_samples/max(1,hold_samples),events=events,
        release_drop_m=None if release_z is None else release_z-float(d.xpos[bid,2]),
        scope='original groove contact on isolated fixture; no Nori arm or loaded-rack validation')

if __name__=='__main__':
    results=[run('slide',render=True),run('handle',render=True),run('slide',offset=.085),run('handle',force=.00001)]
    (ROOT/'groove_bench_validation.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
    assert results[0]['passed'] and results[1]['passed']
    assert not results[2]['passed'] and not results[3]['passed']
    scene=build();ET.indent(scene);ET.ElementTree(scene).write(ROOT/'groove_bench.xml',encoding='utf-8',xml_declaration=True)
