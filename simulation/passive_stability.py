"""Gravity/contact soak of the lab jars, inserts and a loaded rack.

Copies the actual scene bodies and table. No robot, actuator, weld, mocap,
sleeping-body trick or runtime pose reset. This is support validation only.
"""
import copy,json,hashlib
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from build_scene import ROOT,el,vec
from test_jar_riser import fixture as rack_fixture

def build():
    source=ET.parse(ROOT/'exhist_operational.xml').getroot()
    root=ET.Element('mujoco',model='Live passive special-stain stability soak')
    el(root,'compiler',angle='radian')
    el(root,'option',timestep='.001',integrator='implicitfast',iterations='100',gravity='0 0 -9.81')
    assets=el(root,'asset');world=el(root,'worldbody')
    for b in source.find('worldbody'):
        if b.get('name','').startswith(('special_staining_jar_','qc_cassette_')) or b.get('name') in ('special_table','sendout_table'):world.append(copy.deepcopy(b))
    needed={g.get('mesh') for g in world.findall('.//geom') if g.get('mesh')}
    for a in source.find('asset'):
        if a.get('name') not in needed:continue
        a=copy.deepcopy(a)
        if a.get('file'):a.set('file',str((ROOT/a.get('file')).resolve()))
        assets.append(a)
    rackroot=rack_fixture();rack=copy.deepcopy(rackroot.find("worldbody/body[@name='test_rack']"))
    jar=world.find("body[@name='special_staining_jar_6']")
    p=np.fromstring(jar.get('pos'),sep=' ');p[2]+=-.0015+.0254
    rack.set('pos',vec(p));rack.set('quat',jar.get('quat','1 0 0 0'));world.append(rack)
    assets.append(copy.deepcopy(rackroot.find("asset/mesh[@name='actual_rack']")))
    # Table texture is cosmetic; keep the copied physical tabletop unchanged.
    for g in world.findall('.//geom'):g.attrib.pop('material',None)
    return root

def run(seconds=30.):
    root=build();m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    ids=[i for i in range(1,m.nbody) if m.body_jntnum[i] and m.jnt_type[m.body_jntadr[i]]==mujoco.mjtJoint.mjJNT_FREE]
    positions=[];rotations=[];speeds=[]
    for step in range(round(seconds/m.opt.timestep)):
        mujoco.mj_step(m,d)
        if step%10==0 and d.time>=2.:
            positions.append(d.xpos[ids].copy());rotations.append(d.xquat[ids].copy());speeds.append(float(np.max(np.abs(d.qvel))))
    pos=np.asarray(positions);quat=np.asarray(rotations);rows=[]
    for k,i in enumerate(ids):
        angles=2*np.arccos(np.clip(np.abs(quat[:,k]@quat[0,k]),0,1))
        rows.append(dict(body=m.body(i).name,translation_span_m=float(np.linalg.norm(np.ptp(pos[:,k],axis=0))),
            rotation_excursion_rad=float(angles.max()),final_z_m=float(pos[-1,k,2])))
    checks=dict(no_actuators=m.nu==0,no_attachments=not root.findall('.//weld') and not root.findall(".//body[@mocap='true']"),
        finite=bool(np.isfinite(d.qpos).all()),translation_stable=all(r['translation_span_m']<.0001 for r in rows),
        rotation_stable=all(r['rotation_excursion_rad']<.001 for r in rows),quiet_velocity=max(speeds)<.001,
        rack_supported=abs(float(d.body('test_rack').xpos[2])-.8294)<.0005)
    r=dict(passed=all(checks.values()),checks=checks,sim_seconds=float(d.time),settle_seconds=2.,sample_hz=100,bodies=rows,
        max_generalized_speed=max(speeds),scene_sha256=hashlib.sha256((ROOT/'exhist_operational.xml').read_bytes()).hexdigest(),
        scope='11 free jars + 11 free 25.4 mm inserts + one 145 g loaded-rack envelope + 8 free QC blocks on copied lab tables. QC blocks use provisional 20 g masses and support envelopes. Dry nominal support only; loose slides, folders and robot contacts not certified.')
    (ROOT/'passive_stability_validation.json').write_text(json.dumps(r,indent=2));print(json.dumps(r),flush=True)
    return r

if __name__=='__main__':
    if not run()['passed']:raise SystemExit(1)
