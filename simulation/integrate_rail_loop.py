"""Pin the tested station and compose its live physics with the whole lab.

The rest of the lab is a held visual context, not certified coupled dynamics.
The station has real free-body contacts and a bounded copy of its actual bench.
No change is made to the source experiment, its CAD, or its evidence files.
"""
import copy
import hashlib
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from build_scene import ROOT, vec
from nori_posture import initialize_bench_view

SOURCE=Path(r'C:/Users/Owner/.codex/visualizations/2026/08/07/019fde06-5766-7fe2-8736-d06324be20d8/rack_sim')
REFERENCE=ROOT/'rail_loop_reference'
MODEL=ROOT/'exhist_rail_loop.xml'
PREFIX='lab_context_'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot():
    summary=json.loads((SOURCE/'rail_loop_summary.json').read_text())
    result=json.loads((SOURCE/'rail_loop.json').read_text())
    route=list(range(1,12))+[1]
    assert summary['passed'] and result['passed'] and not result['joint_limit_violations']
    assert [(r['source'],r['destination']) for r in result['completed_transfers']]==list(zip(route[:-1],route[1:]))
    for name,digest in summary['source_hashes'].items():
        assert sha(SOURCE/name)==digest, 'Changed source evidence: '+name
    names=['rail_loop.xml','rail_loop_controller.py','rail_loop.py','reference_trial.py',
           'multi_container_scene.py','build_scene.py','rail_loop_layout.json','rail_loop_summary.json',
           'rail_loop.json','rail_loop_trace.json','RAIL_LOOP_RESULTS.md']
    REFERENCE.mkdir(exist_ok=True)
    hashes={}
    for name in names:
        shutil.copy2(SOURCE/name,REFERENCE/name)
        hashes[name]=sha(REFERENCE/name)
    # URDF IK and MuJoCo now resolve a local pinned mesh snapshot.
    for file in (SOURCE/'source/robot').rglob('*'):
        if not file.is_file():continue
        rel=file.relative_to(SOURCE);target=REFERENCE/rel
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(file,target)
        hashes[rel.as_posix()]=sha(target)
    native=ET.parse(REFERENCE/'rail_loop.xml')
    for e in native.getroot().iter():
        if not e.get('file'):continue
        file=Path(e.get('file'))
        rel=file.resolve().relative_to(SOURCE.resolve())
        assert (REFERENCE/rel).is_file(),rel
        e.set('file',str((REFERENCE/rel).resolve()))
    ET.indent(native)
    native.write(REFERENCE/'station.xml',encoding='utf-8',xml_declaration=True)
    hashes['station.xml']=sha(REFERENCE/'station.xml')
    (REFERENCE/'snapshot.json').write_text(json.dumps(dict(source=str(SOURCE),sha256=hashes),indent=2))
    return json.loads((REFERENCE/'rail_loop_layout.json').read_text())


def build_context(layout):
    lab=ET.parse(ROOT/'exhist_operational.xml').getroot()
    lm=mujoco.MjModel.from_xml_path(str(ROOT/'exhist_operational.xml'));ld=mujoco.MjData(lm)
    initialize_bench_view(lm,ld)
    ld.qpos[lm.joint('base_yaw').qposadr[0]]=np.pi/2
    # Unscheduled preview batches must not appear floating at staging edges.
    ld.mocap_pos[:]=[0,0,-10]
    mujoco.mj_forward(lm,ld)
    root=ET.parse(REFERENCE/'station.xml').getroot();root.set('model','ExHist Labs - live LeHisto rail loop / other equipment held')
    R=np.asarray(layout['world_rotation']);t=np.asarray(layout['world_translation'])
    q=np.zeros(4);mujoco.mju_mat2Quat(q,R.T.ravel())
    wrapper=ET.Element('body',name='full_lab_context',pos=vec(-R.T@t),quat=vec(q))
    def freeze(body):
        if body.tag=='body':
            bid=lm.body(body.get('name')).id;pid=lm.body_parentid[bid]
            rp=ld.xmat[pid].reshape(3,3)
            localp=rp.T@(ld.xpos[bid]-ld.xpos[pid])
            localr=rp.T@ld.xmat[bid].reshape(3,3);localq=np.zeros(4);mujoco.mju_mat2Quat(localq,localr.ravel())
            for k in ('euler','axisangle','xyaxes','zaxis','mocap'):body.attrib.pop(k,None)
            body.set('pos',vec(localp));body.set('quat',vec(localq))
        for child in list(body):
            if child.tag in ('joint','freejoint','inertial','site','camera','light'):
                body.remove(child)
            else:freeze(child)
        for k in ('name','mesh','material','texture'):
            if body.get(k):body.set(k,PREFIX+body.get(k))
        if body.tag=='geom':
            body.set('contype','0');body.set('conaffinity','0');body.set('density','0');body.attrib.pop('mass',None)
    for child in lab.find('worldbody'):
        name=child.get('name','')
        if child.tag not in ('body','geom'):continue
        if name=='special_lehisto' or name.startswith(('special_staining_jar_','B01_','B02_','B03_','B04_')):continue
        child=copy.deepcopy(child);freeze(child);wrapper.append(child)
    assets=root.find('asset')
    for a in lab.find('asset'):
        a=copy.deepcopy(a)
        for k in ('name','texture','material'):
            if a.get(k):a.set(k,PREFIX+a.get(k))
        if a.get('file'):a.set('file',str((ROOT/a.get('file')).resolve()))
        assets.append(a)
    world=root.find('worldbody');world.append(wrapper)
    # Replace the benchmark's infinite bench-height plane with this actual bench.
    floor=world.find("geom[@name='floor']");world.remove(floor)
    support=ET.SubElement(world,'body',name='actual_lab_bench_contact',pos=vec(-R.T@t),quat=vec(q))
    for g in lab.find("worldbody/body[@name='special_table']").findall('geom'):
        g=copy.deepcopy(g);g.set('name','loop_contact_'+g.get('name'))
        g.set('rgba','0 0 0 0');g.set('group','3');g.set('contype','1');g.set('conaffinity','1')
        g.attrib.pop('material',None);support.append(g)
    ET.SubElement(world,'geom',name='actual_room_floor_contact',type='plane',pos=vec([0,0,-t[2]]),size='10 10 .01',rgba='0 0 0 0',group='3')
    # Preserve original physics settings. Camera range changes are visual-only.
    vis=root.find('visual');vg=vis.find('global');vg.set('offwidth','1600');vg.set('offheight','900')
    vm=vis.find('map')
    if vm is None:vm=ET.SubElement(vis,'map')
    vm.set('znear','.0002');vm.set('zfar','50')
    if root.find('statistic') is None:ET.SubElement(root,'statistic',extent='2')
    ET.indent(root);ET.ElementTree(root).write(MODEL,encoding='utf-8',xml_declaration=True)
    return root


def integrate():
    layout=snapshot()
    (ROOT/'special_rail_loop_layout.json').write_text(json.dumps(layout,indent=2))
    from special_layout import apply
    tree=ET.parse(ROOT/'exhist_operational.xml');apply(tree.getroot(),layout);ET.indent(tree)
    tree.write(ROOT/'exhist_operational.xml',encoding='utf-8',xml_declaration=True)
    from special_layout import apply_static
    static=ET.parse(ROOT/'exhist.xml');apply_static(static.getroot());ET.indent(static)
    static.write(ROOT/'exhist.xml',encoding='utf-8',xml_declaration=True)
    build_context(layout)
    report=dict(model=MODEL.name,canonical_layout='exhist_operational.xml',source=str(SOURCE),
                route=layout['route'],rail_limits_m=layout['rail_limits_m'],riser_height_m=.0254,
                model_sha256=sha(MODEL),lab_sha256=sha(ROOT/'exhist_operational.xml'),
                source_manifest_sha256=sha(REFERENCE/'snapshot.json'),
                scope='Live contact-driven LeHisto station composed with all lab visual geometry. Other stations are held context, not coupled or collision-certified. Actual bounded bench replaces benchmark plane. Native source experiment and historical first-pass video are preserved.')
    (ROOT/'rail_loop_integration.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    return report


if __name__=='__main__':integrate()
