"""Import the separately validated eleven-bath layout into the active lab.

Geometry integration is not full-lab collision or controller certification.
The old first-pass film retains its historical straight-row choreography.
"""
import copy,json,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np,mujoco
from build_scene import ROOT,el,vec
from jar_riser import ensure_riser,RACK_SEAT_Z

SOURCE=Path('C:/Users/Owner/.codex/visualizations/2026/08/07/019fde06-5766-7fe2-8736-d06324be20d8/rack_sim')
LEGACY_SNAPSHOT=ROOT/'special_eleven_layout.json'
RAIL_SNAPSHOT=ROOT/'special_rail_loop_layout.json'
SNAPSHOT=RAIL_SNAPSHOT if RAIL_SNAPSHOT.exists() else LEGACY_SNAPSHOT

def apply(root,layout=None):
    layout=layout or json.loads(SNAPSHOT.read_text());world=root.find('worldbody')
    template=world.find("body[@name='special_staining_jar_1']")
    rotation=np.asarray(layout['world_rotation']);rq=np.zeros(4);mujoco.mju_mat2Quat(rq,rotation.ravel())
    rig=world.find("body[@name='special_lehisto']")
    # Preserve the actual CAD base height, matching the accepted shoulder frame.
    p=np.fromstring(rig.get('pos'),sep=' ');p[:2]=[1.1725,.54];rig.set('pos',vec(p))
    for s in layout['stations']:
        name='special_staining_jar_'+str(s['id']);body=world.find(f"body[@name='{name}']")
        if body is None:
            body=copy.deepcopy(template)
            for node in body.iter():
                if node.get('name'):node.set('name',node.get('name').replace('special_staining_jar_1',name))
            world.append(body)
        q=np.zeros(4);mujoco.mju_mulQuat(q,rq,np.asarray(s['quat_wxyz']))
        body.set('pos',vec(s['world_position_m']));body.set('quat',vec(q))
        body.find('geom').set('rgba','.55 .68 .73 1');ensure_riser(root,body)
        site=world.find(f"site[@name='special_bath_target_{s['id']}']")
        if site is None:site=el(world,'site',name=f"special_bath_target_{s['id']}",size='.002',rgba='0 0 0 0')
        site.set('pos',vec([*s['world_position_m'][:2],RACK_SEAT_Z]));site.set('quat',vec(q))

def apply_static(root):
    """Keep the non-animated CAD inspector aligned with the selected layout."""
    if not RAIL_SNAPSHOT.exists():return
    world=root.find('worldbody')
    for child in list(world):
        if child.get('name','').startswith(('bucket_','reagent_')) or child.get('name')=='special_dropoff':
            world.remove(child)
    if world.find("body[@name='special_staining_jar_1']") is None:
        from cad_jar import add_jar
        add_jar(root,'special_staining_jar_1',[1.0,.8,.8055])
    apply(root,json.loads(RAIL_SNAPSHOT.read_text()))

def integrate():
    names=['lab_eleven_layout.json','lab_eleven.json','lab_eleven_reverse.json','lab_eleven_summary.json','LAB_ELEVEN_RESULTS.md']
    evidence={n:hashlib.sha256((SOURCE/n).read_bytes()).hexdigest() for n in names}
    for n in ('lab_eleven.json','lab_eleven_reverse.json'):
        result=json.loads((SOURCE/n).read_text())
        if not result['passed'] or len(result['completed_transfers'])!=10 or result['joint_limit_violations']:raise RuntimeError('Invalid source route evidence: '+n)
    LEGACY_SNAPSHOT.write_text((SOURCE/'lab_eleven_layout.json').read_text())
    tree=ET.parse(ROOT/'exhist_operational.xml');apply(tree.getroot());ET.indent(tree)
    tree.write(ROOT/'exhist_operational.xml',encoding='utf-8',xml_declaration=True)
    # A separate evidence viewer uses the exact measured-state trace and original
    # physical rig, avoiding an unverified mapping to the old CAD animation arm.
    report=dict(source_directory=str(SOURCE),source_sha256=evidence,active_model='exhist_operational.xml',
        historical_model='exhist_first_pass.xml',baths=11,rail_parked_m=0.,riser_height_m=.0254,
        standalone_transfers_passed=20,full_lab_collision_certified=False,
        reference_video=str(SOURCE/'lab_eleven_review.mp4'),
        scope='Active lab layout updated. Exact physical reference motion is available in the linked evidence video; native full-lab controller adapter and Nori coordination remain guarded.')
    (ROOT/'special_eleven_integration.json').write_text(json.dumps(report,indent=2));return report

if __name__=='__main__':print(json.dumps(integrate(),indent=2))
