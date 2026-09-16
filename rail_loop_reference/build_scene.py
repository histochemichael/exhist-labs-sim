"""Offline, isolated CAD-derived rack benchmark. No hardware interfaces."""
from pathlib import Path
import hashlib,json,shutil
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parent
ROBOT=Path(r'C:\Users\Owner\slide_gripper_mujoco')
CAD=Path(r'C:\Users\Owner\.codex\visualizations\2026\09\13\01a09cad-d87f-7ad2-82ec-ff5b9e1d79fd\exhist-mujoco\assets')
JAR=np.array([.01,-.23,-.0642]); HEIGHT=.0254; SEAT=JAR[2]-.0015+HEIGHT
GRASP=np.array([*JAR[:2],SEAT+.0925])
HANDLE_WIDTH_M=.020  # Original CAD width; artificial widening was not the fix.
def vec(x):return ' '.join(f'{v:.10g}' for v in x)
def add(p,tag,**kw):return ET.SubElement(p,tag,{k:str(v) for k,v in kw.items()})
def box(p,name,pos,size,**kw):return add(p,'geom',name=name,type='box',pos=vec(pos),size=vec(size),**kw)
def build():
    sources=list(ROBOT.glob('*.xml'))+list((ROBOT/'meshes').rglob('*.stl'))+[ROBOT/'so101_rail.urdf']+[CAD/n for n in ['rack24.json','rack24_jar.json','rack24_riser.json']]
    records=[]
    for src in sources:
        rel=Path('cad')/src.name if src.parent==CAD else Path('robot')/src.relative_to(ROBOT)
        dst=ROOT/'source'/rel;dst.parent.mkdir(parents=True,exist_ok=True)
        if not dst.exists():shutil.copy2(src,dst)
        records.append(dict(source=str(src),snapshot=str(rel),sha256=hashlib.sha256(dst.read_bytes()).hexdigest()))
    (ROOT/'sources.json').write_text(json.dumps(records,indent=2))
    root=ET.parse(ROOT/'source/robot/so101_slide_gripper.xml').getroot()
    def expand(parent):
        for node in list(parent):
            if node.tag=='include':
                inc=ET.parse(ROOT/'source/robot'/node.attrib['file']).getroot();idx=list(parent).index(node);parent.remove(node)
                for child in list(inc):parent.insert(idx,child);idx+=1;expand(child)
            else:expand(node)
    expand(root)
    for mesh in root.findall('.//mesh'):
        if mesh.get('file'):mesh.set('file',(ROOT/'source/robot'/mesh.get('file')).as_posix())
    root.find('option').set('timestep','.001');root.find('option').set('iterations','100');root.find('option').set('cone','elliptic')
    flag=root.find('option/flag')
    if flag is None:flag=add(root.find('option'),'flag')
    flag.set('multiccd','enable')
    asset=root.find('asset');world=root.find('worldbody')
    for name,file in [('jar_mesh','rack24_jar.json'),('rack_mesh','rack24.json'),('riser_mesh','rack24_riser.json')]:
        part=json.loads((ROOT/'source/cad'/file).read_text())['parts'][0];v=np.array(part['vertices']).reshape(-1,3)
        if name=='rack_mesh':
            v[v[:,2]>=.0894,1]*=HANDLE_WIDTH_M/.020
            v[:,2]+=.0015
        add(asset,'mesh',name=name,vertex=vec(v.ravel()),face=' '.join(map(str,part['triangles'])),inertia='shell')
    quat='0.7071067812 0 0 0.7071067812'
    jar=add(world,'body',name='jar',pos=vec(JAR),quat=quat);add(jar,'freejoint',name='jar_free')
    add(jar,'inertial',mass='.1',pos='0 0 .04',diaginertia='.00011 .00016 .00012')
    add(jar,'geom',name='jar_visual',type='mesh',mesh='jar_mesh',rgba='.3 .65 .65 .28',contype='0',conaffinity='0',density='0',group='1')
    pieces=[('floor',[0,0,-.0035],[.052,.025,.002])]
    for s in (-1,1):
        pieces.extend([(f'xwall{s}',[s*.0505,0,.04365],[.0015,.025,.04515]),(f'ywall{s}',[0,s*.0235,.04365],[.049,.0015,.04515]),(f'xrim{s}',[s*.05125,0,.0903],[.00225,.0265,.0015]),(f'yrim{s}',[0,s*.02425,.0903],[.049,.00225,.0015])])
        for t in (-1,1):pieces.append((f'corner{s}_{t}',[s*.0465,t*.0195,.04515],[.0025,.0025,.04665]))
    for name,pos,size in pieces:box(jar,'jar_'+name,pos,size,density='0',group='3',rgba='.3 .65 .65 .25',friction='.5 .005 .0001',solref='.004 1')
    riser=add(world,'body',name='riser',pos=vec(JAR+[0,0,-.0015]),quat=quat);add(riser,'freejoint',name='riser_free')
    add(riser,'geom',name='riser_support',type='mesh',mesh='riser_mesh',mass='.120',rgba='.7 .75 .4 1',friction='.5 .005 .0001',solref='.004 1')
    rack=add(world,'body',name='rack',pos=vec([*JAR[:2],SEAT+.0005]),quat=quat);add(rack,'freejoint',name='rack_free')
    add(rack,'inertial',mass='.145',pos='0 0 .04',diaginertia='.00015 .00008 .00012')
    add(rack,'geom',name='rack_visual',type='mesh',mesh='rack_mesh',rgba='.68 .70 .73 1',contype='0',conaffinity='0',density='0',group='1')
    for name,pos,size in [('bar',[0,0,.0925],[.001,HANDLE_WIDTH_M/2,.0015]),('neck',[0,0,.080],[.001,.0045,.011]),('lower',[0,0,.03425],[.044,.016647,.03425])]:
        box(rack,'rack_'+name,pos,size,density='0',group='3',rgba='.8 .3 .3 .15',friction='.6 .005 .0001',condim='4',solref='.004 1')
    # Nominal 24-slide load, fixed for initial insertion-clearance tests.
    for k,x in enumerate(np.linspace(-.037125,.037125,24)):
        box(rack,f'slide_{k}',[x,0,.0405],[.0005,.0125,.0375],density='0',rgba='.7 .88 .96 .55',contype='0',conaffinity='0',group='1')
    add(rack,'site',name='rack_grasp',pos='0 0 .0925',size='.0015',rgba='1 .3 .1 1')
    from groove_contacts import update
    update(root)
    from container_contacts import update as update_container
    update_container(root)
    from support_contacts import update as update_support
    update_support(root)
    from camera_observations import configure_cameras
    configure_cameras(root)
    ET.indent(root);ET.ElementTree(root).write(ROOT/'scene.xml',encoding='utf-8',xml_declaration=True)
    manifest=dict(scope='Offline physics benchmark, NOT a trained policy',support_height_m=HEIGHT,jar_position_m=JAR.tolist(),rack_seated_origin_z_m=SEAT,nominal_grasp_m=GRASP.tolist(),slides=24,no_payload_attachment=True,no_hardware_interfaces=True,assumptions=[
      'Loaded rack 0.145 kg, jar 0.100 kg, riser 0.120 kg are provisional.',
      'Slides are fixed payload with nominal 25x75x1 mm visuals and unverified slot locations; no loose-slide retention or fracture model.',
      'Lower rack collision is a conservative solid envelope; top handle uses original 20 mm CAD width in visual and collision geometry.',
      'Jar inner corners use the CAD 5 mm radius with 10-segment arcs; rim bevels, liquid drag and buoyancy not modeled.',
      'Robot dynamics and gripper friction inherited, not system-identified.',
      'Jar position is a simulation fixture, not a measured hardware transform.'])
    manifest['handle_width_m']=HANDLE_WIDTH_M
    manifest['contact_solver']='MuJoCo multiccd enabled for flat support and groove contacts'
    manifest['reference_grasp_height_offset_m']=.002
    manifest['camera_observations']={'observation.images.gripper':'gripper', 'observation.images.scene':'scene'}
    manifest['camera_calibration']='CAD-derived mounts; provisional FOV and optical orientation, not calibrated to real cameras.'
    (ROOT/'manifest.json').write_text(json.dumps(manifest,indent=2));print(ROOT/'scene.xml')
if __name__=='__main__':build()
