"""Pinned 45-degree QC fixture: fixed layout, not Nori handling validation."""
import argparse, copy, hashlib, json, time
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'source/qc-folder45'
BASE=ROOT/'exhist_rail_loop.xml'
MODEL=ROOT/'exhist_rail_loop_qc45.xml'
MANIFEST=ROOT/'qc_folder45_layout.json'
PREFIX='lab_context_qc45_'
SOURCE_FRAME_XY=np.array([2.364,.700])

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def vec(x):return ' '.join(f'{float(v):.12g}' for v in x)

def strip_addition(root):
    root=copy.deepcopy(root);root.set('model',ET.parse(BASE).getroot().get('model'))
    for parent in root.iter():
        for child in list(parent):
            if child.get('name','').startswith(PREFIX):parent.remove(child)
    for e in root.iter():e.text=None;e.tail=None
    return ET.tostring(root)

def verify_variant():
    record=json.loads(MANIFEST.read_text())
    for name,digest in record['sha256'].items():
        if sha(ROOT/name)!=digest:raise RuntimeError('QC dependency changed: '+name)
    if strip_addition(ET.parse(MODEL).getroot())!=strip_addition(ET.parse(BASE).getroot()):
        raise RuntimeError('QC composition changed something outside the fixed fixture')
    return MODEL

def build():
    spec=json.loads((SOURCE/'folder_stand_45.json').read_text())
    source=ET.parse(SOURCE/'folder_slide_45.xml').getroot()
    folder=copy.deepcopy(source.find("worldbody/body[@name='folder_fixture']"))
    np.testing.assert_allclose(np.fromstring(folder.get('pos'),sep=' '),spec['folder_center_m'],atol=1e-8)
    for part in spec['parts']:
        geom=source.find("worldbody/geom[@name='stand45_"+part['name']+"']")
        np.testing.assert_allclose(np.fromstring(geom.get('pos'),sep=' '),part['center_m'],atol=1e-8)
        np.testing.assert_allclose(np.fromstring(geom.get('size'),sep=' '),part['half_size_m'],atol=1e-8)
    # Room context is held, so retain exact CAD visuals, not unused contact shells.
    for child in list(folder):
        if child.get('name','').startswith('folder_shell_'):folder.remove(child)
    meshes={g.get('mesh') for g in folder.iter('geom') if g.get('mesh')}
    root=ET.parse(BASE).getroot();root.set('model','ExHist Labs - live special stain + fixed 45-degree QC cradle')
    world=root.find("worldbody/body[@name='full_lab_context']")
    translation=np.r_[SOURCE_FRAME_XY,.8-spec['bench_z_m']]
    center_world=translation+np.array(spec['folder_center_m'])
    fixture=ET.SubElement(world,'body',name=PREFIX+'fixture',pos=vec(translation));fixture.append(folder)
    for g in source.findall('worldbody/geom'):
        if g.get('name','').startswith('stand45_'):fixture.append(copy.deepcopy(g))
    for e in fixture.iter():
        if e is fixture:continue
        for k in ('name','mesh'):
            if e.get(k):e.set(k,PREFIX+e.get(k))
        if e.tag=='geom':e.attrib.update(contype='0',conaffinity='0',density='0',group='1')
    for mesh in source.findall('asset/mesh'):
        if mesh.get('name') in meshes:
            mesh=copy.deepcopy(mesh);mesh.set('name',PREFIX+mesh.get('name'));root.find('asset').append(mesh)
    corridors=[]
    for side,sign in [('left',-1),('right',1)]:
        center=center_world+[sign*.225,-.06,-.015];half=[.04,.17,.075]
        ET.SubElement(fixture,'site',name=PREFIX+side+'_access',type='box',pos=vec(center-translation),size=vec(half),rgba='.1 .65 .9 .18',group='5')
        corridors.append(dict(side=side,center_m=center.tolist(),half_size_m=half,status='Reserved volume only; no gripper fit or reach validation'))
    ET.indent(root);ET.ElementTree(root).write(MODEL,encoding='utf-8',xml_declaration=True)
    # Portable standalone inspection asset, using the STL tabletop-z=0 convention.
    standalone=ET.Element('mujoco',model='QC folder cradle 45 degrees - fixed inspection only')
    ET.SubElement(standalone,'compiler',angle='radian')
    vis=ET.SubElement(standalone,'visual');ET.SubElement(vis,'global',offwidth='1400',offheight='900')
    assets=ET.SubElement(standalone,'asset')
    for mesh in root.findall('asset/mesh'):
        if mesh.get('name','').startswith(PREFIX):assets.append(copy.deepcopy(mesh))
    wb=ET.SubElement(standalone,'worldbody');f=copy.deepcopy(fixture);f.set('pos',vec([0,0,-spec['bench_z_m']]));wb.append(f)
    out=ROOT/'models/assets';out.mkdir(parents=True,exist_ok=True)
    ET.indent(standalone);ET.ElementTree(standalone).write(out/'qc_folder45.xml',encoding='utf-8',xml_declaration=True)
    paths=[BASE,MODEL,SOURCE/'folder_slide_45.xml',SOURCE/'folder_stand_45.json',SOURCE/'slide_folder.json']
    record=dict(model=MODEL.name,base_model=BASE.name,units='m',bench_top_m=.8,folder_center_world_m=center_world.tolist(),source_to_lab_translation_m=translation.tolist(),tilt_above_table_deg=45,cover_flaps='Held 90 degrees behind tray',proposed_handler='Existing Nori left arm; no second Nori added',reserved_access=corridors,status='Fixed layout only. No Nori grip, reach, passive docking, slide placement, flap closing or loaded transport validation.',sha256={p.relative_to(ROOT).as_posix():sha(p) for p in paths})
    MANIFEST.write_text(json.dumps(record,indent=2)+'\n');verify_variant();return record

def export_stl():
    """Export the pinned specification, not a concurrently edited source file."""
    import trimesh
    spec=json.loads((SOURCE/'folder_stand_45.json').read_text());out=ROOT/'cad/stl';out.mkdir(parents=True,exist_ok=True)
    meshes=[]
    for part in spec['parts']:
        T=np.eye(4);T[:3,:3]=part['rotation'];T[:3,3]=np.array(part['center_m'])-[0,0,spec['bench_z_m']]
        mesh=trimesh.creation.box(extents=2*np.array(part['half_size_m']),transform=T);mesh.apply_scale(1000)
        mesh.export(out/('Folder-Stand-45-'+part['name']+'.stl'));meshes.append(mesh)
    trimesh.util.concatenate(meshes).export(out/'Folder-Stand-45-Assembly.stl')

def vertices(m,d,i):
    if m.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH:
        j=m.geom_dataid[i];v=m.mesh_vert[m.mesh_vertadr[j]:m.mesh_vertadr[j]+m.mesh_vertnum[j]]
    elif m.geom_type[i]==mujoco.mjtGeom.mjGEOM_BOX:
        v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*m.geom_size[i]
    else:return None
    return v@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]

def bounds(m,d,ids,R,t):
    vv=[vertices(m,d,i) for i in ids];v=np.concatenate([v for v in vv if v is not None])@R.T+t
    return np.stack([v.min(0),v.max(0)])

def gap(a,b):return float(np.linalg.norm(np.maximum(np.maximum(a[0]-b[1],b[0]-a[1]),0)))

def check():
    verify_variant();m=mujoco.MjModel.from_xml_path(str(MODEL));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    base=mujoco.MjModel.from_xml_path(str(BASE))
    layout=json.loads((ROOT/'rail_loop_reference/rail_loop_layout.json').read_text());R=np.array(layout['world_rotation']);t=np.array(layout['world_translation'])
    ids=[i for i in range(m.ngeom) if m.geom(i).name.startswith(PREFIX)]
    b=bounds(m,d,ids,R,t);bench=bounds(m,d,[m.geom('lab_context_sendout_top').id],R,t)
    folder=bounds(m,d,[i for i in ids if 'folder_visual' in m.geom(i).name],R,t)
    feet=bounds(m,d,[i for i in ids if 'stand45_foot_' in m.geom(i).name],R,t)
    checks=dict(compiles=True,only_added_fixed_context=True,unchanged_degrees_of_freedom=(m.nq,m.nv,m.nu,m.neq)==(base.nq,base.nv,base.nu,base.neq),no_new_contacts=all(m.geom_contype[i]==0 and m.geom_conaffinity[i]==0 for i in ids),feet_on_bench=abs(feet[0,2]-bench[1,2])<1e-7,fixture_inside_bench=bool(np.all(b[0,:2]>bench[0,:2]) and np.all(b[1,:2]<bench[1,:2])),folder_above_bench=bool(folder[0,2]>bench[1,2]))
    for f in ('qpos0','jnt_range','jnt_type','jnt_axis','dof_damping','dof_frictionloss','dof_armature','actuator_forcerange','actuator_gainprm','actuator_biasprm','eq_data'):
        np.testing.assert_array_equal(getattr(m,f),getattr(base,f),err_msg=f)
    checks['unchanged_joint_actuator_constraint_parameters']=True
    neighbors={}
    for name in ('sendout_lehisto_1','sendout_lehisto_2','qc_cassette_','imaging_a_s60_pc'):
        gids=[]
        for i in range(m.ngeom):
            bid=int(m.geom_bodyid[i]);match=False
            while bid:
                if m.body(bid).name.startswith('lab_context_'+name):match=True;break
                bid=int(m.body_parentid[bid])
            if match and m.geom_rgba[i,3]>0:gids.append(i)
        bb=bounds(m,d,gids,R,t);neighbors[name]=dict(bounds_m=bb.tolist(),conservative_aabb_gap_m=gap(b,bb))
    checks['clear_of_named_neighbors_at_held_pose']=all(v['conservative_aabb_gap_m']>.01 for v in neighbors.values())
    access=[]
    for c in json.loads(MANIFEST.read_text())['reserved_access']:
        a=np.array([np.array(c['center_m'])-c['half_size_m'],np.array(c['center_m'])+c['half_size_m']]);hits=[]
        for i in range(m.ngeom):
            if i in ids or m.geom_rgba[i,3]==0:continue
            v=vertices(m,d,i)
            if v is None:continue
            v=v@R.T+t;bb=np.stack([v.min(0),v.max(0)])
            if np.all(np.minimum(a[1],bb[1])-np.maximum(a[0],bb[0])>1e-5):hits.append(m.geom(i).name or m.body(int(m.geom_bodyid[i])).name)
        access.append(dict(side=c['side'],obstructions=hits))
    checks['side_access_volumes_unoccupied_at_held_pose']=all(not c['obstructions'] for c in access)
    checks={key:bool(value) for key,value in checks.items()}
    result=dict(passed=all(checks.values()),checks=checks,engine=mujoco.__version__,model_sha256=sha(MODEL),fixture_bounds_world_m=b.tolist(),folder_bounds_world_m=folder.tolist(),feet_bounds_world_m=feet.tolist(),bench_bounds_world_m=bench.tolist(),neighbors=neighbors,access=access,geometry=dict(stand_boxes=10,folder_cad_visual_meshes=3),scope='Static compiled geometry and conservative AABB clearances at held posture only. No swept motion, IK, docking, contact, stiffness, balance or grasp validation.')
    (ROOT/'qc_folder45_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
    if not result['passed']:raise RuntimeError('QC layout checks failed')
    return m,d,R,t

def render():
    from PIL import Image,ImageDraw,ImageFont
    m,d,R,t=check();out=ROOT/'docs/images';out.mkdir(parents=True,exist_ok=True)
    m.vis.global_.offwidth=1400;m.vis.global_.offheight=900;m.vis.headlight.ambient[:]=.3;m.vis.headlight.diffuse[:]=.5;m.light_castshadow[:]=0
    m.light_diffuse[:]=.12;m.light_ambient[:]=0
    opt=mujoco.MjvOption();opt.geomgroup[3:]=0;opt.sitegroup[:]=0
    font=lambda n:ImageFont.truetype('C:/Windows/Fonts/arial.ttf',n)
    for name,point,dist,az,el in [('qc-folder45',[2.74,.47,1.0],2.35,205,-35),('qc-folder45-detail',[2.39,.42,.94],.95,25,-48)]:
        c=mujoco.MjvCamera();c.lookat[:]=R.T@(np.array(point)-t);c.distance=dist;c.azimuth=az;c.elevation=el
        with mujoco.Renderer(m,height=900,width=1400) as renderer:
            renderer.update_scene(d,c,scene_option=opt);im=Image.fromarray(renderer.render())
        draw=ImageDraw.Draw(im);draw.rectangle([0,0,1400,86],fill='#10283c')
        draw.text((24,12),'ExHist Labs | QC: 45-degree slide-folder cradle',font=font(27),fill='white')
        draw.text((24,49),'Fixed fixture / flaps held behind / Nori handling and slide-placement validation pending',font=font(19),fill='#f2cc88');im.save(out/(name+'.png'))

def view():
    import mujoco.viewer
    m,d,R,t=check()
    with mujoco.viewer.launch_passive(m,d) as v:
        v.cam.lookat[:]=R.T@(np.array([2.65,.45,.98])-t);v.cam.distance=2.3;v.cam.azimuth=205;v.cam.elevation=-30;v.opt.geomgroup[3:]=0;v.opt.sitegroup[:]=0
        while v.is_running():
            v.set_texts([(100,mujoco.mjtGridPos.mjGRID_TOPLEFT,'QC 45-degree cradle | held layout inspection\nNori handling, slide placement and passive docking not validated','')]);v.sync();time.sleep(.03)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',action='store_true');p.add_argument('--check',action='store_true');p.add_argument('--render',action='store_true');a=p.parse_args()
    if a.build:build();export_stl()
    if a.render:render()
    elif a.check:check()
    elif not a.build:view()
