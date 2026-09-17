"""Build a separate kinematic review scene from the actual three-minute CAD film.

Never modifies the contact-test scene or the source CAD/film. The film's rigid
assembly transforms are playback data, not MuJoCo dynamic constraints.
"""
import json, sys, collections, math
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET
from build_scene import ROOT, el, vec

FILM = Path('C:/Users/Owner/.codex/visualizations/2026/09/13/01a09bd0-bb0b-7e32-bc05-42f089adee39/Leica_CV5030_Project/mechanical_qc_v2')

def build():
    data=json.loads((FILM/'native_qc_meshes.json').read_text())
    # Keep the user's later snap-handle update on the rack from the older film.
    source=json.loads((ROOT/'assets/leica_rack_handled.json').read_text())
    rack=next(p for p in source['parts'] if p['name']=='root/Slide rack')
    handle=next(p for p in source['parts'] if 'Removable snap handle' in p['name'])
    rack_item=next(i for i in data['instances'] if i['component']=='20 RACK L33 - liftable')
    native=np.asarray(data['meshes'][rack_item['key']]['vertices']).reshape(-1,3)*.01
    original=np.asarray(rack['vertices']).reshape(-1,3)
    shift=(native.min(0)+native.max(0)-original.min(0)-original.max(0))/2
    added=dict(rack_item,key='first_pass/current_removable_snap_handle')
    data['meshes'][added['key']]=dict(vertices=((np.asarray(handle['vertices']).reshape(-1,3)+shift)*100).ravel().tolist(),triangles=handle['triangles'],rgba=handle['rgba'])
    data['instances'].append(added)
    root=ET.parse(ROOT/'exhist_operational.xml').getroot()
    root.set('model','ExHist FIRST PASS - KINEMATIC WORKFLOW, NOT CONTACT PHYSICS')
    world=root.find('worldbody');assets=root.find('asset')
    # Additional fixed head angle aimed toward the right-hand interaction zone.
    # Simulation camera only; no claim that this mounting location is hardware validated.
    yaw=math.radians(-40);pitch=math.radians(30)
    right=[math.sin(yaw),-math.cos(yaw),0]
    up=[math.sin(pitch)*math.cos(yaw),math.sin(pitch)*math.sin(yaw),math.cos(pitch)]
    el(world.find(".//body[@name='lift_top_link']"),'camera',name='nori_head_interaction',pos='.09 -.035 .24',xyaxes=vec(right+up),fovy='80')
    # Keep every original CAD joint, but hide its old visual layer in this copy.
    bases={}
    for station in ('routine_a','routine_b'):
        base=world.find(f"body[@name='{station}_workstation']")
        bases[station]=list(map(float,base.get('pos').split()))
        for g in base.iter('geom'):
            g.set('rgba','0 0 0 0');g.set('contype','0');g.set('conaffinity','0')
    groups=collections.defaultdict(list)
    for item in data['instances']:
        # The flex-hose renderer uses non-rigid segment scaling, not a rigid pose.
        # Omit that cosmetic hose rather than turn a scaled matrix into a joint.
        if item['component']=='QC_FLEXIBLE_MOUNTANT_HOSE_EST':continue
        key=(item['path'],'film' if 'QC_mountant_film' in item['key'] else '')
        groups[key].append(item)
    out=ROOT/'assets/film_v3';out.mkdir(exist_ok=True)
    keys=sorted({i['key'] for items in groups.values() for i in items})
    mesh_names={key:f'film_v3_{n}' for n,key in enumerate(keys)}
    for key,name in mesh_names.items():
        rec=data['meshes'][key];v=np.asarray(rec['vertices']).reshape(-1,3)*.01
        f=np.asarray(rec['triangles']).reshape(-1,3)+1
        with (out/(name+'.obj')).open('w') as stream:
            stream.writelines('v '+vec(row)+'\n' for row in v)
            stream.writelines('f '+' '.join(map(str,row))+'\n' for row in f)
        el(assets,'mesh',name=name,file=f'assets/film_v3/{name}.obj',inertia='shell')
    records=[]
    for n,items in enumerate(groups.values()):
        sample=items[0];record=dict(index=n,item=sample,geom_names={})
        for station in bases:
            body=el(world,'body',name=f'{station}_film_{n}',mocap='true',pos='0 0 -10')
            names=[]
            for k,item in enumerate(items):
                name=f'{station}_film_{n}_{k}';names.append(name)
                color=list(data['meshes'][item['key']]['rgba'])
                co=item['component']
                if 'hood' in co.lower() or 'FIXED LID' in co or 'CLEAR_COVER' in co:color=[.65,.8,.82,.12]
                el(body,'geom',name=name,type='mesh',mesh=mesh_names[item['key']],rgba=vec(color),contype='0',conaffinity='0',density='0',group='1')
            record['geom_names'][station]=names
        records.append(record)
    # Original fixture, measured-groove and head-camera definitions are preserved.
    el(world,'body',name='first_pass_slide',mocap='true',pos='0 0 -10')
    b=world.find("body[@name='first_pass_slide']")
    el(b,'geom',name='first_pass_slide_glass',type='box',size='.012475 .0005 .0375',rgba='.3 .85 .95 .85',contype='0',conaffinity='0')
    el(b,'geom',type='box',pos='0 0 .029',size='.012475 .0007 .0085',rgba='.95 .95 .8 1',contype='0',conaffinity='0')
    from folder_finish import add_folders
    add_folders(root)
    from review_corrections import apply
    apply(root)
    ET.indent(root);ET.ElementTree(root).write(ROOT/'exhist_first_pass.xml',encoding='utf-8',xml_declaration=True)
    manifest=dict(source=str(FILM/'Leica-Workstation-Three-Minute-v3.mp4'),film_dir=str(FILM),bases=bases,
                  offset_m=[.6,.0728575,0],groups=records,hose_reference=data.get('hose_reference_state',[0,0,0]),
                  scope='CAD-film rigid-transform playback. Not force/contact validation. Cosmetic deforming hose omitted.')
    (ROOT/'first_pass_assets.json').write_text(json.dumps(manifest))
    print('First-pass CAD layer:',len(records),'rigid groups,',len(keys),'meshes per workstation.')

if __name__=='__main__':build()
