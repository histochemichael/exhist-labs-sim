"""Export representative procedural room assets from actual compiled scene geoms."""
import copy,json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco,numpy as np,trimesh
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'))
from build_gallery import render,stl,vec,font
def main():
    m=mujoco.MjModel.from_xml_path(str(ROOT/'exhist_operational.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    groups={
      'bench':lambda n,b:b=='sorting_table',
      'canopy':lambda n,b:b=='special_canopy',
      'cabinet':lambda n,b:n.startswith('lab_storage_tower_0'),
      'shelves':lambda n,b:n.startswith(('lab_shelf_rail_0','lab_shelf_0_','lab_shelf_box_0_')),
      'chair':lambda n,b:n.startswith('lab_task_chair_0'),
      'pc':lambda n,b:n.startswith('lab_desk_pc_2'),
      'entry-door':lambda n,b:n.startswith(('lab_entry_door','lab_entry_vision','lab_entry_pushbar')),
      'window':lambda n,b:n.startswith('lab_window_0')
    }
    titles={'bench':'Process bench','canopy':'Canopy hood concept','cabinet':'Storage cabinet','shelves':'Shelf bay and boxes','chair':'Lab task chair','pc':'PC workstation','entry-door':'Staff entry door','window':'Observation window'}
    report=[]
    for key,select in groups.items():
        ids=[i for i in range(m.ngeom) if select(m.geom(i).name,m.body(int(m.geom_bodyid[i])).name)]
        assert ids,key
        chunks=[];specs=[]
        for i in ids:
            kind=mujoco.mjtGeom(int(m.geom_type[i])).name.replace('mjGEOM_','').lower()
            s=m.geom_size[i]
            if kind=='box':mesh=trimesh.creation.box(extents=2*s)
            elif kind=='sphere':mesh=trimesh.creation.icosphere(subdivisions=2,radius=s[0])
            elif kind=='cylinder':mesh=trimesh.creation.cylinder(radius=s[0],height=2*s[1],sections=32)
            elif kind=='capsule':mesh=trimesh.creation.capsule(radius=s[0],height=2*s[1],count=[12,16])
            else:raise RuntimeError((key,m.geom(i).name,kind))
            T=np.eye(4);T[:3,:3]=d.geom_xmat[i].reshape(3,3);T[:3,3]=d.geom_xpos[i];mesh.apply_transform(T);chunks.append(mesh)
            quat=np.zeros(4);mujoco.mju_mat2Quat(quat,d.geom_xmat[i])
            count=3 if kind=='box' else 1 if kind=='sphere' else 2
            specs.append(dict(type=kind,size=vec(s[:count]),pos=vec(d.geom_xpos[i]),quat=vec(quat),rgba=vec(m.geom_rgba[i])))
        merged=trimesh.util.concatenate(chunks);center=merged.bounds.mean(0);merged.apply_translation(-center)
        root=ET.Element('mujoco',model=titles[key]+' | static procedural reference')
        visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='1000',offheight='600')
        asset=ET.SubElement(root,'asset');ET.SubElement(asset,'texture',type='skybox',builtin='gradient',rgb1='.92 .95 .97',rgb2='.70 .78 .84',width='512',height='3072')
        world=ET.SubElement(root,'worldbody')
        for spec in specs:
            spec['pos']=vec(np.fromstring(spec['pos'],sep=' ')-center)
            ET.SubElement(world,'geom',**spec,contype='0',conaffinity='0')
        path=ROOT/'models/assets'/(key+'.xml');ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
        stl(ROOT/'cad/stl'/(key+'.stl'),np.asarray(merged.vertices),np.asarray(merged.faces))
        mm=mujoco.MjModel.from_xml_path(str(path));dd=mujoco.MjData(mm);mujoco.mj_forward(mm,dd)
        render(mm,dd,[0,0,0],max(merged.extents)*1.8,ROOT/'docs/images'/('asset-'+key+'.png'),az=45 if key in ('cabinet','pc','entry-door') else 230,el=-20)
        report.append(dict(id=key,title=titles[key],geoms=[m.geom(i).name for i in ids],xml='models/assets/'+key+'.xml',stl='cad/stl/'+key+'.stl',units_stl='mm',note='Procedural scene geometry, no native STEP source; static visualization.'))
        print('Room asset:',key,flush=True)
    sheet=Image.new('RGB',(1000,4*328),'#eff4f7');draw=ImageDraw.Draw(sheet)
    for i,row in enumerate(report):
        im=Image.open(ROOT/'docs/images'/('asset-'+row['id']+'.png'));im.thumbnail((490,294));x=(i%2)*500;y=(i//2)*328
        sheet.paste(im,(x,y));draw.text((x+10,y+299),row['title'],font=font(19),fill='#142b3c')
    sheet.save(ROOT/'docs/images/room-assets.jpg',quality=88)
    (ROOT/'docs/room-asset-catalog.json').write_text(json.dumps(report,indent=2))
if __name__=='__main__':main()
