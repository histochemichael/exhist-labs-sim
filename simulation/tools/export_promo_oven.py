"""Static STL/XML of the modified promo oven, not manufacturer/native STEP CAD."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco,trimesh

ROOT=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(ROOT/'promo_v4/promo_scene.xml'));d=mujoco.MjData(m)
d.qpos[m.joint('quincy_1_door_joint').qposadr[0]]=0;mujoco.mj_kinematics(m,d)
body=m.body('quincy_1').id;origin=d.xpos[body].copy();pieces=[]
for g in range(m.ngeom):
    b=m.geom_bodyid[g]
    while b and b!=body:b=m.body_parentid[b]
    if b!=body or m.geom_group[g]!=1:continue
    if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
        k=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k]+m.mesh_vertnum[k]];f=m.mesh_face[m.mesh_faceadr[k]:m.mesh_faceadr[k]+m.mesh_facenum[k]]
        mesh=trimesh.Trimesh(v.copy(),f.copy(),process=False)
    elif m.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX:mesh=trimesh.creation.box(2*m.geom_size[g])
    elif m.geom_type[g]==mujoco.mjtGeom.mjGEOM_CYLINDER:mesh=trimesh.creation.cylinder(radius=m.geom_size[g,0],height=2*m.geom_size[g,1],sections=32)
    else:continue
    mesh.vertices=(mesh.vertices@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]-origin)*1000;pieces.append(mesh)
target=ROOT/'cad/stl/quincy-promo-tall-concept.stl';trimesh.util.concatenate(pieces).export(target)
root=ET.Element('mujoco',model='Modified promo oven - static inspection, not fabrication CAD')
ET.SubElement(root,'compiler',angle='radian');assets=ET.SubElement(root,'asset')
ET.SubElement(assets,'mesh',name='oven',file='../../cad/stl/quincy-promo-tall-concept.stl',scale='.001 .001 .001',inertia='shell')
world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='0 -1 2');ET.SubElement(world,'geom',type='mesh',mesh='oven',rgba='.65 .65 .67 1')
ET.indent(root);ET.ElementTree(root).write(ROOT/'models/assets/quincy-promo-tall.xml',encoding='utf-8',xml_declaration=True)
print('Exported modified promo oven: STL millimetres; static XML metres; no fabricated native STEP.')
