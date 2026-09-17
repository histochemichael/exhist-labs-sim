"""Display the tested proposed pull on all incubators in the review scene."""
import math,copy
import numpy as np
from build_scene import el,vec

def add(root):
    from review_corrections import cad_coordinates
    from quincy_offset_pull import add as add_interface
    src,rotation,origin=cad_coordinates('quincy')
    part=next(p for p in src['parts'] if 'Vertical thermoplastic handle' in p['name'])
    v=np.array(part['vertices']).reshape(-1,3)-origin
    handle=(v.min(0)+v.max(0))/2
    for machine in root.findall('worldbody/body'):
        name=machine.get('name','')
        if not name.startswith('quincy_'):continue
        door=machine.find(f"body[@name='{name}_passive_door']")
        if door is None or door.find(f"body[@name='{name}_proposed_offset_pull']") is not None:continue
        center=handle-np.fromstring(door.get('pos'),sep=' ');angle=float(door.find('joint').get('ref','0'))
        R=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
        dummy=el(door,'geom',name='access_handle',type='sphere',size='.001',rgba='0 0 0 0',contype='0',conaffinity='0')
        grasp=add_interface(door,center,R);door.remove(dummy)
        b=door.find("body[@name='quincy_proposed_offset_pull']");b.set('name',name+'_proposed_offset_pull')
        for geom in b.findall('geom'):geom.set('name',name+'_'+geom.get('name'));geom.set('contype','0');geom.set('conaffinity','0')
        el(b,'site',name=name+'_proposed_pull_grasp',pos='0 -.035 0',size='.002',rgba='0 0 0 0')
    return root

if __name__=='__main__':
    import xml.etree.ElementTree as ET
    from build_scene import ROOT
    path=ROOT/'exhist_first_pass.xml';root=ET.parse(path).getroot();add(root);ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
