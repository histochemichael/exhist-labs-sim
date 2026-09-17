"""Passive QC blocks: CAD visuals and conservative support-only contacts.

20 g mass and cuboid inertia are provisional. This is not a grasp, fracture,
wax-deformation or labeling-face collision model.
"""
import json,xml.etree.ElementTree as ET
import numpy as np
from build_scene import ROOT,el,vec

def apply(root):
    src=json.loads((ROOT/'assets/cassette_block.json').read_text())
    v=np.concatenate([np.asarray(p['vertices']).reshape(-1,3) for p in src['parts']])
    lo,hi=v.min(0),v.max(0);center=(lo+hi)/2;half=(hi-lo)/2
    for i in range(1,9):
        name=f'qc_cassette_{i}';body=root.find(f"worldbody/body[@name='{name}']")
        if body is None:continue
        if body.find('freejoint') is None:el(body,'freejoint',name=name+'_free')
        if body.find(f"geom[@name='{name}_support']") is None:
            el(body,'geom',name=name+'_support',type='box',pos=vec(center),size=vec(half),mass='.020',
                rgba='0 0 0 0',group='3',contype='1',conaffinity='1',friction='.5 .005 .0001',solref='.004 1')

if __name__=='__main__':
    t=ET.parse(ROOT/'exhist_operational.xml');apply(t.getroot());ET.indent(t);t.write(ROOT/'exhist_operational.xml',encoding='utf-8',xml_declaration=True)
