"""Proposed passive U-pulls, attached to existing CAD drawer bodies.
Concept mounting pattern, not a manufacturer-approved machine modification.
"""
import json
from functools import lru_cache
import numpy as np
from build_scene import ROOT,el,box,vec

PANELS={'UNLOAD_DRAWER_OPEN_mm':'PHOTO_FLUSH_UNLOAD_white_front_fascia',
        'LOAD_DRAWER_OPEN_mm':'PHOTO_FLUSH_LOAD_white_front_fascia',
        'EST_DRAWER_SLIDER':'drawer_front','TS5025_DRAWER_OPEN_mm':'TS_front_white'}

@lru_cache(maxsize=1)
def designs():
    src=json.loads((ROOT/'assets/leica_workstation_handled.json').read_text())
    kin=json.loads((ROOT/'assets/leica_kinematics.json').read_text())
    result=[]
    for name,part in PANELS.items():
        j=next(j for j in kin['joints'] if j['name']==name)
        p=next(p for p in src['parts'] if p['name']==j['one']+'/'+part)
        v=np.array(p['vertices']).reshape(-1,3);lo=v.min(0);hi=v.max(0)
        center=np.array([(lo[0]+hi[0])/2,lo[1],(lo[2]+hi[2])/2])
        result.append(dict(cad_joint=name,source_panel=p['name'],front_center_m=center.tolist(),
            joint_origin_m=j['origin'],grip_frame_CAD_m=(center+[0,-.030,0]).tolist(),
            dimensions_mm=dict(width=82,projection=33,height=10,clear_width=66,clear_depth=27,mount_hole_diameter=3.3),
            joint_axis=j['dofs'][0]['axis'],joint_limits=j['dofs'][0]['limits']))
    return result

def add_pull(root,body,joint,station):
    rows={r['cad_joint']:r for r in designs()}
    if joint['name'] not in rows:return
    row=rows[joint['name']];base=np.array(row['front_center_m'])-joint['origin']
    prefix=station+'_'+joint['name']+'_pull'
    for name,delta,size in [('left',[-.037,-.0165,0],[.004,.0165,.005]),
                           ('right',[.037,-.0165,0],[.004,.0165,.005]),
                           ('bar',[0,-.030,0],[.041,.003,.005])]:
        box(body,prefix+'_'+name,base+delta,size,[.25,.58,.62,1],mass='0',contype='1',conaffinity='1',group='1')
    el(body,'site',name=prefix+'_grasp',pos=vec(base+[0,-.030,0]),size='.003',rgba='1 .5 0 1')
    el(body,'site',name=prefix+'_approach',pos=vec(base+[0,-.12,0]),size='.003',rgba='.1 .6 1 1')

def manifest():
    report=dict(pulls=designs(),copies=2,total_pulls=8,passive=True,actuators_added=0,
        limitations=['Concept pull mounts require real panel/load/fastener review.',
            '3.3 mm mounting holes are present in the separate CAD concept, omitted from simple contact boxes.',
            'CAD limits retained; Nori left-hand reach, latch force and collision sweep still need validation.'])
    (ROOT/'drawer_pull_design.json').write_text(json.dumps(report,indent=2));return report

def upgrade_pulls(root):
    """Apply the wider v02 concept to existing generated scenes, not source Fusion."""
    for body in root.findall('.//body'):
        bars=[g for g in body.findall('geom') if g.get('name','').endswith('_pull_bar')]
        for bar in bars:
            center=np.fromstring(bar.get('pos'),sep=' ');bar.set('size','.041 .003 .005')
            prefix=bar.get('name')[:-4]
            for side,dx in [('left',-.037),('right',.037)]:
                leg=body.find(f"geom[@name='{prefix}_{side}']")
                p=np.fromstring(leg.get('pos'),sep=' ');p[0]=center[0]+dx;leg.set('pos',vec(p))
    manifest()

if __name__=='__main__':print(json.dumps(manifest(),indent=2))
