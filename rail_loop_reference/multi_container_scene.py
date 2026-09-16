"""Separate nine-station horseshoe scene derived from the validated contact model."""
import copy,hashlib,json
import xml.etree.ElementTree as ET
import numpy as np
from shapely.geometry import Polygon,box
from build_scene import ROOT,JAR,SEAT,vec

ORIGIN=np.array([-.000099,-.007243,0.])
ANGLES=[-105.,-80.,-55.,-30.,0.,30.,55.,80.,105.]

def rz(angle):
    a=np.deg2rad(angle);c,s=np.cos(a),np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def build():
    root=ET.parse(ROOT/'scene.xml').getroot();world=root.find('worldbody')
    templates={name:world.find(f"body[@name='{name}']") for name in ('jar','riser')}
    for item in templates.values():world.remove(item)
    palette=[(.22,.66,.69),(.45,.72,.48),(.83,.58,.31),(.72,.45,.56),(.40,.62,.82),(.68,.62,.33),(.66,.46,.77),(.40,.73,.65),(.80,.49,.39)]
    stations=[];footprints=[]
    for i,(angle,color) in enumerate(zip(ANGLES,palette),1):
        pos=ORIGIN+rz(angle)@(JAR-ORIGIN)
        a=np.deg2rad(90+angle)/2;quat=[np.cos(a),0,0,np.sin(a)]
        prefix=f'station_{i:02d}_'
        for kind,template in templates.items():
            body=copy.deepcopy(template)
            for node in body.iter():
                if node.get('name'):node.set('name',prefix+node.get('name'))
            body.set('pos',vec(pos+[0,0,-.0015 if kind=='riser' else 0]))
            body.set('quat',vec(quat))
            if kind=='jar':body.find(f"geom[@name='{prefix}jar_visual']").set('rgba',vec([*color,.38]))
            world.append(body)
        corners=np.array([[-.0535,-.0265],[.0535,-.0265],[.0535,.0265],[-.0535,.0265]])
        footprint=corners@rz(90+angle)[:2,:2].T+pos[:2]
        footprints.append(Polygon(footprint))
        stations.append(dict(id=i,name=f'Container {i}',angle_deg=angle,position_m=pos.tolist(),quat_wxyz=quat,
                             jar_body=prefix+'jar',riser_body=prefix+'riser',prefix=prefix,color_rgb=color,footprint_xy_m=footprint.tolist()))
    rack=world.find("body[@name='rack']")
    rack.set('pos',vec([*stations[0]['position_m'][:2],SEAT+.0005]))
    rack.set('quat',vec(stations[0]['quat_wxyz']))
    gaps=[footprints[i].distance(footprints[j]) for i in range(len(footprints)) for j in range(i)]
    # Conservative plan-view keep-out for the rail and base; stations never overlap it.
    rig_keepout=box(-.105,-.10,.105,.55)
    rig_gaps=[p.distance(rig_keepout) for p in footprints]
    if min(gaps)<.010 or min(rig_gaps)<.010:raise RuntimeError('Container layout clearance failed')
    ET.indent(root);ET.ElementTree(root).write(ROOT/'multi_container.xml',encoding='utf-8',xml_declaration=True)
    manifest=dict(stations=stations,route=list(range(1,10)),source_scene_sha256=hashlib.sha256((ROOT/'scene.xml').read_bytes()).hexdigest(),
                  layout='210-degree reachable horseshoe; rear/rail kept clear, not an unreachable 360-degree ring',
                  minimum_container_gap_m=min(gaps),minimum_rig_keepout_gap_m=min(rig_gaps),
                  support_height_m=.0254,rail_parked_m=0,hardware_access=False,payload_attachment=False,
                  scope='Offline reference motion. Not a learned visual policy or a measured physical layout.')
    (ROOT/'multi_container_layout.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps({'stations':len(stations),'minimum_container_gap_mm':min(gaps)*1000,'minimum_rig_gap_mm':min(rig_gaps)*1000},indent=2))
    return manifest

if __name__=='__main__':build()