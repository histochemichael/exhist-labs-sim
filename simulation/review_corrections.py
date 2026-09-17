"""CAD placement and interface corrections for the kinematic review copy.

No source CAD or dynamic test fixture is changed. Coordinates are CAD-derived
where possible; a pose match is not proof of a force-closed grasp.
"""
import copy,json,math
import numpy as np
from build_scene import ROOT,el,vec
from jar_riser import ensure_riser,export_design,RACK_SEAT_Z,RISER_HEIGHT
from lehisto_loaded_reach import profile,row_position

JAR_COUNT=profile()['layout']['count']
JAR_PITCH=profile()['layout']['pitch_m']
JAR_Y=profile()['layout']['row_y_m']
TABLE_Z=.8

def cad_coordinates(key):
    src=json.loads((ROOT/'assets'/f'{key}.json').read_text())
    rotation=np.array([[1,0,0],[0,0,-1],[0,1,0]]) if key=='s60' else np.eye(3)
    points=np.concatenate([np.array(p['vertices']).reshape(-1,3)@rotation.T for p in src['parts']])
    lo,hi=points.min(0),points.max(0)
    return src,rotation,np.array([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]])

def apply(root):
    from canopy_hoods import add_special_canopy
    add_special_canopy(root)
    from lab_quincy_pulls import add as add_quincy_pulls
    add_quincy_pulls(root)
    from drawer_pulls import upgrade_pulls
    upgrade_pulls(root)
    world=root.find('worldbody')
    # The original table begins at y=.05. Previous y=.02/- .05 points
    # were outside the support polygon, irrespective of their z coordinate.
    for i in range(1,9):
        body=world.find(f"body[@name='qc_cassette_{i}']")
        body.set('pos',vec([3.64+(i-1)%2*.065,.43+(i-1)//2*.055,TABLE_Z]))
    template=world.find("body[@name='special_staining_jar_1']")
    for name in ('special_lehisto','sorter_1','sorter_2'):
        b=world.find(f"body[@name='{name}']");p=np.fromstring(b.get('pos'),sep=' ');p[1]=.54;b.set('pos',vec(p))
        if name=='special_lehisto':p[0]=1.1725;b.set('pos',vec(p))
    for i in range(1,JAR_COUNT+1):
        body=world.find(f"body[@name='special_staining_jar_{i}']")
        if body is None:
            body=copy.deepcopy(template)
            for e in body.iter():
                if e.get('name'):e.set('name',e.get('name').replace('jar_1','jar_'+str(i)))
            world.append(body)
        body.set('pos',vec([*row_position(i),.8055]))
        body.set('quat',vec([math.sqrt(.5),0,0,math.sqrt(.5)]))
        body.find('geom').set('rgba','.55 .68 .73 1' if i in (4,5,6,7,8) else '.42 .47 .49 1')
        ensure_riser(root,body)
    export_design()
    for site in list(world.findall('site')):
        if site.get('name','').startswith('special_bath_target_'):world.remove(site)
    for i in range(JAR_COUNT):
        el(world,'site',name=f'special_bath_target_{i+1}',pos=vec([*row_position(i+1),RACK_SEAT_Z]),size='.002',rgba='0 0 0 0')
    # Frames on the existing OEM-style left jaw, no replacement gripper added.
    wrist=world.find(".//body[@name='left_wrist_roll_link']")
    el(wrist,'site',name='nori_left_carrier_grasp',pos='0 0 -.075',size='.002',rgba='0 0 0 0')
    for x in (2.39,3.31):
        el(world,'geom',name='slide_staging_support_'+str(x),type='box',pos=vec([x+.304,.52,.8008]),size='.04 .018 .0008',rgba='.3 .4 .45 1')
    for key,prefix,handle_name in [('quincy','quincy_','Vertical thermoplastic handle'),('s60','imaging_','/handle')]:
        src,r,origin=cad_coordinates(key)
        part=next(p for p in src['parts'] if handle_name in p['name'] and (key!='s60' or p['name'].startswith('cassette_door')))
        v=np.array(part['vertices']).reshape(-1,3)@r.T-origin
        center=(v.min(0)+v.max(0))/2
        for b in world.findall('body'):
            name=b.get('name','')
            if not name.startswith(prefix):continue
            door=b.find(f"body[@name='{name}_passive_door']")
            if door is None:continue
            local=center-np.fromstring(door.get('pos'),sep=' ')
            el(door,'site',name=name+'_handle_contact',pos=vec(local),size='.003',rgba='0 0 0 0')
    # Hide debug spheres, not equipment components, in normal review views.
    for site in root.findall('.//site'):
        if not site.get('name','').endswith('_activity'):site.set('rgba','0 0 0 0')
    (ROOT/'review_correction_design.json').write_text(json.dumps(dict(
        jars={'count':JAR_COUNT,'pitch_m':JAR_PITCH,'y_m':JAR_Y,'yaw_deg':90,'reach_verified':False,'riser_height_m':RISER_HEIGHT,'rack_seat_world_z_m':RACK_SEAT_Z},
        support_top_m=TABLE_Z,
        grasp_evidence=['assets/rack24.json T-bar bounds','assets/leica_rack_handled.json snap handle bounds',
            'C:/Users/Owner/lerobot/data/so101_handle_segmentation_work/images/so101_rack_lift_place_aruco_2cam__ep000__f0620.jpg',
            'C:/Users/Owner/lerobot/data/so101_rack_approach_centered_15hz_20260913/diagnostics/training_audit/episodes_00_04.jpg'],
        limits=['Left carrier grasp is a provisional frame on existing jaws; no new gripper installed.',
                'Door sites are CAD-derived; contact forces and loaded extraction remain unvalidated.']),indent=2))
