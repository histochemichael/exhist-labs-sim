"""Equipment-facing docking and fixed head-camera frames, not tracking cameras."""
import json,math
import numpy as np
from build_scene import ROOT,el,vec

PARAMETERS=json.loads((ROOT/'simulation_parameters.json').read_text())

def facing_yaw(base_xy,equipment_xy):
    delta=np.asarray(equipment_xy,float)-np.asarray(base_xy,float)
    if delta.shape!=(2,) or not np.isfinite(delta).all() or np.linalg.norm(delta)<1e-6:raise ValueError('Distinct finite base and equipment points required')
    return math.atan2(delta[1],delta[0])

def facing_error(base_xy,base_yaw,equipment_xy):
    return (facing_yaw(base_xy,equipment_xy)-base_yaw+math.pi)%(2*math.pi)-math.pi

def docking_ready(base_xy,yaw,equipment_xy,linear_speed,yaw_speed):
    p=PARAMETERS['docking']
    values=np.r_[base_xy,yaw,equipment_xy,linear_speed,yaw_speed]
    if not np.isfinite(values).all():return False
    return (abs(facing_error(base_xy,yaw,equipment_xy))<math.radians(p['heading_tolerance_deg'])
            and 0<=linear_speed<p['stopped_linear_speed_mps'] and abs(yaw_speed)<p['stopped_yaw_speed_radps'])

def add_head_cameras(root):
    head=root.find(".//body[@name='lift_top_link']")
    for name,y in [('nori_head_left',.0313632),('nori_head_right',-.0313632)]:
        pitch=math.radians(PARAMETERS['head_cameras']['downward_pitch_deg'])
        # Camera -Z points forward/down, +X points toward robot right (-Y).
        right=np.array([0,-1,0.]);up=np.array([math.sin(pitch),0,math.cos(pitch)])
        el(head,'camera',name=name,pos=vec([.058,y,.240548]),xyaxes=vec(np.r_[right,up]),fovy=str(PARAMETERS['head_cameras']['fovy_deg']))
    pitch=math.radians(35)
    el(head,'camera',name='nori_head_wide',pos='.094 0 .312',xyaxes=vec([0,-1,0,math.sin(pitch),0,math.cos(pitch)]),fovy='100')

def apply_provisional_lift(root):
    eq=root.find('equality');brake=eq.find("joint[@name='TEST_FIXTURE_lift_brake']")
    if brake is not None:eq.remove(brake)
    p=PARAMETERS['nori_lift']
    root.find(".//joint[@name='lift_extension_joint']").set('actuatorfrcrange',vec([-p['maximum_force_N'],p['maximum_force_N']]))
    el(root.find('actuator'),'position',name='provisional_lift_motor',joint='lift_extension_joint',kp=str(p['kp_N_per_m']),kv=str(p['kv_Ns_per_m']),forcerange=vec([-p['maximum_force_N'],p['maximum_force_N']]),ctrlrange='0 .73')
    # Nested lift proxies are solid bounding boxes, not the hollow telescopic tubes.
    # Suppress ONLY their spurious internal pair; external collisions stay active.
    contact=root.find('contact')
    if contact is None:contact=el(root,'contact')
    el(contact,'exclude',name='nested_lift_proxy_pair',body1='lift_top_link',body2='lift_middle_link')
    eq.find("joint[@name='lift_mimic']").set('solref','.002 1')

def write_station_docks():
    layout=json.loads((ROOT/'layout.json').read_text());rows=[]
    for station in layout['stations']:
        rows.append(dict(station=station['name'],base_position_m=station['dock'],equipment_point_m=[station['x'],station['handoff'][1]],
                         required_yaw_rad=facing_yaw(station['dock'][:2],[station['x'],station['handoff'][1]]),
                         status='Heading contract; arm reach, chassis stand-off and collision clearance still require physical validation'))
    (ROOT/'equipment_facing_docks.json').write_text(json.dumps(rows,indent=2))
    return rows

if __name__=='__main__':print(json.dumps(write_station_docks(),indent=2))
