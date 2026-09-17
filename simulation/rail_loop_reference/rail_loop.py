"""Loaded moving-carriage loop restricted to existing stroke, with rail obstacle guard."""
import argparse,copy,hashlib,json
import xml.etree.ElementTree as ET
import numpy as np,mujoco
from shapely.geometry import Polygon,box
from build_scene import ROOT,vec
from multi_container_scene import ORIGIN,rz
from lab_row_test import ROT,TRANSLATION,JAR_Z,SEAT
from multi_transfer import YawIK
from reference_trial import ARM
NAME='rail_loop'
RAIL_LO=-.06
RAIL_HI=.21
RADIUS=.27
RAIL_BOUNDS=[[-.08135169,-.18470239,-.06966158],[.08189195,.41529765,-.007352]]

class RailIK(YawIK):
    def __init__(self,rail=RAIL_LO):
        super().__init__();self.rail=rail
    def target_yaw(self,xyz,yaw,iterations=5):
        if not RAIL_LO-1e-9<=self.rail<=RAIL_HI+1e-9:raise ValueError('Planned rail outside original limits')
        self.robot.set_joint('rail_travel',self.rail)
        return super().target_yaw(xyz,yaw,iterations)

def build():
    root=ET.parse(ROOT/'scene.xml').getroot();world=root.find('worldbody')
    templates={n:world.find(f"body[@name='{n}']")for n in ('jar','riser')}
    for b in templates.values():world.remove(b)
    world.find("geom[@name='floor']").set('pos',vec([0,0,JAR_Z-.0055]))
    design=[(q,-90.)for q in np.linspace(RAIL_LO,RAIL_HI,4)]+[(q,90.)for q in np.linspace(RAIL_HI,RAIL_LO,4)]+[(RAIL_LO,a)for a in (45.,0.,-45.)]
    polygons=[];stations=[];bench=box(.575,.05,1.725,1.05)
    for i,(rail,angle) in enumerate(design,1):
        p=ORIGIN+[0,rail,0]+rz(angle)@np.array([.010099,-np.sqrt(RADIUS**2-.010099**2),0]);p[2]=JAR_Z
        a=np.deg2rad(90+angle)/2;quat=[np.cos(a),0,0,np.sin(a)];prefix=f'station_{i:02d}_'
        for kind,temp in templates.items():
            b=copy.deepcopy(temp)
            for node in b.iter():
                if node.get('name'):node.set('name',prefix+node.get('name'))
            b.set('pos',vec(p+[0,0,-.0015 if kind=='riser' else 0]));b.set('quat',vec(quat));world.append(b)
        corners=np.array([[-.0535,-.0265],[.0535,-.0265],[.0535,.0265],[-.0535,.0265]])
        fp=corners@rz(90+angle)[:2,:2].T+p[:2];polygons.append(Polygon(fp))
        world_fp=np.c_[fp,np.full(4,JAR_Z)]@ROT.T+TRANSLATION
        if not bench.buffer(-.02).contains(Polygon(world_fp[:,:2])):raise RuntimeError(f'Bath {i} outside bench')
        stations.append(dict(id=i,name=f'Bath {i}',rail_m=float(rail),angle_deg=angle,position_m=p.tolist(),world_position_m=(ROT@p+TRANSLATION).tolist(),quat_wxyz=quat,jar_body=prefix+'jar',riser_body=prefix+'riser',prefix=prefix,footprint_xy_m=fp.tolist(),world_footprint_xy_m=world_fp[:,:2].tolist(),color_rgb=[.3,.65,.7]))
    gap=min(polygons[i].distance(polygons[j]) for i in range(11)for j in range(i))
    riggap=min(p.distance(box(-.105,-.19,.105,.43))for p in polygons)
    if gap<.01 or riggap<.01:raise RuntimeError(f'Layout overlap: jars {gap}, rig {riggap}')
    rack=world.find("body[@name='rack']");rack.set('pos',vec([*stations[0]['position_m'][:2],SEAT+.0005]));rack.set('quat',vec(stations[0]['quat_wxyz']))
    # Rack-only conservative solid proxy: retain normal contacts and add rail contact bit.
    for g in rack.findall('geom'):
        if g.get('name') in ('rack_lower','rack_neck','rack_bar'):g.set('contype','5')
    lo,hi=np.array(RAIL_BOUNDS);ET.SubElement(world,'geom',name='rail_rack_guard',type='box',pos=vec((lo+hi)/2),size=vec((hi-lo)/2),contype='0',conaffinity='4',group='3',rgba='1 .2 .2 .15')
    ET.indent(root);ET.ElementTree(root).write(ROOT/f'{NAME}.xml',encoding='utf-8',xml_declaration=True)
    layout=dict(stations=stations,route=list(range(1,12))+[1],layout_title='Moving rail: both sides, full modeled stroke, closed return',world_rotation=ROT.tolist(),world_translation=TRANSLATION.tolist(),jar_z=JAR_Z,seat_z=SEAT,radius_m=RADIUS,minimum_container_gap_m=gap,minimum_rig_gap_m=riggap,rail_limits_m=[RAIL_LO,RAIL_HI],support_height_m=.0254,bench_bounds_m=[.575,.05,1.725,1.05],rail_bounds_m=RAIL_BOUNDS,scope='Full 270mm MODELED stroke, not the full extrusion length. Far-side crossover passes above the rail, not a continuous 360deg orbit around its physical ends. Existing hardware limits unchanged.')
    (ROOT/f'{NAME}_layout.json').write_text(json.dumps(layout,indent=2))
    print('layout gaps mm',gap*1000,riggap*1000,flush=True);return layout

def trajectory(source,dest,p0,p1,u):
    c0=ORIGIN[:2]+[0,source['rail_m']];c1=ORIGIN[:2]+[0,dest['rail_m']]
    v0=p0[:2]-c0;v1=p1[:2]-c1
    a=np.arctan2(v0[1],v0[0])+np.deg2rad(dest['angle_deg']-source['angle_deg'])*u
    radius=np.linalg.norm(v0)*(1-u)+np.linalg.norm(v1)*u
    p=np.r_[c0*(1-u)+c1*u+radius*np.array([np.cos(a),np.sin(a)]),p0[2]*(1-u)+p1[2]*u+.11]
    return p,source['angle_deg']*(1-u)+dest['angle_deg']*u,source['rail_m']*(1-u)+dest['rail_m']*u

def screen():
    layout=build();m=mujoco.MjModel.from_xml_path(str(ROOT/f'{NAME}.xml'));d=mujoco.MjData(m);qa=[m.joint(n).qposadr[0]for n in ARM];cases=[]
    route=layout['stations']+[layout['stations'][0]]
    for s,dest in zip(route[:-1],route[1:]):
        ik=RailIK(s['rail_m']);samples=[]
        tasks=[(np.array([*s['position_m'][:2],SEAT+.0925+.002+h]),s['angle_deg'],s['rail_m'])for h in (.055,0,.005,.11)]
        p0=np.array([*s['position_m'][:2],SEAT+.0925+.002]);p1=np.array([*dest['position_m'][:2],SEAT+.0925+.002])
        tasks += [trajectory(s,dest,p0,p1,u)for u in np.linspace(0,1,11)]
        for p,yaw,rail in tasks:
            ik.rail=rail;q=ik.target_yaw(p,yaw,120);d.qpos[qa]=q;mujoco.mj_forward(m,d)
            err=float(np.linalg.norm(d.site('sg_handle_tcp').xpos-p));rot=ik.T[:3,:3].T@d.site('sg_handle_tcp').xmat.reshape(3,3)
            samples.append(dict(error_m=err,angle_error_deg=float(np.degrees(np.arccos(np.clip((np.trace(rot)-1)/2,-1,1)))),rail_m=rail,q=q.tolist()))
        passed=max(x['error_m']for x in samples)<.003 and max(x['angle_error_deg']for x in samples)<3
        cases.append(dict(source=s['id'],destination=dest['id'],passed=passed,samples=samples));print('screen',s['id'],dest['id'],passed,max(x['error_m']for x in samples),flush=True)
    (ROOT/'rail_loop_screen.json').write_text(json.dumps(cases,indent=2));return all(c['passed']for c in cases)

def controller():
    source=(ROOT/'multi_transfer.py').read_text()
    def replace(old,new):
        nonlocal source
        if source.count(old)!=1:raise RuntimeError('Baseline controller changed: '+old[:60])
        source=source.replace(old,new)
    replace('ik=YawIK();initial=',"ik=RailIK(route[0]['rail_m']);initial=")
    replace("transfer_s=max(4.,abs(dest['angle_deg']-source['angle_deg'])/8.)","transfer_s=max(4.,abs(dest['angle_deg']-source['angle_deg'])/8.,1.5*abs(dest['rail_m']-source['rail_m'])/.02)")
    replace("yaw=source['angle_deg'];desired=source_grasp.copy()","yaw=source['angle_deg'];desired=source_grasp.copy();planned_rail=source['rail_m']")
    begin=source.index('                u=smooth(elapsed/transfer_s);p0=')
    end=source.index("            elif phase in ('HOVER'",begin)
    source=source[:begin]+"""                u=smooth(elapsed/transfer_s)
                desired,yaw,planned_rail=trajectory(source,dest,source_grasp,dest_grasp,u)
"""+source[end:]
    replace("yaw=dest['angle_deg'];desired=dest_grasp.copy()","yaw=dest['angle_deg'];desired=dest_grasp.copy();planned_rail=dest['rail_m']")
    replace('                d.ctrl[aids]=ik.target_yaw(desired+bias,yaw)','                ik.rail=planned_rail\n                d.ctrl[aids]=ik.target_yaw(desired+bias,yaw)')
    replace('max_slip=0.;max_rim=',"minimum_rail_clearance=float('inf');max_rail_contact=0.\n    max_slip=0.;max_rim=")
    replace('both=min(forces.values())>.02;',"""rail_force=0.
            guard=m.geom('rail_rack_guard').id
            for ci,c in enumerate(d.contact):
                if guard in (c.geom1,c.geom2):
                    f=np.zeros(6);mujoco.mj_contactForce(m,d,ci,f);rail_force+=max(0.,float(f[0]))
            max_rail_contact=max(max_rail_contact,rail_force)
            if rail_force>.5:failure='rack_rail_collision';break
            both=min(forces.values())>.02;""")
    replace("            if phase=='TRANSFER':\n                transfer_min_clearance=", """            if phase=='TRANSFER':
                size=np.abs(d.geom('rack_lower').xmat.reshape(3,3))@m.geom('rack_lower').size
                xyz=d.geom('rack_lower').xpos;lo,hi=np.array(RAIL_BOUNDS)
                if np.all(xyz[:2]+size[:2]>lo[:2]) and np.all(xyz[:2]-size[:2]<hi[:2]):
                    clearance=bottom-hi[2];minimum_rail_clearance=min(minimum_rail_clearance,clearance)
                    if clearance<.015:failure='rail_overflight_clearance';break
                transfer_min_clearance=""")
    replace("qpos=d.qpos.tolist(),qvel=", "planned_rail_m=planned_rail,rail_contact_N=rail_force,qpos=d.qpos.tolist(),qvel=")
    replace("result=dict(passed=passed,", "result=dict(minimum_rail_clearance_m=None if not np.isfinite(minimum_rail_clearance) else minimum_rail_clearance,max_rail_contact_N=max_rail_contact,passed=passed,")
    (ROOT/'rail_loop_controller.py').write_text(source)
    namespace={'__name__':'rail_loop_controller'};exec(compile(source,str(ROOT/'rail_loop_controller.py'),'exec'),namespace)
    namespace.update(RailIK=RailIK,SEAT=SEAT,trajectory=trajectory,RAIL_BOUNDS=RAIL_BOUNDS)
    return namespace['run']

def run():
    layout=build();result=controller()(hops=11,name=NAME,scene_path=ROOT/f'{NAME}.xml',layout_path=ROOT/f'{NAME}_layout.json',route_ids=layout['route'],initial_fk_correction=True)
    m=mujoco.MjModel.from_xml_path(str(ROOT/f'{NAME}.xml'));rows=json.loads((ROOT/f'{NAME}_trace.json').read_text());q=np.array([r['qpos']for r in rows]);bad=[]
    for j in range(m.njnt):
        if m.jnt_limited[j]:
            a=m.jnt_qposadr[j];lo,hi=m.jnt_range[j];tol=.0005 if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_SLIDE else .002
            if q[:,a].min()<lo-tol or q[:,a].max()>hi+tol:bad.append(m.joint(j).name)
    qa=m.joint('rail_travel').qposadr[0];va=m.joint('rail_travel').dofadr[0];rail=q[:,qa]
    result.update(joint_limit_violations=bad,rail_actual_range_m=[float(rail.min()),float(rail.max())],rail_start_m=float(rail[0]),rail_end_m=float(rail[-1]),rail_peak_speed_m_s=max(abs(r['qvel'][va])for r in rows),scope=layout['scope'])
    stroke_ok=rail.min()<RAIL_LO+.001 and rail.max()>RAIL_HI-.001 and abs(rail[-1]-rail[0])<.001
    if bad or not stroke_ok or result['rail_peak_speed_m_s']>.1:result.update(passed=False,failure='joint_or_stroke_or_speed_verification')
    (ROOT/f'{NAME}.json').write_text(json.dumps(result,indent=2));print('FINAL',result['passed'],result['rail_actual_range_m'],result['rail_peak_speed_m_s'],flush=True);return result['passed']
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--screen',action='store_true');a=p.parse_args();raise SystemExit(0 if (screen()if a.screen else run())else 1)