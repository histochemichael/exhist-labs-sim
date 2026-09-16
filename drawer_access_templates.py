"""Transfer a verified arm branch between parallel drawer pulls, then verify FK."""
import json
import numpy as np
import mujoco
import xml.etree.ElementTree as ET
import equipment_access as e

def build_template(kind):
    _,source=e.build('st_load');root,target=e.build(kind)
    plan=json.loads((e.ROOT/'access_st_load_plan.json').read_text())
    dz=target['handle_closed_world'][2]-source['handle_closed_world'][2]
    path=np.array(plan['path']);path[:,-1]+=dz
    plan.update(kind=kind,path=path.tolist(),template='st_load',height_offset_m=dz)
    robot=root.find("worldbody/body[@name='mobile_chassis']");h=np.array(target['handle_closed_world'])
    robot.set('pos',e.vec(h+[plan['lateral'],-plan['stand'],-h[2]+.002]));robot.set('quat','.707106781 0 0 .707106781')
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    qa=[m.joint(n).qposadr[0] for n in plan['names']];jq=m.joint(target['joint']).qposadr[0]
    for t,q in zip(np.linspace(0,1,len(path)),path):
        d.qpos[qa]=q;d.qpos[m.joint('lift_middle_joint').qposadr[0]]=q[-1]/2;d.qpos[jq]=t*target['open_target'];mujoco.mj_forward(m,d)
        assert np.linalg.norm(d.site_xpos[m.site(e.SITE).id]-d.site_xpos[m.site('access_handle_frame').id])<.001
        assert .3<q[-1]<.69
    (e.ROOT/f'access_{kind}_plan.json').write_text(json.dumps(plan,indent=2));print(kind,'verified template',dz,flush=True)

if __name__=='__main__':build_template('cv_load')
