"""Proposed stand-off pull interface; mounting/thermal suitability unvalidated."""
import numpy as np
from build_scene import el,vec

def add(moving,center,R):
    original=moving.find("geom[@name='access_handle']");original.set('name','access_oem_handle')
    grasp=center+R@np.array([0,-.035,0.]);q=np.zeros(4)
    import mujoco
    mujoco.mju_mat2Quat(q,R.ravel())
    b=el(moving,'body',name='quincy_proposed_offset_pull',pos=vec(center),quat=vec(q))
    el(b,'inertial',mass='.08',pos='0 -.018 0',diaginertia='.00009 .00009 .000025')
    # Open-center frame avoids the OEM handle; attachment to the machine still
    # needs a measured mounting design. This is deliberately a separate concept.
    for name,p,s in [('bar',[0,-.035,0],[.005,.003,.05]),('top',[0,-.007,.045],[.005,.028,.005]),('bottom',[0,-.007,-.045],[.005,.028,.005])]:
        el(b,'geom',name='access_handle' if name=='bar' else 'quincy_adapter_'+name,type='cylinder' if name=='bar' else 'box',pos=vec(p),size='.005 .05' if name=='bar' else vec(s),rgba='.2 .58 .61 1',group='1',contype='1',conaffinity='2',density='0',friction='.6 .005 .0001',solref='.004 1')
    return grasp
