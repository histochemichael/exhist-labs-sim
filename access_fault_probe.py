import sys,numpy as np,mujoco
from equipment_access import Access
a=Access(sys.argv[1] if len(sys.argv)>1 else 'quincy')
while not a.step():pass
print(a.phase,a.d.time,flush=True)
print('arm actual',a.d.qpos[a.qa],'command',a.qtarget,'effort',a.d.actuator_force[a.ai],flush=True)
print('tip',a.d.site_xpos[a.sid],'handle',a.d.site_xpos[a.hsid],'target',a.desired(float(a.d.qpos[a.jq])+.006)[0],flush=True)
print('jointlast',a.trace[-1],flush=True)
contacts={}
for i,c in enumerate(a.d.contact):
    f=np.zeros(6);mujoco.mj_contactForce(a.m,a.d,i,f)
    if np.linalg.norm(f)>.05:
        key=(a.m.body(a.m.geom_bodyid[c.geom1]).name,a.m.body(a.m.geom_bodyid[c.geom2]).name)
        item=contacts.setdefault(key,[0.,0.,a.m.geom(c.geom1).name,a.m.geom(c.geom2).name]);item[0]+=float(f[0]);item[1]=min(item[1],float(c.dist))
print('contacts',contacts,flush=True)
