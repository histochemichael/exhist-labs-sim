"""Scoped geometric clearance checks for the cinematic promo, not dynamics."""
import numpy as np
import mujoco
from pose_control import rotation_error
from first_pass import kinematics

class Clearance:
    def __init__(self,m,d):
        self.m=m;self.d=d;self.fromto=np.zeros(6)
        self.head=[g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name=='lift_top_link' and m.geom_pos[g,2]>.12]
        self.arms={}
        for side in ('left','right'):
            self.arms[side]=[g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(side+'_') and ('wheel' not in m.body(m.geom_bodyid[g]).name) and m.geom_type[g] in (mujoco.mjtGeom.mjGEOM_BOX,)]
        self.arms['right'] += [m.geom('lehisto_part_00').id,m.geom('lehisto_part_14').id]
        self.arms['right']=list(set(self.arms['right']+[g for g in range(m.ngeom) if m.geom(g).name.startswith('lehisto_part_')]))
        for side in ('left','right'):
            self.arms[side]=list(set(self.arms[side]+[g for g in range(m.ngeom) if m.geom_group[g]==1 and m.body(m.geom_bodyid[g]).name.startswith(side+'_') and 'wheel' not in m.body(m.geom_bodyid[g]).name]))
        self.drawer=[g for g in range(m.ngeom) if m.geom(g).name=='routine_a_top' or m.geom(g).name in ('promo_drawer_part_461','promo_drawer_part_467','promo_drawer_part_468','promo_drawer_part_469','promo_drawer_part_470','promo_drawer_part_472')]
        door=m.body('quincy_1_passive_door').id
        self.door=[g for g in range(m.ngeom) if m.geom_bodyid[g]==door and m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.mesh(m.geom_dataid[g]).name in ('passive_quincy_6','passive_quincy_7')]

    def distances(self,side,door=False):
        pairs=[(a,h) for a in self.arms[side] for h in self.head]
        if door:pairs += [(a,h) for a in self.arms[side] for h in self.door]
        elif side=='left':pairs += [(a,h) for a in self.arms[side] for h in self.drawer]
        elif side=='right':pairs += [(a,h) for a in self.arms[side] for h in self.drawer if self.m.geom(h).name in ('routine_a_top','promo_drawer_part_472')]
        return np.array([mujoco.mj_geomDistance(self.m,self.d,a,b,.25,self.fromto) for a,b in pairs])

    def tighten(self,site,names,p,R):
        m=self.m;d=self.d;qa=np.array([m.joint(n).qposadr[0] for n in names]);va=np.array([m.joint(n).dofadr[0] for n in names]);jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);sid=m.site(site).id
        for _ in range(60):
            kinematics(m,d);ep=np.asarray(p)-d.site(site).xpos;er=rotation_error(R,d.site(site).xmat.reshape(3,3))
            if np.linalg.norm(ep)<2e-6 and np.linalg.norm(er)<2e-5:break
            mujoco.mj_jacSite(m,d,jp,jr,sid)
            if 'lift_extension_joint' in names:
                full=m.joint('lift_extension_joint').dofadr[0];half=m.joint('lift_middle_joint').dofadr[0];jp[:,full]+=.5*jp[:,half];jr[:,full]+=.5*jr[:,half]
            J=np.vstack((jp[:,va],.1*jr[:,va]));e=np.r_[ep,.1*er];d.qpos[qa]+=np.clip(J.T@np.linalg.solve(J@J.T+np.eye(6)*1e-8,e),-.04,.04)
            for n,i in zip(names,qa):d.qpos[i]=np.clip(d.qpos[i],*m.joint(n).range)
        kinematics(m,d)
        return float(np.linalg.norm(ep)),float(np.linalg.norm(er))

    def path(self,start,end,side,door=True,margin=.004):
        """Find a clearance-checked intermediate joint pose; no dynamic claim."""
        m=self.m;d=self.d;rng=np.random.default_rng(29)
        names=[side+'_'+n+'_joint' for n in ('shoulder_pitch','shoulder_roll','bicep_yaw','elbow_pitch','forearm_yaw','wrist_pitch','wrist_roll')]
        qa=np.array([m.joint(n).qposadr[0] for n in names]);bounds=np.array([m.joint(n).range for n in names])
        def edge(a,b,n):
            for u in np.linspace(0,1,n):
                d.qpos[:]=a+(b-a)*u;kinematics(m,d)
                if self.distances(side,door).min()<margin:return False
            return True
        if edge(start,end,101):d.qpos[:]=start;return [start,end]
        for trial in range(700):
            mid=(start+end)/2;mid[qa]=np.clip(mid[qa]+rng.normal(0,.25+.15*(trial//100),len(qa)),bounds[:,0]+.001,bounds[:,1]-.001)
            if edge(start,mid,15) and edge(mid,end,15) and edge(start,mid,101) and edge(mid,end,101):
                d.qpos[:]=start;print('Clearance waypoint',side,'trial',trial,flush=True);return [start,mid,end]
        d.qpos[:]=start;raise RuntimeError(('No clear joint transition',side))

    def solve(self,site,names,p,R,door=False,attempts=8,margin=.007):
        m=self.m;d=self.d;qa=np.array([m.joint(n).qposadr[0] for n in names]);ref=d.qpos[qa].copy()
        bounds=np.array([m.joint(n).range for n in names]);lo=bounds[:,0]+1e-5;hi=bounds[:,1]-1e-5
        if 'lift_extension_joint' in names:lo[names.index('lift_extension_joint')]=.48
        side='left' if 'left' in site else 'right';rng=np.random.default_rng(19);best=None;metric=1e8
        def residual(x):
            d.qpos[qa]=x;kinematics(m,d)
            ep=d.site(site).xpos-p;er=rotation_error(R,d.site(site).xmat.reshape(3,3))
            gaps=self.distances(side,door)
            return np.r_[ep,.10*er,4*np.minimum(gaps-margin,0),.00004*(x-ref)]
        for attempt in range(attempts):
            seed=np.clip(ref,lo+1e-6,hi-1e-6) if not attempt else rng.uniform(lo,hi)
            x=seed.copy()
            for _ in range(150):
                e=residual(x)
                if np.linalg.norm(e[:3])<.0004 and np.linalg.norm(e[3:6])<.0005 and np.linalg.norm(e[6:-len(x)])<.0001:break
                j=np.column_stack([(residual(x+np.eye(len(x))[k]*1e-4)-e)/1e-4 for k in range(len(x))])
                free=np.ones(len(x),bool);dx=np.zeros(len(x))
                for _active in range(len(x)):
                    jj=j[:,free];dx[:]=0;dx[free]=-np.linalg.solve(jj.T@jj+np.eye(free.sum())*1e-5,jj.T@e)
                    blocked=((x<=lo+1e-6)&(dx<0))|((x>=hi-1e-6)&(dx>0))
                    if not np.any(blocked&free):break
                    free[blocked]=False
                    if not free.any():break
                dx=np.clip(dx,-.14,.14)
                improved=False
                for scale in (1,.5,.2,.05):
                    xx=np.clip(x+dx*scale,lo,hi);ee=residual(xx)
                    if ee@ee<e@e:x=xx;improved=True;break
                if not improved:break
            e=residual(x);pe=np.linalg.norm(e[:3]);re=np.linalg.norm(e[3:6])/.1;gap=self.distances(side,door).min()
            score=pe+.1*re+max(0,margin-gap)*5
            if score<metric:metric=score;best=(x.copy(),pe,re,gap)
            if pe<.0007 and re<.012 and gap>margin-.0003:break
        d.qpos[qa]=best[0];kinematics(m,d)
        return best
