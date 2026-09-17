"""Frame-rate CAD-solid intersection audit of the promo interaction corridor."""
import sys,json,time,hashlib
from pathlib import Path
import numpy as np
import mujoco
# Install the portable checker dependencies from requirements-promo-checks.txt.
import manifold3d as mf
from promo_demo import MODEL,OUT,FPS,kinematics,exact_grip_playback

def audit(step=1):
    m=mujoco.MjModel.from_xml_path(str(MODEL));d=mujoco.MjData(m);states=np.load(OUT/'promo_states.npz');manifest=json.loads((OUT/'promo_manifest.json').read_text());action=next(e['start'] for e in manifest['events'] if e['label']=='FADE TO WHITE')
    states=dict(states) # Avoid repeated NPZ decompression during the frame audit.
    machine_start=next(e['start'] for e in manifest['events'] if e['label']=='STAINING RUN STARTS')
    names=[m.geom(i).name for i in range(m.ngeom)];bn=[m.body(m.geom_bodyid[i]).name for i in range(m.ngeom)]
    def under(g,root):
        b=m.geom_bodyid[g]
        while b:
            if m.body(b).name==root:return True
            b=m.body_parentid[b]
        return False
    nori=[g for g in range(m.ngeom) if m.geom_group[g]==1 and ((bn[g].startswith(('left_','right_')) and 'wheel' not in bn[g]) or names[g].startswith('lehisto_part_'))]
    rack=[m.geom('B01_leica_rack_'+str(i)).id for i in (0,1)]
    machine=[g for g in range(m.ngeom) if m.geom_group[g]==1 and 'envelope' not in names[g] and any(under(g,b) for b in ('quincy_1','quincy_2','quincy_3','routine_a_workstation'))]
    head=[g for g in range(m.ngeom) if bn[g]=='lift_top_link' and m.geom_pos[g,2]>.12]
    tables=[g for g in range(m.ngeom) if names[g] in ('baking_top','routine_a_top','sorting_top')]
    starm=[g for g in machine if any(under(g,'routine_a_'+n) for n in ('j71_ST_ARM_SHOULDER_deg','j72_ST_ARM_ELBOW_deg','j73_ST_ARM_WRIST_deg'))]
    fixedst=[g for g in machine if under(g,'routine_a_workstation') and g not in starm and not under(g,'routine_a_j70_ST_ARM_Z_mm')]
    groups=[('nori',nori,machine+head+tables),('rack',rack,machine+tables),('stainer',starm,fixedst)]
    # Rack/fork contact is intentional; other arm geometry is still audited.
    groups[1]=('rack',rack,[g for g in machine+tables if not under(g,'routine_a_j73_ST_ARM_WRIST_deg')])
    left=[g for g in nori if bn[g].startswith('left_')];right=[g for g in nori if g not in left]
    groups.append(('cross_arm',left,right))
    groups.append(('rack_robot',rack,head+left))
    ids=sorted(set(g for _,a,b in groups for g in a+b));solids={};bounds={};quality=[];fallbacks=[]
    for g in ids:
        if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
            k=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k]+m.mesh_vertnum[k]]*1000;f=m.mesh_face[m.mesh_faceadr[k]:m.mesh_faceadr[k]+m.mesh_facenum[k]]
            mesh=mf.Mesh(v.astype(np.float32),f.astype(np.uint32),tolerance=.0001);mesh.merge();solid=mf.Manifold(mesh);lo=v.min(0);hi=v.max(0)
        elif m.geom_type[g]==mujoco.mjtGeom.mjGEOM_BOX:
            hi=m.geom_size[g]*1000;lo=-hi;solid=mf.Manifold.cube((2*hi).tolist(),True)
        elif m.geom_type[g]==mujoco.mjtGeom.mjGEOM_CYLINDER:
            radius,h=m.geom_size[g,:2]*1000;solid=mf.Manifold.cylinder(2*h,radius,radius,32,True);hi=np.array([radius,radius,h]);lo=-hi
        else:continue
        if solid.status()!=mf.Error.NoError:
            if g in nori:
                solid=mf.Manifold.hull_points(v.astype(np.float64));fallbacks.append(dict(geom=names[g] or str(g),body=bn[g],method='conservative convex hull of non-manifold robot visual mesh'))
            else:raise RuntimeError(('Invalid CAD solid',names[g],bn[g],str(solid.status())))
        solids[g]=solid;bounds[g]=((lo+hi)/2,(hi-lo)/2);quality.append(dict(geom=names[g] or str(g),body=bn[g],volume_mm3=solid.volume()))
    groups=[(name,[g for g in a if g in solids],[g for g in b if g in solids]) for name,a,b in groups]
    mid=m.body('B01_leica_rack').mocapid[0];hits=[];tests=0;start=time.monotonic()
    for frame in range(0,1800,step):
        t=frame/30;st=t*action/54.5 if t<54.5 else action+t-54.5;f=st*FPS;i=min(int(f),len(states['qpos'])-1);j=min(i+1,len(states['qpos'])-1);u=np.clip(f-i,0,1)
        d.qpos[:]=(1-u)*states['qpos'][i]+u*states['qpos'][j];d.mocap_pos[mid]=(1-u)*states['rack'][i]+u*states['rack'][j];rq=(1-u)*states['rack_quat'][i]+u*states['rack_quat'][j];d.mocap_quat[mid]=rq/np.linalg.norm(rq);exact_grip_playback(m,d,states,f)
        bb={};transformed={}
        for g,(center,half) in bounds.items():
            R=d.geom_xmat[g].reshape(3,3);c=R@center+d.geom_xpos[g]*1000;h=abs(R)@half;bb[g]=(c-h,c+h)
        def solid_at(g):
            if g not in transformed:transformed[g]=solids[g].transform(np.column_stack([d.geom_xmat[g].reshape(3,3),d.geom_xpos[g]*1000]))
            return transformed[g]
        label=str(states['label'][i])
        for group,moving,obstacles in groups:
            if group=='stainer' and st<machine_start:continue
            for a in moving:
                alo,ahi=bb[a]
                for b in obstacles:
                    blo,bhi=bb[b]
                    if np.any(ahi<blo+.00001) or np.any(bhi<alo+.00001):continue
                    if group=='nori' and ('gripper' in bn[a] or 'idler' in bn[a]) and ('_pull_' in names[b] or 'handle' in names[b]):continue
                    tests+=1;vol=float((solid_at(a)^solid_at(b)).volume())
                    if vol>.02:hits.append(dict(frame=frame,t=t,source_t=st,phase=label,group=group,a=names[a] or f'{bn[a]}:{a}',b=names[b] or f'{bn[b]}:{b}',volume_mm3=vol))
        if frame%120==0:print('Solid audit',frame,'/1800','hits',len(hits),'seconds',round(time.monotonic()-start,1),flush=True)
    report=dict(frame_step=step,frames_checked=len(range(0,1800,step)),boolean_tests=tests,intersections=hits,solid_count=len(solids),robot_visual_hull_fallbacks=fallbacks,scope='Tessellated CAD solids in the oven/stainer corridor: Nori arm/gripper versus equipment, head and three worktops; rack versus equipment except intentional wrist fork; stainer horizontal arms/fork versus fixed workstation. Intentional left-claw/handle contacts excluded. Not forces or continuous-time certification.',physical_success=False)
    report['scope']+=' Also checks left versus right arm/gripper and rack versus head/left arm.'
    report['scene_sha256']=hashlib.sha256(MODEL.read_bytes()).hexdigest();report['states_sha256']=hashlib.sha256((OUT/'promo_states.npz').read_bytes()).hexdigest()
    (OUT/f'promo_solid_audit_step{step}.json').write_text(json.dumps(report,indent=2));print('Done',len(hits),'intersections',flush=True)
    return report

if __name__=='__main__':audit(int(sys.argv[1]) if len(sys.argv)>1 else 1)
