"""Render and measure the screenshot-reported errors, not hardware success."""
import json,math
import numpy as np
import mujoco
from PIL import Image
from first_pass import Scene,ROOT,camera,carrier_side,grasp_offset,carrier_rotation,kinematics


def mesh_world_bounds(m,d,body):
    ids=[]
    for i in range(m.nbody):
        p=i
        while p and p!=body:p=m.body_parentid[p]
        if p==body:ids.append(i)
    points=[]
    for i in range(m.ngeom):
        if m.geom_bodyid[i] not in ids:continue
        if m.geom_type[i]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        mesh=m.geom_dataid[i];a=m.mesh_vertadr[mesh];n=m.mesh_vertnum[mesh]
        v=m.mesh_vert[a:a+n]@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i]
        points.extend(v)
    p=np.asarray(points);return p.min(0),p.max(0)


def main(layout_only=False):
    s=Scene(guarded=False);m=s.m;d=s.d
    shots={};rows=[]
    with mujoco.Renderer(m,height=720,width=1100) as renderer:
        def shot(name,x,y,z,distance=1.6):
            c=camera(x,True);c.lookat[:]=[x,y,z];c.distance=distance;c.elevation=-32;c.azimuth=90
            renderer.update_scene(d,camera=c);Image.fromarray(renderer.render()).save(ROOT/name);shots[name]=s.lab.time
        shot('correction_sorting.png',-6.5,.38,.93,2.2)
        shot('correction_special.png',1.15,.45,.94,1.5)
        shot('correction_blocks.png',3.35,.43,.89,1.2)
        for i in range(1,9):
            lo,hi=mesh_world_bounds(m,d,m.body('qc_cassette_'+str(i)).id)
            rows.append(dict(body='qc_cassette_'+str(i),lo=lo.tolist(),hi=hi.tolist(),gap=float(lo[2]-.8)))
        for _ in range(0 if layout_only else 12000):
            s.tick(.1);tr=s.lab.transport
            if tr:
                j=next(j for j in s.lab.jobs if j.id==tr['job']);kind=j.carrier
                if s.held and kind not in shots:
                    c=camera(s.rx,True);c.lookat[:]=d.site_xpos[m.site('nori_left_carrier_grasp' if carrier_side(kind)=='left' else 'nori_right_handle_groove').id];c.distance=.42;c.elevation=-20;c.azimuth=110
                    renderer.update_scene(d,camera=c);name='correction_grasp_'+kind+'.png';Image.fromarray(renderer.render()).save(ROOT/name);shots[kind]=s.lab.time
            if s.lab.completed==36:break
    (ROOT/'review_correction_measurements.json').write_text(json.dumps(dict(shots=shots,static_blocks=rows,physical_success=False),indent=2))
    print(json.dumps(dict(shots=shots,blocks=rows,completed=s.lab.completed),indent=2))

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--layout-only',action='store_true');args=p.parse_args();main(args.layout_only)
