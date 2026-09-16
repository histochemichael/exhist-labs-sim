"""45-degree folder fixture, preserving the historical 70-degree benchmark."""
import argparse
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import trimesh
import folder_slide as base
from build_scene import ROOT, box, vec

FLOOR = -.0697
CENTER = np.array([.026, -.300, .090])
ANGLE = 45.
a = np.radians(ANGLE)
L = np.array([0., -np.sin(a), np.cos(a)])
W = np.array([1., 0., 0.])
N = np.cross(L, W)
F = np.column_stack([L, W, N])
SCENE = ROOT / 'folder_slide_45.xml'


def configure():
    base.ANGLE = ANGLE
    base.SOURCE = np.array([.010, -.210, .0027])
    base.GRIP_BELOW_TOP_M = .014
    base.TRANSFER_SECONDS = 12.
    base.SUPPORT_ROTATION = False
    base.L, base.W, base.N, base.F, base.CENTER = L, W, N, F, CENTER
    base.SCENE = SCENE


def stand_parts():
    parts = []
    def part(name, p, size, rotation):
        parts.append(dict(name=name, center_m=np.asarray(p).tolist(),
                          half_size_m=list(size), rotation=np.asarray(rotation).tolist()))
    # Backing stops short of both crease lines and leaves the side edges exposed.
    part('backing', CENTER + F @ [0, 0, -.007], [.085, .149, .003], F)
    part('lower_stop', CENTER + F @ [-.108, 0, .0005], [.0015, .185, .0045], F)
    for side in (-1, 1):
        part(f'stop_link_{side}', CENTER+F@[-.07, side*.164, -.007], [.009,.023,.003],F)
        part(f'stop_arm_{side}', CENTER+F@[-.089, side*.180, -.007], [.021,.005,.003],F)
    for j, width in enumerate((-.125, .125)):
        # Two vertical posts joined to horizontal bench feet; all joints need fastening.
        top = CENTER + F @ [.018, width, -.012]
        bottom = FLOOR + .008
        part(f'post_{j}', [top[0], top[1], (top[2]+bottom)/2],
             [.008, .008, (top[2]-bottom)/2], np.eye(3))
        part(f'foot_{j}', [top[0], top[1], FLOOR+.004],
             [.022, .100, .004], np.eye(3))
    return parts


def build():
    configure()
    base.build()
    root = ET.parse(SCENE).getroot()
    world = root.find('worldbody')
    for node in list(world):
        if node.get('name','').startswith('source_holder_'):
            world.remove(node)
    world.find("body[@name='loose_slide']").set('pos',vec(base.SOURCE))
    bottom=base.SOURCE[2]-.0375
    box(world,'source_holder_base',[*base.SOURCE[:2],(FLOOR+bottom)/2],[.020,.012,(bottom-FLOOR)/2],rgba='.30 .42 .55 1')
    for sign in (-1,1):
        box(world,f'source_holder_cheek_{sign}',[base.SOURCE[0],base.SOURCE[1]+sign*.00215,bottom+.004],[.0175,.0015,.004],rgba='.30 .42 .55 1',friction='.3 .005 .0001')
    folder = world.find("body[@name='folder_fixture']")
    folder.remove(folder.find("geom[@name='folder_backing']"))
    for node in list(world):
        if node.get('name', '').startswith('folder_fixture_foot_'):
            world.remove(node)
    for p in stand_parts():
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.array(p['rotation']).ravel())
        box(world, 'stand45_'+p['name'], p['center_m'], p['half_size_m'],
            quat=vec(q), rgba='.18 .40 .43 1', friction='.5 .005 .0001')
    ET.indent(root)
    ET.ElementTree(root).write(SCENE, encoding='utf-8', xml_declaration=True)
    spec = dict(units='m', tilt_from_table_deg=45, bench_z_m=FLOOR,
                folder_center_m=CENTER.tolist(), folder_rotation=F.tolist(),
                parts=stand_parts(), mount='Fixed bench fixture; posts and stop require fasteners',
                nori_handling='Side-edge access reserved; Nori loading/unloading not validated',
                scope='Concept CAD; folder held fixed during slide placement, not a passive folder docking test')
    (ROOT/'folder_stand_45.json').write_text(json.dumps(spec, indent=2))
    # Matching STL concept in millimeters, with the tabletop as z=0.
    meshes = []
    out = ROOT.parent/'hardware/cad/stl'
    for p in stand_parts():
        t = np.eye(4)
        t[:3,:3] = p['rotation']
        t[:3,3] = np.array(p['center_m'])-[0,0,FLOOR]
        mesh = trimesh.creation.box(extents=2*np.array(p['half_size_m']), transform=t)
        mesh.apply_scale(1000)
        mesh.export(out/('Folder-Stand-45-'+p['name']+'.stl'))
        meshes.append(mesh)
    trimesh.util.concatenate(meshes).export(out/'Folder-Stand-45-Assembly.stl')
    return spec


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--build', action='store_true')
    p.add_argument('--name', default='folder45_supported')
    p.add_argument('--row',type=int,default=4)
    p.add_argument('--dt',type=float,default=.0005)
    p.add_argument('--no-close',action='store_true')
    p.add_argument('--offset-mm',type=float,default=0)
    args=p.parse_args()
    configure()
    if args.build: build()
    else: base.run(args.name,not args.no_close,args.row,args.offset_mm,args.dt)








