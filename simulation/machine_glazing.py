"""Tinted inspection glazing, independent of equipment mechanics.

Split only visual CAD meshes that accidentally merged glass with solid parts.
Source CAD and existing OBJ assets remain untouched. No collision or joint
geometry is replaced. Alpha is an inspection preference, not a material spec.
"""
import collections
import copy
import json
import re
import xml.etree.ElementTree as ET
import numpy as np
from build_scene import ROOT, ASSETS, el, vec

GLAZING = {
    'PHOTO_curved_hood_left': (.55, .78, .83, .18),
    'PHOTO_curved_canopy': (.55, .78, .83, .18),
    'PHOTO_curved_hood_right': (.55, .78, .83, .18),
    'Rounded transparent hood': (.55, .78, .83, .18),
    'Transparent left return for gentle trim': (.55, .78, .83, .18),
    'Transparent right return for gentle trim': (.55, .78, .83, .18),
    'TS_clear_front_top': (.55, .78, .83, .18),
    'TS_clear_side_210': (.55, .78, .83, .18),
    'TS_clear_side_348': (.55, .78, .83, .18),
}


def glazing_color(name):
    # Exact body names: do not classify hinges, hood edges or slide coverslips.
    return GLAZING.get(name.rsplit('/', 1)[-1])


def grouped_parts(articulated):
    src = json.loads((ASSETS/'leica_workstation_handled.json').read_text())
    vertices = np.concatenate([np.asarray(p['vertices']).reshape(-1, 3) for p in src['parts']])
    offset = np.r_[(vertices.min(0)[:2]+vertices.max(0)[:2])/2, vertices.min(0)[2]]
    origins = {'fixed': offset}
    groups = collections.defaultdict(list)
    owners = {}
    if articulated:
        owners = json.loads((ROOT/'machine_articulation.json').read_text())['part_owners']
        kin = json.loads((ASSETS/'leica_kinematics.json').read_text())
        for i, joint in enumerate(j for j in kin['joints'] if j['kind'] != 0):
            key = 'j'+str(i)+'_'+re.sub(r'[^a-zA-Z0-9_]', '_', joint['name'])
            origins[key] = np.asarray(joint['origin'])
    for part in src['parts']:
        color = tuple(round(c, 3) for c in part['rgba'])
        owner = owners.get(part['name'], 'fixed')
        shell = any(s in part['name'] for s in ['transparent_hood', '02_OUTER_ENCLOSURE', 'FIXED CHASSIS', 'FIXED LID', 'COVER', 'HOUSING'])
        groups[(owner, color, shell) if articulated else color].append(part)
    for i, (key, parts) in enumerate(groups.items()):
        yield ('art_leica_' if articulated else 'leica_workstation_')+str(i), parts, origins[key[0] if articulated else 'fixed']


def write_mesh(name, parts, origin):
    path = ASSETS/(name+'.obj')
    with path.open('w') as stream:
        index = 1
        for part in parts:
            vertices = np.asarray(part['vertices']).reshape(-1, 3)-origin
            faces = np.asarray(part['triangles']).reshape(-1, 3)
            stream.writelines('v '+vec(v)+'\n' for v in vertices)
            stream.writelines('f '+' '.join(str(int(i)+index) for i in face)+'\n' for face in faces)
            index += len(vertices)
    return path.relative_to(ROOT).as_posix()


def apply(root, articulated=True):
    """Idempotent visual-only edit; hidden historical layers stay hidden."""
    assets = root.find('asset')
    existing_assets = {a.get('name') for a in assets}
    parents = {g: b for b in root.iter() for g in b if g.tag == 'geom'}
    rows = []
    for mesh, parts, origin in grouped_parts(articulated):
        glass = [p for p in parts if glazing_color(p['name'])]
        if not glass:
            continue
        solid = [p for p in parts if not glazing_color(p['name'])]
        colors = {glazing_color(p['name']) for p in glass}
        assert len(colors) == 1, (mesh, colors)
        color = vec(next(iter(colors)))
        targets = [g for g in parents if g.get('mesh') == mesh and float(g.get('rgba', '1 1 1 1').split()[3]) > 0]
        if not targets:
            continue
        for g in targets:
            assert g.get('contype') == '0' and g.get('conaffinity') == '0' and g.get('density') == '0', 'Glazing must be massless, non-contact visual geometry'
        if solid:
            for suffix, subset in [('opaque', solid), ('glazing', glass)]:
                name = mesh+'_'+suffix
                file = write_mesh(name, subset, origin)
                if name not in existing_assets:
                    el(assets, 'mesh', name=name, file=file, inertia='shell')
                    existing_assets.add(name)
            for g in targets:
                parent = parents[g]
                transparent = copy.deepcopy(g)
                transparent.set('name', (g.get('name') or parent.get('name')+'_'+mesh)+'_glazing_shell')
                transparent.set('mesh', mesh+'_glazing')
                transparent.set('rgba', color)
                g.set('mesh', mesh+'_opaque')
                parent.append(transparent)
        else:
            for g in targets:
                g.set('rgba', color)
        rows.append(dict(mesh=mesh, instances=len(targets), split=bool(solid), rgba=color,
                         glass_parts=[p['name'] for p in glass]))
    return rows


def main():
    report = {}
    for filename, articulated in [('exhist.xml', False), ('exhist_operational.xml', True)]:
        path = ROOT/filename
        root = ET.parse(path).getroot()
        before = ET.tostring(root)
        report[filename] = apply(root, articulated)
        if ET.tostring(root) != before:
            ET.indent(root)
            ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    (ROOT/'machine_glazing_update.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
