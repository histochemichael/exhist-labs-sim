"""Genuine Slide Rack v13 Staining Jar visuals and conservative open contacts.

Planar faces measured from Fusion: cavity flats +/-49 and +/-22 mm,
floor -1.5 mm, rim 91.8 mm; inner vertical corner radius 5 mm.
Corner blocks and an un-beveled rim conservatively reduce usable space.
This contact approximation does not certify a rack insertion trajectory.
"""
import json
import numpy as np
from build_scene import ROOT,el,vec,box

def add_jar(root,name,position,free=True,riser=True):
    source=json.loads((ROOT/'assets/rack24_jar.json').read_text());part=source['parts'][0]
    asset=root.find('asset')
    if asset.find("mesh[@name='rack24_actual_jar']") is None:
        el(asset,'mesh',name='rack24_actual_jar',vertex=vec(part['vertices']),face=' '.join(map(str,part['triangles'])),inertia='shell')
    body=el(root.find('worldbody'),'body',name=name,pos=vec(position))
    if free:el(body,'freejoint',name=name+'_free')
    el(body,'inertial',pos='0 0 .04',mass='.1',diaginertia='.00011 .00016 .00012')
    el(body,'geom',name=name+'_CAD',type='mesh',mesh='rack24_actual_jar',rgba=vec(part['rgba']),contype='0',conaffinity='0',group='1',density='0')
    pieces=[('floor',[0,0,-.0035],[.052,.025,.002])]
    for sign in (-1,1):
        pieces.extend([(f'xwall{sign}',[sign*.0505,0,.04365],[.0015,.025,.04515]),
                       (f'ywall{sign}',[0,sign*.0235,.04365],[.049,.0015,.04515]),
                       (f'xrim{sign}',[sign*.05125,0,.0903],[.00225,.0265,.0015]),
                       (f'yrim{sign}',[0,sign*.02425,.0903],[.049,.00225,.0015])])
        for other in (-1,1):pieces.append((f'corner{sign}_{other}',[sign*.0465,other*.0195,.04515],[.0025,.0025,.04665]))
    for label,p,s in pieces:box(body,name+'_'+label,p,s,[.3,.6,.6,0],group='3',density='0',friction='.5 .005 .0001',solref='.004 1')
    el(body,'site',name=name+'_insert_frame',pos='0 0 .045',size='.002',rgba='1 .5 0 .6')
    if riser:
        from jar_riser import ensure_riser
        ensure_riser(root,body)
    return body

def add_special_jars(root):
    layout=json.loads((ROOT/'layout.json').read_text());station=next(s for s in layout['stations'] if s['name']=='special')
    for k in range(6):add_jar(root,f'special_staining_jar_{k+1}',[station['x']-.375+k*.15,.30,.8055])
    report=dict(source='Slide Rack v13 / root/Staining Jar',count=6,passive_free_bodies=True,
                cavity_flat_dimensions_m=[.098,.044],inside_floor_CAD_z_m=-.0015,rim_CAD_z_m=.0918,
                inner_vertical_corner_radius_m=.005,riser_height_m=.0254,
                assumptions=['Empty jar mass 0.1 kg and inertia are provisional; no fluid model.',
                             'Conservative corner blocks and non-beveled rim; bottom edge blends simplified.',
                             'No validated robot insertion trajectory or loaded-rack retention yet.'])
    (ROOT/'rack24_jar_interface.json').write_text(json.dumps(report,indent=2))
