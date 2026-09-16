"""CAD-sliced convex contact pieces for Nori's unchanged original left claw.

Whole-finger convex hulls fill the hooked profile. Four-millimetre axial and
distal-width cells retain that profile; within-cell concavities remain conservative.
No visual mesh, finger dimensions or joint transforms are changed.
"""
import copy,struct
import numpy as np
from build_scene import el,vec
from gripper_physics import SOURCE

def clipped(poly,axis,bound,keep_above):
    out=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        ia=(a[axis]>=bound) if keep_above else (a[axis]<=bound)
        ib=(b[axis]>=bound) if keep_above else (b[axis]<=bound)
        if ia:out.append(a)
        if ia!=ib:out.append(a+(b-a)*(bound-a[axis])/(b[axis]-a[axis]))
    return np.array(out)

def replace(root,wrist):
    asset=root.find('asset');counts={}
    for body in wrist.findall('body'):
        visual=next((g for g in body.findall('geom') if g.get('mesh','').startswith('gripper_') and g.get('contype')=='0'),None)
        if visual is None:continue
        for g in list(body.findall('geom')):
            if g.get('type')=='box' or '_access_contact' in g.get('name',''):body.remove(g)
        name=visual.get('mesh');mesh=asset.find(f"mesh[@name='{name}']")
        # The original wrist-pitch collision box fills its rotating gear socket.
        # Scope the mount-overlap exemption to proximal gear cells only. Distal
        # finger/housing, all external equipment and arm contacts remain enabled.
        gear=el(body,'body',name=body.get('name')+'_proximal_gear_contacts')
        el(root.find('contact'),'exclude',name=body.get('name')+'_internal_gear_socket',body1='left_wrist_pitch_link',body2=gear.get('name'))
        raw=__import__('pathlib').Path(mesh.get('file')).read_bytes();n=struct.unpack_from('<I',raw,80)[0]
        tris=np.array([struct.unpack_from('<9f',raw,84+50*i+12) for i in range(n)]).reshape(-1,3,3)*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
        flat=tris.reshape(-1,3);axis=int(np.argmax(np.ptp(flat,axis=0)));lo=flat[:,axis].min();hi=flat[:,axis].max();edges=np.linspace(lo,hi,int(np.ceil((hi-lo)/.004))+1)
        count=0
        width_axis=int(np.argsort(np.ptp(flat,axis=0))[-2])
        width_edges=np.linspace(flat[:,width_axis].min(),flat[:,width_axis].max(),int(np.ceil(np.ptp(flat[:,width_axis])/.004))+1)
        for low,high in zip(edges[:-1],edges[1:]):
            # Distal CAD cross-sections vary substantially across finger width.
            # A single whole-width hull fills those recesses. Partition both
            # length and width, without changing the visual CAD or friction.
            spans=list(zip(width_edges[:-1],width_edges[1:])) if high>.055 else [(width_edges[0],width_edges[-1])]
            for wlow,whigh in spans:
                pieces=[]
                for tri in tris:
                    if tri[:,axis].max()<low or tri[:,axis].min()>high or tri[:,width_axis].max()<wlow or tri[:,width_axis].min()>whigh:continue
                    p=clipped(tri,axis,low,True)
                    if len(p):p=clipped(p,axis,high,False)
                    if len(p):p=clipped(p,width_axis,wlow,True)
                    if len(p):p=clipped(p,width_axis,whigh,False)
                    if len(p):pieces.extend(p)
                if not pieces:continue
                v=np.unique(np.round(pieces,9),axis=0)
                if len(v)<4 or np.linalg.matrix_rank(v-v.mean(0))<3:continue
                key=f'{name}_access_contact_{count}';el(asset,'mesh',name=key,vertex=vec(v.ravel()))
                g=copy.deepcopy(visual);g.set('name',key);g.set('mesh',key);g.set('contype','2');g.set('conaffinity','1');g.set('group','3');g.set('rgba','0 .8 .3 0');g.set('friction','.6 .005 .0001');g.set('solref','.004 1');(gear if high<=.019 else body).append(g);count+=1
        counts[name]=count
    return counts
