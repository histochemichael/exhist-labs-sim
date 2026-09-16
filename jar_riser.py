"""One-inch passive support insert for the actual 24-slide staining jar.

Geometry is parametric. Material compatibility is NOT selected/validated. The
120 g mass is provisional; inserts are free bodies, never actuated or welded.
"""
import json,struct
import numpy as np
from build_scene import ROOT,el,vec

RISER_HEIGHT=.0254
JAR_FLOOR_Z=-.0015
JAR_RIM_Z=.0918
JAR_ORIGIN_Z=.8055
RACK_SEAT_Z=JAR_ORIGIN_Z+JAR_FLOOR_Z+RISER_HEIGHT
LENGTH=.094
WIDTH=.040
CORNER_CLIP=.0062
MASS=.120

def geometry():
    x,y,c=LENGTH/2,WIDTH/2,CORNER_CLIP
    outline=np.array([[-x+c,-y],[x-c,-y],[x,-y+c],[x,y-c],[x-c,y],[-x+c,y],[-x,y-c],[-x,-y+c]])
    vertices=np.array([[a,b,z] for z in (0,RISER_HEIGHT) for a,b in outline])
    faces=[]
    for i in range(1,7):faces.extend([[0,i+1,i],[8,8+i,8+i+1]])
    for i in range(8):
        j=(i+1)%8;faces.extend([[i,j,j+8],[i,j+8,i+8]])
    return vertices,np.array(faces)

def export_design():
    vertices,faces=geometry();assets=ROOT/'assets'
    data=dict(document='24-slide jar / removable one-inch riser / simulation design v1',units='m',
              dimensions_m=[LENGTH,WIDTH,RISER_HEIGHT],corner_clip_m=CORNER_CLIP,
              provisional_mass_kg=MASS,material='UNSPECIFIED - chemical and temperature compatibility unvalidated',
              parts=[dict(name='24-slide jar / 25.4 mm support insert',vertices=vertices.ravel().tolist(),triangles=faces.ravel().tolist(),rgba=[.72,.76,.65,1])])
    (assets/'rack24_riser.json').write_text(json.dumps(data,indent=2))
    # STL coordinates are millimetres; MuJoCo uses the metre mesh below.
    with (assets/'rack24_riser_25p4mm.stl').open('wb') as stream:
        stream.write(b'ExHist 24-slide jar riser | mm | material unvalidated'.ljust(80,b' '));stream.write(struct.pack('<I',len(faces)))
        for face in faces:
            tri=vertices[face]*1000;n=np.cross(tri[1]-tri[0],tri[2]-tri[0]);n/=np.linalg.norm(n)
            stream.write(struct.pack('<12fH',*n,*tri.ravel(),0))
    points=(vertices[:8,:2]*1000).tolist()
    (assets/'rack24_riser_25p4mm.scad').write_text('// Millimetres. Material/chemical compatibility not validated.\nheight_mm = 25.4;\nlinear_extrude(height=height_mm) polygon(points='+json.dumps(points)+');\n')
    return data

def ensure_riser(root,jar):
    """Idempotent free-body placement; no runtime attachment to the jar."""
    assets=root.find('asset');world=root.find('worldbody');vertices,faces=geometry()
    if assets.find("mesh[@name='rack24_one_inch_riser']") is None:
        el(assets,'mesh',name='rack24_one_inch_riser',vertex=vec(vertices.ravel()),face=' '.join(map(str,faces.ravel())))
    name=jar.get('name')+'_riser'
    existing=world.find(f"body[@name='{name}']")
    if existing is not None:world.remove(existing)
    # All current jar orientations are rotations around vertical Z.
    if jar.get('euler') or jar.get('xyaxes'):raise ValueError('Riser placement requires a quaternion jar pose')
    quat=np.fromstring(jar.get('quat','1 0 0 0'),sep=' ')
    import mujoco
    rotation=np.zeros(9);mujoco.mju_quat2Mat(rotation,quat);rotation=rotation.reshape(3,3)
    position=np.fromstring(jar.get('pos','0 0 0'),sep=' ')+rotation@np.array([0,0,JAR_FLOOR_Z])
    body=el(world,'body',name=name,pos=vec(position),quat=vec(quat))
    el(body,'freejoint',name=name+'_free')
    el(body,'inertial',mass=MASS,pos=vec([0,0,RISER_HEIGHT/2]),diaginertia=vec([MASS*(WIDTH**2+RISER_HEIGHT**2)/12,MASS*(LENGTH**2+RISER_HEIGHT**2)/12,MASS*(LENGTH**2+WIDTH**2)/12]))
    el(body,'geom',name=name+'_support',type='mesh',mesh='rack24_one_inch_riser',rgba='.72 .76 .65 1',group='1',density='0',contype='1',conaffinity='1',friction='.5 .005 .0001',solref='.004 1',solimp='.95 .99 .001')
    el(body,'site',name=name+'_top',pos=vec([0,0,RISER_HEIGHT]),size='.002',rgba='0 0 0 0')
    for suffix,z in (('_insert_frame',.045+RISER_HEIGHT),('_rack_seat_frame',JAR_FLOOR_Z+RISER_HEIGHT)):
        site=jar.find(f"site[@name='{jar.get('name')+suffix}']")
        if site is None:site=el(jar,'site',name=jar.get('name')+suffix,size='.002',rgba='0 0 0 0')
        site.set('pos',vec([0,0,z]))
    return body

def update_scene(path):
    import xml.etree.ElementTree as ET
    root=ET.parse(path).getroot();jars=[b for b in root.findall('worldbody/body') if b.get('name','').startswith('special_staining_jar_') and not b.get('name').endswith('_riser')]
    for jar in jars:ensure_riser(root,jar)
    for jar in jars:
        index=jar.get('name').rsplit('_',1)[1]
        site=root.find(f"worldbody/site[@name='special_bath_target_{index}']")
        if site is not None:
            point=np.fromstring(site.get('pos'),sep=' ')
            point[2]=np.fromstring(jar.get('pos'),sep=' ')[2]+JAR_FLOOR_Z+RISER_HEIGHT
            site.set('pos',vec(point))
    ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return len(jars)

if __name__=='__main__':
    export_design();print('Updated operational jars:',update_scene(ROOT/'exhist_operational.xml'))
