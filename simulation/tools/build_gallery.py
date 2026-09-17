"""Generate actual MuJoCo renders and exact tessellated STL asset exports."""
import collections, copy, json, struct, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco, numpy as np
from PIL import Image, ImageDraw, ImageFont
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
OUT=ROOT/'docs/images'
CATALOG=[
 ('lehisto','LeHisto v38','Rail-mounted SO-101 arm, dual-groove parallel gripper and cameras.'),
 ('quincy','Quincy 10GC','Three baking incubators; swing-door access is tested separately.'),
 ('leica_workstation_handled','Leica ST5020 / CV5030','Two routine-staining/coverslipping workstations with articulated mechanisms.'),
 ('s60','Hamamatsu NanoZoomer S60','Two scanner and PC stations; cassette insertion remains development work.'),
 ('rack24','24-slide handled rack','User-tested top handle; distinct from Leica input and output carriers.'),
 ('rack24_jar','24-slide staining container','Matching vessel used in the eleven-bath special-staining layout.'),
 ('rack24_riser','One-inch support insert','25.4 mm passive riser raises the handle; material compatibility unvalidated.'),
 ('leica_rack_handled','Handled Leica input rack','Updated Leica rack with an accessible carrying handle.'),
 ('output_magazine','CV5030 output magazine','Coverslipped-slide output carrier; Nori extraction is not yet validated.'),
 ('scanner_cassette','S60 scanner cassette','Scanner-specific slide carrier; no automatic compatibility assumed.'),
 ('slide_folder','20-place slide folder','Send-out folder with modeled flaps; physical closing/carrying unvalidated.'),
 ('cassette_block','Embedded cassette block','Modeled tissue cassette with paraffin/tissue geometry for QC staging.')
]
def font(size):
    for p in ['C:/Windows/Fonts/arial.ttf','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']:
        if Path(p).is_file():return ImageFont.truetype(p,size)
    return ImageFont.load_default()
def vec(x):return ' '.join(f'{float(v):.9g}' for v in x)
def render(m,d,look,distance,path,az=135,el=-25):
    opt=mujoco.MjvOption();opt.geomgroup[3:]=0;opt.sitegroup[:]=0
    m.vis.headlight.ambient[:]=.50;m.vis.headlight.diffuse[:]=.60
    if m.nlight:m.light_diffuse[:]=.15
    m.light_castshadow[:]=0
    cam=mujoco.MjvCamera();cam.lookat[:]=look;cam.distance=distance;cam.azimuth=az;cam.elevation=el
    with mujoco.Renderer(m,height=600,width=1000) as r:
        r.update_scene(d,cam,scene_option=opt);im=Image.fromarray(r.render());im.save(path)
def stl(path,vertices,faces):
    tris=vertices[faces]*1000
    normal=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
    norm=np.linalg.norm(normal,axis=1);normal/=np.maximum(norm[:,None],1e-30)
    record=np.zeros(len(tris),dtype=[('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
    record['n']=normal;record['v']=tris
    path.write_bytes(b'ExHist tessellation | millimetres | visualization, not certified manufacture'.ljust(80,b' ')+struct.pack('<I',len(tris))+record.tobytes())
def asset(key,title,description):
    src=json.loads((ROOT/'assets'/f'{key}.json').read_text())
    vertices=[];faces=[];groups=collections.defaultdict(list);offset=0
    for part in src['parts']:
        v=np.array(part['vertices']).reshape(-1,3);f=np.array(part['triangles']).reshape(-1,3)
        if key in ('s60','scanner_cassette'):v=v@np.array([[1,0,0],[0,0,-1],[0,1,0]]).T
        vertices.append(v);faces.append(f+offset);offset+=len(v)
        groups[tuple(part['rgba'])].append((v,f))
    whole=np.concatenate(vertices);tri=np.concatenate(faces);lo=whole.min(0);hi=whole.max(0)
    modeldir=ROOT/'models/assets';meshdir=modeldir/'meshes';meshdir.mkdir(parents=True,exist_ok=True)
    stldir=ROOT/'cad/stl';stldir.mkdir(parents=True,exist_ok=True);stl(stldir/f'{key}.stl',whole,tri)
    root=ET.Element('mujoco',model=title+' | static asset inspection')
    ET.SubElement(root,'compiler',angle='radian')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='1000',offheight='600')
    ET.SubElement(visual,'headlight',ambient='.4 .4 .4',diffuse='.6 .6 .6')
    assets=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody')
    ET.SubElement(assets,'texture',type='skybox',builtin='gradient',rgb1='.92 .95 .97',rgb2='.70 .78 .84',width='512',height='3072')
    for i,(rgba,chunks) in enumerate(groups.items()):
        name=f'{key}_{i}';obj=meshdir/(name+'.obj');n=1
        with obj.open('w',encoding='utf-8') as stream:
            for v,f in chunks:
                for pt in v:stream.write('v '+vec(pt)+'\n')
                for face in f:stream.write('f '+' '.join(str(int(j)+n) for j in face)+'\n')
                n+=len(v)
        ET.SubElement(assets,'mesh',name=name,file='meshes/'+obj.name,inertia='shell')
        ET.SubElement(world,'geom',type='mesh',mesh=name,rgba=vec(rgba),contype='0',conaffinity='0',density='0')
    ET.indent(root);xml=modeldir/(key+'.xml');ET.ElementTree(root).write(xml,encoding='utf-8',xml_declaration=True)
    m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    render(m,d,(lo+hi)/2,max(hi-lo)*1.8,OUT/f'asset-{key}.png',az=45 if key=='output_magazine' else 135,el=-25)
    return dict(id=key,title=title,description=description,document=src.get('document'),units_stl='mm',units_xml='m',
                mesh=f'cad/stl/{key}.stl',xml=f'models/assets/{key}.xml',image=f'docs/images/asset-{key}.png',
                dimensions_m=(hi-lo).tolist(),part_count=len(src['parts']))
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    catalog=[]
    for row in CATALOG:
        catalog.append(asset(*row));print('Asset:',row[0],flush=True)
    # Room renders use the current integrated model, not old first-pass screenshots.
    from lab_rail_loop import camera
    m=mujoco.MjModel.from_xml_path(str(ROOT/'exhist_rail_loop.xml'));d=mujoco.MjData(m)
    mujoco.mj_forward(m,d)
    layout=json.loads((ROOT/'rail_loop_reference/rail_loop_layout.json').read_text())
    stations=json.loads((ROOT/'layout.json').read_text())['stations']
    c=camera(layout,'overview');render(m,d,c.lookat,c.distance,OUT/'lab-overview.png',c.azimuth,c.elevation)
    for station in stations:
        c=camera(layout,station_x=station['x'])
        # Include canopy while keeping the table readable.
        R=np.array(layout['world_rotation']);t=np.array(layout['world_translation'])
        c.lookat[:]=R.T@(np.array([station['x'],.50,1.15])-t)
        render(m,d,c.lookat,3.15,OUT/('station-'+station['name']+'.png'),205,-22)
        print('Station:',station['name'],flush=True)
    # Nori shown in actual current room stance.
    b=d.body('lab_context_nori_mobile_base').xpos
    render(m,d,b+np.array([0,0,.68]),2.25,OUT/'asset-nori.png',225,-15)
    # Full furniture overview, including support bays/chairs/PCs/doors/windows.
    c=camera(layout,'overview');render(m,d,c.lookat+np.array([0,0,.3]),c.distance,OUT/'asset-room.png',225,-42)
    (ROOT/'docs/asset-catalog.json').write_text(json.dumps(catalog,indent=2))
    thumbs=[OUT/'lab-overview.png']+[OUT/('station-'+x['name']+'.png') for x in stations]+[OUT/f'asset-{x[0]}.png' for x in CATALOG]+[OUT/'asset-nori.png']
    sheet=Image.new('RGB',(1000,((len(thumbs)+2)//3)*222),'#eff4f7')
    draw=ImageDraw.Draw(sheet)
    for i,p in enumerate(thumbs):
        im=Image.open(p);im.thumbnail((326,190));x=(i%3)*333;y=(i//3)*222
        sheet.paste(im,(x,y));draw.text((x+5,y+193),p.stem,font=font(13),fill='#142b3c')
    sheet.save(OUT/'gallery-contact-sheet.jpg',quality=85)
if __name__=='__main__':main()
