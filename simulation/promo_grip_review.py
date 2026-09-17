"""Inspect the release from the CAD-mounted camera and an external close-up."""
import json
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from promo_demo import MODEL,OUT,FPS,exact_grip_playback

m=mujoco.MjModel.from_xml_path(str(MODEL));d=mujoco.MjData(m);s=np.load(OUT/'promo_states.npz');report=json.loads((OUT/'promo_manifest.json').read_text());mid=m.body('B01_leica_rack').mocapid[0]
m.light_castshadow[:]=0;m.light_diffuse[:]=.15;m.vis.headlight.ambient[:]=.43;m.vis.headlight.diffuse[:]=.55
opt=mujoco.MjvOption();opt.sitegroup[:]=0;opt.geomgroup[3:]=0
events=[e for e in report['events'] if e['label'] in ('LOWER INTO STAINER CONTAINER','RELEASE RACK IN CONTAINER','RIGHT GRIPPER CLEAR','OPEN JAWS ABOVE CONTAINER')]
sheet=Image.new('RGB',(1280,360*len(events)))
with mujoco.Renderer(m,height=360,width=640) as renderer:
    for row,e in enumerate(events):
        i=round((e['start']+.92*(e['end']-e['start']))*FPS);d.qpos[:]=s['qpos'][i];d.mocap_pos[:]=[0,0,-10];d.mocap_pos[mid]=s['rack'][i];d.mocap_quat[mid]=s['rack_quat'][i];exact_grip_playback(m,d,s,float(i))
        camera=mujoco.MjvCamera();camera.lookat[:]=[-2.36,.18,1.045];camera.distance=.47;camera.azimuth=94;camera.elevation=-28
        for col,view in enumerate((camera,'promo_nori_gripper')):
            renderer.update_scene(d,view,scene_option=opt);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.text((10,10),e['label']+(' | CAD CAMERA (PROVISIONAL FOV)' if col else ' | EXTERIOR'),fill='white',stroke_width=1,stroke_fill='black');sheet.paste(im,(col*640,row*360))
sheet.save(OUT/'gripper-release-review.jpg',quality=90)
print(OUT/'gripper-release-review.jpg')
