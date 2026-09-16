"""Close-up inspection of recorded handle contacts; no physics edits."""
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from equipment_access import Access
from render_access_review import lighting,camera
from build_scene import ROOT

sheet=Image.new('RGB',(1280,900));font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',20)
for row,kind in enumerate(('quincy','s60')):
    a=Access(kind);lighting(a);s=np.load(ROOT/f'access_{kind}_states.npz');i=np.flatnonzero(s['phase']=='HOLD')[0];a.d.qpos[:]=s['qpos'][i];mujoco.mj_forward(a.m,a.d)
    with mujoco.Renderer(a.m,height=450,width=640) as renderer:
        c=camera(a);c.lookat[:]=a.d.site_xpos[a.hsid];c.distance=.42;c.elevation=-12
        opt=mujoco.MjvOption();opt.sitegroup[:]=0
        for col,view in enumerate((c,'nori_head_wide')):
            renderer.update_scene(a.d,view,scene_option=opt);im=Image.fromarray(renderer.render());dr=ImageDraw.Draw(im);dr.rectangle((0,0,640,30),fill='#10283c');dr.text((8,5),kind+(' | external handle detail' if col==0 else ' | proposed fixed head-side camera'),font=font,fill='white');sheet.paste(im,(col*640,row*450))
sheet.save(ROOT/'access_visual_contact_qa.png')
