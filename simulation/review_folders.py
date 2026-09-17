"""Render evidence of filling, both flap motion, and the separate finished bench."""
import json,math
import mujoco
from PIL import Image,ImageDraw,ImageFont
from first_pass import Scene,ROOT,camera
from folder_finish import FOLDER_CENTERS

def render(allow_unvalidated_storyboard=False):
    if not allow_unvalidated_storyboard:raise RuntimeError('Use --storyboard explicitly to inspect unvalidated folder candidate motions.')
    s=Scene(guarded=False);seen=set();font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',23)
    with mujoco.Renderer(s.m,height=720,width=1280) as r:
        for i in range(6000):
            s.tick(.2);j=s.lab.jobs[0];phase=j.route[j.index].name if j.index<len(j.route) else 'finished'
            angle=s.d.qpos[s.q[j.id+'_folder_hinge_L']]
            name=None;title=''
            if j.folder_loaded==len(j.slides) and not j.folder_closed and angle<.01:name='filled';title=f'{j.id}: all {len(j.slides)} assigned slides seated; ready to close'
            if .9<angle<2.2 and not j.folder_closed:name='closing';title='Nori follows the left flap through its crease; right flap closes next'
            if j.folder_closed and j.location=='sendout':name='closed';title='Both folder flaps closed; ready for Nori pickup'
            if s.lab.completed==36:name='finished_bench';title='Four closed folders / 36 slides delivered to the separate finished-work bench'
            if name and name not in seen:
                x=FOLDER_CENTERS[j.slot] if j.location=='sendout' else 2.39
                c=camera(x,True);c.lookat[:]=[x,.5,.86];c.distance=1.15;c.elevation=-60;c.azimuth=95
                if name=='finished_bench':c.lookat[:]=[-2.8,-3.87,.86];c.distance=2.65;c.azimuth=-90;c.elevation=-50
                r.update_scene(s.d,camera=c);im=Image.fromarray(r.render());d=ImageDraw.Draw(im)
                d.rectangle((0,0,1280,78),fill='#10283c');d.text((18,10),title,fill='white',font=font)
                d.text((18,44),'First-pass kinematic workflow; cardboard bending, retention and closing forces unvalidated',fill='#ffce82',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',17))
                im.save(ROOT/f'folder_{name}.png');seen.add(name);print(name,round(s.lab.time,1),flush=True)
            if s.lab.completed==36:break
    assert s.lab.completed==36 and seen=={'filled','closing','closed','finished_bench'},seen
    print(json.dumps(dict(completed_slides=s.lab.completed,seconds=s.lab.time,folders=[dict(id=j.id,closed=j.folder_closed,slides=j.folder_loaded,bench=j.location,slot=j.slot) for j in s.lab.jobs]),indent=2))

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--storyboard',action='store_true');args=p.parse_args();render(args.storyboard)
