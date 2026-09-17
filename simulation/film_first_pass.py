"""Three-minute first-pass lab review, using the previous Leica film's motion.

Visible top caption distinguishes kinematic playback from contact physics.
Camera changes are editorial; the robot's head camera is fixed to its head.
"""
import json,time
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from first_pass import Scene,ROOT,camera

def render(allow_unvalidated_storyboard=False):
    if not allow_unvalidated_storyboard:
        raise RuntimeError('Equipment access is not validated. Use --storyboard only for an explicitly labeled candidate-motion review.')
    s=Scene(guarded=False);ffmpeg=s.machines.f.imageio_ffmpeg
    fps=12;frames=180*fps;demo_seconds=json.loads((ROOT/'first_pass_validation.json').read_text())['simulated_seconds']+6
    path=ROOT/'ExHist-Candidate-Storyboard-Not-Validated.mp4'
    writer=ffmpeg.write_frames(str(path),(1280,720),fps=fps,codec='libx264',quality=8,
        pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=2,output_params=['-preset','fast','-movflags','+faststart'])
    writer.send(None);font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',23);small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',17)
    overview=camera();started=time.monotonic();seen=set();shots=[]
    with mujoco.Renderer(s.m,height=720,width=1280) as main,mujoco.Renderer(s.m,height=220,width=390) as head:
        for frame in range(frames):
            if s.lab.completed<36:s.tick(demo_seconds/frames)
            tr=s.lab.transport;active=[j for j in s.lab.jobs if j.holds in ('routine_a','routine_b') and j.state=='PROCESSING']
            focus=None;cam=overview;label='Complete lab workflow'
            finishing=next((j for j in s.lab.jobs if j.state=='PROCESSING' and j.route[j.index].name in ('package','close_folder')),None)
            if frame>fps*5 and frame<frames-fps*5:
                if finishing:
                    from folder_finish import FOLDER_CENTERS
                    cam=camera(FOLDER_CENTERS[finishing.slot],True);cam.lookat[:]=[FOLDER_CENTERS[finishing.slot],.45,.94];cam.distance=1.8;cam.elevation=-42
                    label=f'{finishing.id} | {finishing.folder_loaded}/{len(finishing.slides)} slides in folder | '+finishing.route[finishing.index].name
                elif active and (not tr or int(frame/fps)%16>=6):
                    focus=active[(int(frame/fps)//16)%len(active)];station=focus.holds;phase=focus.route[focus.index].name
                    cam=camera(s.lab.x[station],True);cam.lookat[:]=[s.lab.x[station],.57,1.1];cam.distance=2.15;cam.elevation=-40
                    if phase in ('coverslip','cure'):
                        cam.lookat[:]=[s.lab.x[station]-.62,.5,1.12];cam.distance=1.05;cam.elevation=-28;cam.azimuth=110
                    label=s.machines.labels[station]
                elif tr:
                    cam=camera(s.rx,True);cam.lookat[:]=[s.rx,s.robot_pose[1]+.25,1.0];cam.distance=2.6;cam.elevation=-28
                    label=tr['job']+' | Nori: '+tr['source']+' -> '+tr['target']
                else:
                    working=next((j for j in s.lab.jobs if j.state=='PROCESSING'),None)
                    if working:cam=camera(s.lab.x[working.location],True);label=working.id+' | '+working.route[working.index].name
            if s.lab.completed==36:
                cam=camera();cam.lookat[:]=[-2.8,-3.79,.91];cam.distance=2.8;cam.azimuth=-90;cam.elevation=-45;label='Finished-work bench | four closed folders | 36 slides'
            main.update_scene(s.d,camera=cam);im=Image.fromarray(main.render());draw=ImageDraw.Draw(im)
            draw.rectangle((0,0,1280,87),fill='#10283c')
            draw.text((18,9),f'ExHist Labs | UNVALIDATED STORYBOARD | {s.lab.completed}/36 logical slides',fill='white',font=font)
            draw.text((18,41),label[:130],fill='#d4e6ef',font=small)
            draw.text((18,64),'Kinematic review / compressed timing / contact and hardware validation separate',fill='#ffce82',font=small)
            if s.held:
                head.update_scene(s.d,camera='nori_head_left');inset=Image.fromarray(head.render());im.paste(inset,(874,475))
                draw=ImageDraw.Draw(im);draw.rectangle((874,451,1264,475),fill='#10283c');draw.text((882,454),'Nori head - left view of rack interaction',fill='white',font=small)
            if focus:
                phase=focus.route[focus.index].name
                if phase not in seen and (phase!='coverslip' or 'turn specimen up' in label or 'leftward insertion' in label):
                    name=f'first_pass_review_{phase}.png';im.save(ROOT/name);seen.add(phase);shots.append(name)
            if s.held and 'head' not in seen:im.save(ROOT/'first_pass_review_head.png');seen.add('head')
            writer.send(np.asarray(im))
            if frame%(fps*15)==0:print('Video',frame,'/',frames,'demo',round(s.lab.time,1),flush=True)
    writer.close()
    assert s.lab.completed==36
    report=dict(video=str(path),frames=frames,fps=fps,duration_s=180,completed_slides=36,physical_success=False,
                source_video=s.machines.meta['source'],render_seconds=time.monotonic()-started,proof_frames=shots)
    (ROOT/'first_pass_video_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--storyboard',action='store_true');args=p.parse_args()
    render(args.storyboard)
