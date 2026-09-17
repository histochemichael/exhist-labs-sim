"""Render recorded physics states; replay poses are NOT the live controller."""
import sys,json,math
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
from equipment_access import Access,KINDS
from build_scene import ROOT
from access_suite import source_hashes
sys.path.insert(0,r'C:/Users/Owner/.codex/visualizations/2026/09/07/01a079ae-3b7f-7562-a6b4-d9df80acd01c/st5020-demo/tools')
import imageio_ffmpeg

LABELS=dict(quincy='Quincy incubator | 70 degree swing',s60='NanoZoomer S60 | 200 mm slide door',st_load='Leica ST5020 | load drawer',st_unload='Leica ST5020 | unload drawer',cv_load='Leica CV5030 | load drawer',ts_drawer='Leica TS5025 | transfer drawer')

def lighting(a):
    a.m.light_diffuse[:]=.45;a.m.light_ambient[:]=.1;a.m.light_specular[:]=.05
    a.m.vis.headlight.diffuse[:]=.35;a.m.vis.headlight.ambient[:]=.15;a.m.vis.headlight.specular[:]=.03
    # A fixed, PROPOSED head-side bracket view. The earlier centered wide view
    # is occluded by the working forearm. This is not calibrated hardware.
    cid=a.m.camera('nori_head_wide').id;a.m.cam_pos[cid]=[.06,.115,.32]
    yaw=math.radians(-10);pitch=math.radians(35)
    right=np.array([math.sin(yaw),-math.cos(yaw),0.]);up=np.array([math.sin(pitch)*math.cos(yaw),math.sin(pitch)*math.sin(yaw),math.cos(pitch)])
    R=np.column_stack([right,up,np.cross(right,up)]);quat=np.zeros(4);mujoco.mju_mat2Quat(quat,R.ravel());a.m.cam_quat[cid]=quat

def camera(a,azimuth=40):
    c=mujoco.MjvCamera();c.lookat[:]=a.closed+[.08,0,.05];c.distance=1.8;c.azimuth=azimuth;c.elevation=-20
    return c

def angle_sheet():
    a=Access('st_unload');lighting(a);states=np.load(ROOT/'access_st_unload_states.npz');i=int(np.argmin(abs(states['time']-25)))
    a.d.qpos[:]=states['qpos'][i];mujoco.mj_forward(a.m,a.d);sheet=Image.new('RGB',(1280,900));opt=mujoco.MjvOption();opt.sitegroup[:]=0
    with mujoco.Renderer(a.m,height=450,width=640) as renderer:
        for k,az in enumerate((40,125,220,310)):
            renderer.update_scene(a.d,camera(a,az),scene_option=opt);im=Image.fromarray(renderer.render());ImageDraw.Draw(im).text((12,12),str(az),fill='red');sheet.paste(im,((k%2)*640,(k//2)*450))
    sheet.save(ROOT/'access_camera_review.png')

def film(proof=False):
    writer=imageio_ffmpeg.write_frames(str(ROOT/'ExHist-Equipment-Access-Contact-Review.mp4'),(1280,720),fps=15,codec='libx264',quality=8,pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=2,output_params=['-preset','fast','-movflags','+faststart']);writer.send(None)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',23);small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',17);count=0
    for kind in KINDS:
        report=json.loads((ROOT/f'access_{kind}_validation.json').read_text());assert report['validation_ok'] and report['source_sha256']==source_hashes()
        a=Access(kind);lighting(a);states=np.load(ROOT/f'access_{kind}_states.npz');opt=mujoco.MjvOption();opt.sitegroup[:]=0
        with mujoco.Renderer(a.m,height=720,width=1280) as renderer,mujoco.Renderer(a.m,height=216,width=384) as head:
            for i,(q,phase,t) in enumerate(zip(states['qpos'],states['phase'],states['time'])):
                if proof and not (phase=='HOLD' and (i==0 or states['phase'][i-1]!='HOLD')):continue
                # Visual replay of saved dynamic results, not a second physics run.
                a.d.qpos[:]=q;mujoco.mj_forward(a.m,a.d)
                renderer.update_scene(a.d,camera(a),scene_option=opt);im=Image.fromarray(renderer.render());dr=ImageDraw.Draw(im)
                dr.rectangle([0,0,1280,88],fill='#10283c');dr.text((16,8),'ExHist Labs | CONTACT-DRIVEN EQUIPMENT ACCESS',font=font,fill='white')
                dr.text((16,39),LABELS[kind]+' | '+phase+' | '+f'{t:.1f} s',font=small,fill='#d5edf5')
                dr.text((16,64),'3x recorded-physics replay | fixed dock | provisional forces / hardware validation separate',font=small,fill='#ffd08b')
                head.update_scene(a.d,'nori_head_wide',scene_option=opt);im.paste(Image.fromarray(head.render()),(880,484));dr=ImageDraw.Draw(im);dr.rectangle([880,460,1264,484],fill='#10283c');dr.text((888,462),'Proposed fixed head-side camera',font=small,fill='white')
                if phase=='HOLD' and (i==0 or states['phase'][i-1]!='HOLD'):im.save(ROOT/f'access_verified_{kind}.png')
                writer.send(np.asarray(im));count+=1
        print('Rendered',kind,flush=True)
    writer.close();(ROOT/'access_video_validation.json').write_text(json.dumps(dict(frames=count,fps=15,duration_s=count/15,physics_playback_speed=3,source_sha256=source_hashes(),recording_interval_sim_s=.2,scope='Recorded simulation states; no independent kinematic equipment animation.'),indent=2))

if __name__=='__main__':
    if '--angles' in sys.argv:angle_sheet()
    else:film('--proof' in sys.argv)
