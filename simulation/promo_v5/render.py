"""Portable render-only replay of the accepted 60-second promo; no hardware IO."""
import argparse,hashlib,json
from pathlib import Path
from contextlib import ExitStack
import numpy as np
import mujoco
import imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def smooth(u):
    u=np.clip(u,0,1);return float(u*u*(3-2*u))
def font(size,bold=False):
    for name in [f'C:/Windows/Fonts/arial{"bd" if bold else ""}.ttf',f'DejaVuSans{"-Bold" if bold else ""}.ttf']:
        try:return ImageFont.truetype(name,size)
        except OSError:pass
    return ImageFont.load_default()
def camera(v):
    c=mujoco.MjvCamera();c.lookat[:]=v[:3];c.distance,c.azimuth,c.elevation=v[3:];return c

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--check',action='store_true',help='Render five frames and compare with published MP4');args=ap.parse_args()
    evidence=json.loads((ROOT/'evidence/portability.json').read_text())
    assert evidence['packaged_scene_sha256']==sha(ROOT/'scene.xml')
    assert evidence['playback_sha256']==sha(ROOT/'playback.npz')
    m=mujoco.MjModel.from_xml_path(str(ROOT/'scene.xml'));d=mujoco.MjData(m);s=dict(np.load(ROOT/'playback.npz'))
    m.light_castshadow[:]=0;m.light_diffuse[:]=.15;m.light_ambient[:]=0
    m.vis.headlight.ambient[:]=.43;m.vis.headlight.diffuse[:]=.55
    opt=mujoco.MjvOption();opt.sitegroup[:]=0;opt.geomgroup[3:]=0
    normal,title,small=font(17),font(28,True),font(12,True)
    logo=Image.open(ROOT/'exhist-logo.png').convert('RGB');logo.thumbnail((720,720),Image.Resampling.LANCZOS)
    endcard=Image.new('RGB',(1280,720),'white');endcard.paste(logo,((1280-logo.width)//2,(720-logo.height)//2))
    selected=[0,675,1245,1650,1770];frames=selected if args.check else range(1800)
    writer=None;rendered={}
    if not args.check:
        writer=imageio_ffmpeg.write_frames(str(ROOT/'rerendered.mp4'),(1280,720),fps=30,codec='libx264',quality=8,pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=2,output_params=['-preset','fast','-movflags','+faststart']);writer.send(None)
    with ExitStack() as stack:
        mainview=stack.enter_context(mujoco.Renderer(m,720,1280));detail=stack.enter_context(mujoco.Renderer(m,216,384))
        for i in frames:
            t=i/30;d.qpos[:]=s['qpos'][i];d.mocap_pos[:]=s['mocap_pos'][i];d.mocap_quat[:]=s['mocap_quat'][i]
            mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d);mujoco.mj_camlight(m,d)
            mainview.update_scene(d,camera(s['camera'][i]),scene_option=opt);im=Image.fromarray(mainview.render());draw=ImageDraw.Draw(im)
            if t<7:draw.text((30,28),'ExHist Labs',font=title,fill='white',stroke_width=1,stroke_fill='#163444')
            draw.text((28,687),'STAGED SIMULATION  |  COMPRESSED TIMING',font=normal,fill='white',stroke_width=1,stroke_fill='#163444')
            if 47<=t<54.5:
                draw.rounded_rectangle((28,628,307,668),radius=8,fill='#0b5548');draw.text((43,638),'ST5020  |  STAINING STARTED',font=normal,fill='white')
            if s['inset_alpha'][i]>0:
                detail.update_scene(d,camera(s['detail_camera'][i]),scene_option=opt)
                panel=Image.new('RGB',(388,244),'#13899a');panel.paste(Image.fromarray(detail.render()),(2,26))
                pd=ImageDraw.Draw(panel);pd.rectangle((2,2,385,25),fill='#102d3d')
                label='RACK PICKUP' if s['inset_kind'][i]==1 else 'STAINER DROP-OFF'
                pd.text((10,4),label+'  |  GRIPPER CLOSE-UP',font=small,fill='white')
                box=(868,24,1256,268);im.paste(Image.blend(im.crop(box),panel,float(s['inset_alpha'][i])),box[:2])
            if 54.5<=t<55.5:im=Image.blend(im,Image.new('RGB',im.size,'white'),smooth(t-54.5))
            elif 55.5<=t<56:im=Image.new('RGB',im.size,'white')
            elif 56<=t<57:im=Image.blend(Image.new('RGB',im.size,'white'),endcard,smooth(t-56))
            elif t>=57:im=endcard.copy()
            if writer:writer.send(np.asarray(im))
            else:rendered[i]=np.asarray(im).copy()
            if i%150==0:print(f'{t:.1f}/60 s',flush=True)
    if writer:writer.close()
    if args.check:
        gen=imageio_ffmpeg.read_frames(str(ROOT/'ExHist-Labs-Promo-60s.mp4'),pix_fmt='rgb24');meta=next(gen);errors={};count=0
        for i,raw in enumerate(gen):
            if i in rendered:errors[str(i)]=float(np.mean(abs(np.frombuffer(raw,np.uint8).reshape(720,1280,3).astype(float)-rendered[i].astype(float))))
            count=i+1
        assert count==1800 and meta['duration']==60 and meta['fps']==30
        assert max(errors.values())<5,'Packaged render differs from delivered video'
        from PIL import ImageSequence
        gif=Image.open(ROOT/'preview.gif');loop=gif.info.get('loop');duration=sum(f.info.get('duration',0) for f in ImageSequence.Iterator(gif))
        assert loop==0 and duration==60000
        result=dict(passed=True,decoded_frames=count,video_duration_s=60,gif_duration_ms=duration,gif_infinite_loop=True,
            frame_mean_absolute_rgb_errors=errors,scene_sha256=sha(ROOT/'scene.xml'),playback_sha256=sha(ROOT/'playback.npz'),gif_sha256=sha(ROOT/'preview.gif'),
            scope='Packaged replay pixel comparison at scanner, rack pickup, stainer drop-off, departure/fade and logo; encoding differences allowed. Not a new physics validation.')
        (ROOT/'evidence/packaged-render-check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
