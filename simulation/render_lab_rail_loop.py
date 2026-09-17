"""Render measured loop physics in the integrated room; never drive live objects."""
import argparse
import hashlib
import json
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from build_scene import ROOT
from integrate_rail_loop import REFERENCE,MODEL
from lab_rail_loop import camera


def render(preview=False):
    import imageio.v2 as imageio
    layout=json.loads((REFERENCE/'rail_loop_layout.json').read_text())
    if preview:
        path=REFERENCE/'rail_loop_trace.json'
    else:
        report=json.loads((ROOT/'rail_loop_lab_validation.json').read_text())
        assert report['passed'] and report['model_sha256']==hashlib.sha256(MODEL.read_bytes()).hexdigest()
        path=ROOT/'rail_loop_lab_trace.json'
        assert report['trace_sha256']==hashlib.sha256(path.read_bytes()).hexdigest(), 'Trace no longer matches validation'
    rows=json.loads(path.read_text());times=np.array([r['t'] for r in rows])
    m=mujoco.MjModel.from_xml_path(str(MODEL));d=mujoco.MjData(m)
    opt=mujoco.MjvOption();opt.geomgroup[3:]=0;opt.sitegroup[:]=0
    from rail_loop_visuals import apply as apply_visuals
    apply_visuals(m)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',23);small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',17)
    fps=15;speed=4
    samples=[float(times[len(times)//2])] if preview else np.arange(times[0],times[-1],speed/fps)
    output=ROOT/'ExHist-Live-LeHisto-Rail-Loop.mp4'
    writer=None if preview else imageio.get_writer(str(output),fps=fps,codec='libx264',quality=7,macro_block_size=2)
    observed=[]
    try:
        with mujoco.Renderer(m,height=720,width=1280) as main,mujoco.Renderer(m,height=216,width=384) as inset:
            for k,t in enumerate(samples):
                r=rows[min(int(np.searchsorted(times,t)),len(rows)-1)]
                d.qpos[:]=r['qpos'];d.qvel[:]=r['qvel'];d.ctrl[:]=r['ctrl'];mujoco.mj_forward(m,d)
                # Begin with the whole room, then show the active station closely.
                wide=not preview and t<8
                cam=camera(layout,'overview' if wide else 'station')
                main.update_scene(d,cam,scene_option=opt);frame=Image.fromarray(main.render());draw=ImageDraw.Draw(frame)
                draw.rectangle((0,0,1280,79),fill='#10283c')
                title='ExHist Labs | layout alignment preview' if preview else 'ExHist Labs | live-physics LeHisto loop | 4x recorded replay'
                draw.text((16,9),title,font=font,fill='white')
                rail=float(d.joint('rail_travel').qpos[0])*1000
                draw.text((16,42),f"Bath {r['source']} -> {r['destination']} | {r['phase']} | {r['t']:.1f}s | rail {rail:+.1f} mm",font=small,fill='#d5edf5')
                if not wide:
                    inset.update_scene(d,'gripper',scene_option=opt);frame.paste(Image.fromarray(inset.render()),(880,440))
                    draw.rectangle((880,416,1264,440),fill='#10283c');draw.text((890,420),'LeHisto gripper camera',font=small,fill='white')
                draw.rectangle((0,681,1280,720),fill='#10283c')
                draw.text((16,689),'11 baths | one-inch inserts | actual bench support | other equipment held; Nori handoffs not validated',font=small,fill='#ffd08b')
                if writer:writer.append_data(np.asarray(frame))
                if preview:frame.save(ROOT/'rail_loop_lab_alignment.png')
                if not preview and r['source']==4 and r['phase']=='TRANSFER' and not observed:
                    frame.save(ROOT/'rail_loop_lab_crossover.png');observed.append(True)
                if not preview and k==0:frame.save(ROOT/'rail_loop_lab_overview.png')
                if k%150==0:print('Rendered',k,'/',len(samples),flush=True)
            if writer:
                draw=ImageDraw.Draw(frame);draw.rectangle((0,610,850,657),fill='#143e2b')
                draw.text((16,622),'PASS: 11/11 transfers; rack returned, released and supported.',font=font,fill='white')
                frame.save(ROOT/'rail_loop_lab_complete.png')
                for _ in range(fps*2):writer.append_data(np.asarray(frame))
            # Separate static room-overview image for clear layout inspection.
            if preview:
                main.update_scene(d,camera(layout,'overview'),scene_option=opt)
                Image.fromarray(main.render()).save(ROOT/'rail_loop_lab_room_alignment.png')
    finally:
        if writer:writer.close()
    if not preview:
        meta=dict(file=output.name,fps=fps,speed=speed,frames=len(samples)+fps*2,duration_s=len(samples)/fps+2,
                  physics_model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
                  trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),scope='Recorded fresh physics in integrated scene, not an animation controller. Other equipment is held context.')
        (ROOT/'rail_loop_lab_video.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--preview',action='store_true');a=p.parse_args();render(a.preview)
