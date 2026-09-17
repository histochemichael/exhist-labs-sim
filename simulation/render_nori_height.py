"""Same-camera comparison of torso height; no bench or robot rescaling."""
import json,hashlib
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
from operate_lab import Scene,ROOT
from nori_posture import BENCH_VIEW_LIFT_M,height_measurements
from access_suite import source_hashes

def main():
    s=Scene();m=s.m;d=s.d;x=s.lab.x['sorting']
    d.qpos[m.joint('base_y').qposadr[0]]=.5
    camera=mujoco.MjvCamera();camera.lookat[:]=[x,.0,.82];camera.distance=2.35;camera.azimuth=145;camera.elevation=-12
    opt=mujoco.MjvOption();opt.sitegroup[:]=0
    m.vis.headlight.ambient[:]=.25;m.vis.headlight.diffuse[:]=.35
    canvas=Image.new('RGB',(1600,780),'#112a3c');draw=ImageDraw.Draw(canvas)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',25);small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',20)
    results=[]
    with mujoco.Renderer(m,height=650,width=800) as renderer:
        for i,lift in enumerate((.25,BENCH_VIEW_LIFT_M)):
            d.qpos[m.joint('lift_extension_joint').qposadr[0]]=lift;d.qpos[m.joint('lift_middle_joint').qposadr[0]]=lift/2
            mujoco.mj_forward(m,d);r=height_measurements(m,d);results.append(r)
            renderer.update_scene(d,camera,scene_option=opt)
            # Highlight actual shoulder anchors without changing model geometry.
            for side in ('left','right'):
                p=d.body(side+'_shoulder_pitch_link').xpos.copy();g=renderer.scene.geoms[renderer.scene.ngeom]
                mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.011,0,0]),p,np.eye(3).ravel(),np.array([1.,.65,.05,1.]));renderer.scene.ngeom+=1
            canvas.paste(Image.fromarray(renderer.render()),(i*800,62))
            draw.text((i*800+22,20),'PREVIOUS STARTUP' if i==0 else 'CORRECTED STARTUP',font=font,fill='white')
            draw.text((i*800+22,722),f"Table 800 mm | shoulders {r['shoulder_height_m']*1000:.0f} mm",font=small,fill='white')
            draw.text((i*800+22,750),f"Shoulder clearance {r['shoulder_above_bench_m']*1000:+.0f} mm | lift {lift*1000:.0f} mm",font=small,fill='#ffd18c')
    canvas.save(ROOT/'nori_bench_height_correction.png')
    report=dict(before=results[0],after=results[1],view_only_startup_change=True,tables_changed=False,
        contact_controller_unchanged=source_hashes()==json.loads((ROOT/'stability_validation.json').read_text())['source_sha256'],
        scope='Measured CAD/MuJoCo geometry, same scale and camera. Resting arms match across panels to isolate the lift change. Existing runtime grasp/access plans retain their own lift targets.')
    (ROOT/'nori_bench_height_correction.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
