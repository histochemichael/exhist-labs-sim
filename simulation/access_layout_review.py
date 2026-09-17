"""Render the requested canopy and chest-height correction in the real lab scene."""
import json,math
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from first_pass import Scene,ROOT,kinematics

def main():
    s=Scene();s.robot_pose[:]=[1.15,-.30,math.pi/2];s.sync()
    s.d.qpos[s.q['lift_extension_joint']]=.43;kinematics(s.m,s.d)
    shoulder=float(s.d.xpos[s.m.body('left_shoulder_pitch_link').id,2])
    with mujoco.Renderer(s.m,height=900,width=1280) as renderer:
        c=mujoco.MjvCamera();c.lookat[:]=[1.15,.38,1.23];c.distance=2.75;c.azimuth=115;c.elevation=-15
        opt=mujoco.MjvOption();opt.sitegroup[:]=0;renderer.update_scene(s.d,c,scene_option=opt)
        im=Image.fromarray(renderer.render());d=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',25)
        d.rectangle([0,0,1280,78],fill='#10283c');d.text((18,12),'Special-stain canopy + higher Nori working posture',fill='white',font=font)
        d.text((18,46),f'Bench 800 mm | shoulders {shoulder*1000:.0f} mm | hood underside 1700 mm | concept ventilation',fill='#ffd18c',font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',21))
        im.save(ROOT/'special_canopy_nori_height.png')
    report=dict(table_top_m=.8,shown_lift_m=.43,shoulder_height_m=shoulder,lift_CAD_limits_m=s.m.jnt_range[s.m.joint('lift_extension_joint').id].tolist(),
                carrier_plan_lifts={k:np.asarray(v)[:,0].tolist() for k,v in s.plans.items()},
                scope='Posture/geometry review. Contact validation is separate; full-lab guard remains active.')
    (ROOT/'nori_height_review.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
