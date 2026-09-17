from operate_lab import Scene,ROOT,camera
import mujoco
from PIL import Image,ImageDraw,ImageFont
s=Scene(preview=True)
with mujoco.Renderer(s.m,height=1080,width=1920) as renderer:
    for i in range(180):
        s.lab.tick(.1);s.sync()
        if i in [0,35,100,175]:
            cam=camera(s.lab.x["sorting"],True);cam.distance=2.7;cam.elevation=-30
            renderer.update_scene(s.d,cam);img=Image.fromarray(renderer.render())
            draw=ImageDraw.Draw(img);draw.rectangle([0,0,1920,95],fill="#10283c")
            draw.text((24,14),f"ExHist robot motion trials | t={s.lab.time:.1f}s",fill="white",font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",28))
            draw.text((24,55),"Source CAD joints moving | Provisional tool points | No physical rack grasp yet",fill="white",font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",22))
            img.save(ROOT/f"robot_motion_{i}.png")
