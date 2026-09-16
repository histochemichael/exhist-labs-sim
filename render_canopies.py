from operate_lab import Scene,ROOT,camera
import mujoco
from PIL import Image,ImageDraw,ImageFont
s=Scene();s.sync()
cam=camera((s.lab.x["routine_a"]+s.lab.x["routine_b"])/2,True)
cam.lookat[2]=1.55;cam.distance=4.4;cam.elevation=-13
with mujoco.Renderer(s.m,height=1080,width=1920) as renderer:
    renderer.update_scene(s.d,cam)
    img=Image.fromarray(renderer.render());draw=ImageDraw.Draw(img)
    draw.rectangle([0,0,1920,86],fill="#10283c")
    draw.text((24,14),"ExHist | Stainer canopy hoods",font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",28),fill="white")
    draw.text((24,51),"Concept layout | 49 cm static clearance | Ventilation performance not modeled",font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",21),fill="white")
    img.save(ROOT/"stainer_canopies.png")
