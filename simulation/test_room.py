import json
import mujoco
from PIL import Image,ImageDraw,ImageFont
from operate_lab import Scene,ROOT,camera
s=Scene(preview=True);hits={};frames=0
for i in range(3500):
    s.lab.tick(.1);s.sync();frames+=1
    for c in s.d.contact:
        if c.dist>=-.001:continue
        a=s.m.geom(c.geom1).name or "";b=s.m.geom(c.geom2).name or ""
        if (a.startswith("lab_") or b.startswith("lab_")) and not (a.startswith("lab_floor") or b.startswith("lab_floor")):
            key=a+" / "+b;hits[key]=min(hits.get(key,0),float(c.dist))
    if s.lab.completed==36:break
assert not hits,hits
assert s.lab.completed==36
with mujoco.Renderer(s.m,height=1080,width=1920) as renderer:
    views=[("furnished_lab",camera()),("lab_work_bays",camera())]
    views[1][1].lookat[:]=[2,-4.1,.8];views[1][1].distance=8;views[1][1].azimuth=85;views[1][1].elevation=-33
    for name,cam in views:
        renderer.update_scene(s.d,cam);img=Image.fromarray(renderer.render())
        draw=ImageDraw.Draw(img);draw.rectangle([0,0,1920,80],fill="#10283c")
        draw.text((24,12),"ExHist Labs | Furnished laboratory environment",font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",28),fill="white")
        draw.text((24,48),"Processing bays / storage / preparation / digital review | Provisional room layout",font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",20),fill="white")
        img.save(ROOT/(name+".png"))
report=dict(compiled=True,sampled_frames=frames,completed_slides=36,new_room_penetrations_m=hits,
    scope="Existing coarse robot colliders against new static room furniture; no human ergonomics, door swing or code validation.")
(ROOT/"room_validation.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
