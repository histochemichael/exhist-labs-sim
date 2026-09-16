"""Static scientific layout audit; not a proof of collision-free loaded motion."""
import json,math
from PIL import Image,ImageDraw,ImageFont
from build_scene import ROOT

r=json.loads((ROOT/'lehisto_lab_reach_audit.json').read_text());rows=[x for x in r['rows'] if 'jar' in x];p=r['profile'];origin=rows[0]['shoulder_world_m']
im=Image.new('RGB',(1200,760),'#f4f7f8');d=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',23);small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',17)
def xy(x,y):return (80+(x-.60)*900,620-(y-.10)*900)
cx,cy=xy(*origin[:2]);d.rectangle((25,92,875,704),fill='#dde8ed',outline='#8ca2ac',width=2)
for radius,color,width in ((.330,'#9aadb3',2),(.320,'#468578',3),(.210,'#468578',3),(.200,'#9aadb3',2)):
    rr=radius*900;d.ellipse([cx-rr,cy-rr,cx+rr,cy+rr],outline=color,width=width)
d.line([*xy(.83,.54),*xy(1.48,.54)],fill='#677783',width=15)
d.ellipse([cx-6,cy-6,cx+6,cy+6],fill='#173a4c');d.text((cx+10,cy-30),'Actual shoulder axis',font=small,fill='#173a4c')
for row in rows:
    x,y=xy(*row['center_world_m'][:2]);bad=bool(row['reasons']);color='#bd7c35' if bad else '#3f8273';w=.053*900/2;h=.107*900/2
    d.rectangle([x-w,y-h,x+w,y+h],fill=color if row['active'] else '#b8c5ca',outline=color,width=3)
    d.text((x-6,y-10),str(row['jar']),font=small,fill='white' if row['active'] else '#173a4c')
    d.text((x-22,y+h+8),str(round(row['radius_m']*1000)),font=small,fill='#173a4c')
d.rectangle((0,0,1200,76),fill='#10283c');d.text((22,15),'LeHisto loaded-reach audit | current special-stain row',font=font,fill='white');d.text((22,45),'Horizontal radius from the actual shoulder axis; rail parked at zero',font=small,fill='#c7dee5')
notes=['200–330 mm tested samples','210–320 mm design band','','Active jars 4 / 5:','too close for 10 mm margin.','','Spare jars 9–11:','beyond tested 330 mm.','','Current row needs fresh','mapped trajectory checks:','jar height is 24.96 mm lower','relative to the shoulder;','grasp-frame mapping and','moving rail differ.','','Rings show radius only,','not transfer approval.']
for i,line in enumerate(notes):d.text((900,110+i*28),line,font=small,fill='#173a4c')
d.text((34,720),'Nominal simulation evidence only. Collision, rim clearance, payload and force checks remain mandatory.',font=small,fill='#173a4c');im.save(ROOT/'lehisto_loaded_reach_audit.png')
