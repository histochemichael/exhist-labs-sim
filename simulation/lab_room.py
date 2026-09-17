"""Furnished architectural context for ExHist; provisional room dimensions."""
import json,math
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image,ImageDraw,ImageFont
from build_scene import el,box,vec
ROOT=Path(__file__).resolve().parent

def add_room(root):
    world=root.find("worldbody");asset=root.find("asset")
    prior=world.find("body[@name='lab_room']")
    if prior is not None:world.remove(prior)
    room=el(world,"body",name="lab_room")
    floor=world.find("geom[@name='checkerboard_floor']")
    floor.set("pos","0 0 -.05")
    white=[.88,.91,.92,1];teal=[.12,.34,.39,1];metal=[.45,.51,.55,1]
    count={};obstacles=[]
    def b(name,pos,size,color=white,**kw):return box(room,"lab_"+name,pos,size,color,**kw)
    def record(kind,pos,size):
        count[kind]=count.get(kind,0)+1
        obstacles.append(dict(kind=kind,center=pos,halfsize=size))
    def sign(name,text,x,y,z,width=1.1,height=.15):
        path=ROOT/"assets"/("lab_"+name+".png")
        img=Image.new("RGB",(1024,128),(23,64,75));draw=ImageDraw.Draw(img)
        draw.text((512,64),text,anchor="mm",font=ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf",43),fill="white");img.save(path)
        tn="lab_"+name+"_tex";mn="lab_"+name+"_mat"
        if asset.find(f"texture[@name='{tn}']") is None:
            el(asset,"texture",name=tn,type="2d",file="assets/"+path.name)
            el(asset,"material",name=mn,texture=tn,texuniform="false",emission=".3")
        el(room,"geom",name="lab_"+name+"_sign",type="plane",pos=vec([x,y,z]),size=vec([width/2,height/2,.001]),euler=vec([math.pi/2,0,0]),material=mn,contype="0",conaffinity="0")
    # Three-sided room keeps the front open for inspection and cameras.
    b("floor",[0,-2.25,-.025],[9.2,4.45,.025],[.73,.79,.80,1])
    for x in range(-9,10):
        b(f"floor_line_x{x}",[x,-2.25,.0005],[.001,4.4,.0005],[.57,.65,.67,1],contype="0",conaffinity="0")
    for i in range(9):
        b(f"floor_line_y{i}",[0,-6.5+i,.0006],[9.1,.001,.0005],[.57,.65,.67,1],contype="0",conaffinity="0")
    # Back wall with real window and doorway openings.
    b("back_lower",[.95,2.08,.68],[8.15,.07,.68])
    b("back_upper",[0,2.08,2.97],[9.1,.07,.23])
    spans=[(-9.1,-8.65),(-7.35,-6.9),(-4.7,-2.9),(-.7,1.1),(3.3,5.1),(7.3,9.1)]
    for i,(a,c) in enumerate(spans):
        b(f"back_pier_{i}",[(a+c)/2,2.08,2.05],[(c-a)/2,.07,.69])
    b("entry_lintel",[-8,2.08,2.52],[.65,.07,.22])
    b("entry_left_jamb",[-8.875,2.08,.68],[.225,.07,.68])
    b("entry_right_jamb",[-7.275,2.08,.68],[.075,.07,.68])
    b("west_wall",[-9.08,-2.25,1.6],[.08,4.4,1.6])
    b("east_wall",[9.08,-2.25,1.6],[.08,4.4,1.6])
    # Observation windows with frames and muntins.
    for i,x in enumerate([-5.8,-1.8,2.2,6.2]):
        b(f"window_{i}",[x,2.075,2.05],[1.09,.015,.68],[.40,.67,.76,.38],contype="0",conaffinity="0")
        for sx in [-1,0,1]:b(f"window_{i}_v{sx}",[x+sx*1.1,1.995,2.05],[.018,.035,.70],metal)
        for sz in [-1,1]:b(f"window_{i}_h{sz}",[x,1.995,2.05+sz*.69],[1.12,.035,.018],metal)
        b(f"window_{i}_sill",[x,1.91,1.35],[1.14,.14,.025],white)
        count["windows"]=count.get("windows",0)+1
    # Entry door and a side service door (closed scene props).
    b("entry_door",[-8,2.075,1.13],[.63,.028,1.11],[.18,.40,.45,1])
    b("entry_vision",[-8,2.038,1.58],[.28,.008,.37],[.48,.73,.80,1],contype="0",conaffinity="0")
    b("entry_pushbar",[-8,2.005,1.0],[.43,.022,.015],metal)
    sign("entry","STAFF ENTRY",-8,1.985,2.52,1.1,.17)
    b("service_door",[8.985,-4.75,1.13],[.022,.62,1.11],teal)
    b("service_vision",[8.958,-4.75,1.6],[.007,.25,.35],[.48,.73,.8,1],contype="0",conaffinity="0")
    b("service_handle",[8.925,-4.3,1.04],[.025,.10,.013],metal)
    count["doors"]=2
    # Wall-storage towers alternate with the windows, behind the equipment row.
    def cabinet(name,x,y,width=.75,height=2.25,depth=.42):
        b(name+"_case",[x,y,height/2],[width/2,depth/2,height/2],[.73,.8,.82,1])
        b(name+"_toe",[x,y,.05],[width/2-.03,depth/2+.01,.05],teal)
        for side in [-1,1]:
            b(name+f"_door{side}",[x+side*width/4,y-depth/2-.01,height/2+.05],[width/4-.007,.015,height/2-.08],white)
            b(name+f"_pull{side}",[x+side*.05,y-depth/2-.037,height*.54],[.007,.012,.09],metal)
        record("cabinets",[x,y,height/2],[width/2,depth/2+.05,height/2])
    for i,x in enumerate([-3.8,.2,4.2,8.1]):cabinet(f"storage_tower_{i}",x,1.72)
    # Segmented shallow shelf bays, stocked with boxes and binders.
    for bay,x in enumerate([-5.8,-1.8,2.2,6.2]):
        for side in [-1,1]:
            b(f"shelf_rail_{bay}_{side}",[x+side*.85,1.91,.66],[.018,.018,.65],metal)
        for z in [.35,.70,1.05]:
            b(f"shelf_{bay}_{z}",[x,1.73,z],[.88,.20,.015],metal)
            for k in range(5):
                color=[[.80,.75,.60,1],[.36,.55,.59,1],[.87,.88,.82,1]][k%3]
                b(f"shelf_box_{bay}_{z}_{k}",[x-.66+k*.31,1.73,z+.095],[.13,.15,.08],color,contype="0",conaffinity="0")
        record("shelving_bays",[x,1.73,.72],[.9,.22,.42])
    # Prep islands and documentation desks on the opposite side of the travel aisle.
    def table(name,x,y,width=2.1,depth=.85):
        b(name+"_top",[x,y,.805],[width/2,depth/2,.025],[.86,.89,.88,1])
        for a in [-1,1]:
            for c in [-1,1]:b(name+f"_leg{a}_{c}",[x+a*(width/2-.08),y+c*(depth/2-.08),.39],[.025,.025,.39],metal)
        record("extra_tables",[x,y,.415],[width/2,depth/2,.415])
    def pc(name,x,y):
        b(name+"_monitor",[x,y,1.15],[.25,.028,.16],[.12,.16,.18,1])
        b(name+"_screen",[x,y-.032,1.15],[.23,.003,.14],[.06,.24,.32,1],contype="0",conaffinity="0")
        for k in range(4):b(name+f"_ui{k}",[x-.02,y-.036,1.23-k*.045],[.18,.001,.007],[.27,.65,.70,1],contype="0",conaffinity="0")
        b(name+"_stem",[x,y,.94],[.018,.025,.08],metal)
        b(name+"_foot",[x,y,.841],[.13,.085,.01],metal)
        b(name+"_keyboard",[x,y-.22,.84],[.18,.065,.007],[.18,.23,.25,1])
        b(name+"_mouse",[x+.24,y-.22,.845],[.027,.04,.014],[.18,.23,.25,1])
        b(name+"_tower",[x+.45,y,.99],[.075,.14,.16],[.16,.20,.23,1])
        count["additional_pcs"]=count.get("additional_pcs",0)+1
    def chair(name,x,y):
        b(name+"_seat",[x,y,.49],[.22,.21,.035],teal)
        b(name+"_back",[x,y-.20,.76],[.22,.03,.23],teal)
        el(room,"geom",name="lab_"+name+"_post",type="cylinder",pos=vec([x,y,.26]),size=".035 .20",rgba=vec(metal))
        for i in range(5):
            angle=i*2*math.pi/5;dx=.28*math.cos(angle);dy=.28*math.sin(angle)
            el(room,"geom",name="lab_"+name+f"_spoke{i}",type="capsule",fromto=vec([x,y,.085,x+dx,y+dy,.085]),size=".018",rgba=vec(metal))
            el(room,"geom",name="lab_"+name+f"_wheel{i}",type="sphere",pos=vec([x+dx,y+dy,.04]),size=".04",rgba=".12 .15 .17 1")
        record("chairs",[x,y,.50],[.34,.34,.50])
    for i,(x,y) in enumerate([(-5.75,-4.05),(-2.8,-4.05),(.25,-4.05),(4.9,-4.25)]):
        table(f"workbay_{i}",x,y,2.35 if i==3 else 2.1)
        chair(f"task_chair_{i}",x,y-.88)
        sign(f"bay_{i}",["PREPARATION","GENERAL WORK","DOCUMENTATION","DIGITAL REVIEW"][i],x,y-.445,.68,1.6,.14)
        if i>=2:pc(f"desk_pc_{i}",x-.2,y+.14)
        else:
            for k in range(3):b(f"workbay_{i}_tray{k}",[x-.62+k*.42,y,.86],[.16,.23,.03],[.64,.75,.77,1])
        # Under-bench drawer cabinet, leaving seated knee space.
        b(f"underbench_{i}",[x+.73,y,.40],[.22,.34,.37],white)
        for k in range(3):
            b(f"drawer_{i}_{k}",[x+.73,y-.35,.23+k*.20],[.205,.015,.087],[.74,.83,.85,1])
            b(f"drawerpull_{i}_{k}",[x+.73,y-.38,.27+k*.20],[.10,.013,.008],metal)
    # Additional freestanding shelving at far right, away from Nori's lane.
    for i,z in enumerate([.15,.65,1.15,1.65]):
        b(f"east_shelf_{i}",[8.3,-2.95,z],[.48,.65,.022],metal)
        for j in range(3):b(f"east_storage_{i}_{j}",[8.3,-3.4+j*.4,z+.13],[.30,.16,.105],[.78,.78,.65,1],contype="0",conaffinity="0")
    for x in [7.84,8.76]:
        for y in [-3.58,-2.32]:b(f"east_post_{x}_{y}",[x,y,.91],[.022,.022,.91],metal)
    record("shelving_bays",[8.3,-2.95,.95],[.50,.67,.95])
    # Visual zoning, task lighting, and a clear striped travel boundary.
    for i,(x,text,width) in enumerate([(-5.45,"SORT / BAKE",3.4),(-1.4,"STAINING",4.0),(2.8,"QC / SEND-OUT",3.0),(6.3,"IMAGING",3.0)]):
        sign(f"zone_{i}",text,x,1.985,2.91,width,.20)
    for y in [-1.85,-.06]:
        b(f"aisle_edge_{y}",[0,y,.002],[7.85,.018,.001],[.97,.77,.27,1],contype="0",conaffinity="0")
    for row,y in enumerate([.25,-3.8]):
        for i,x in enumerate([-6,-2,2,6]):
            b(f"light_frame_{row}_{i}",[x,y,3.11],[.70,.20,.035],metal,contype="0",conaffinity="0")
            b(f"light_panel_{row}_{i}",[x,y,3.07],[.66,.17,.006],[.96,.99,1,1],contype="0",conaffinity="0")
            for side in [-1,1]:
                b(f"light_hanger_{row}_{i}_{side}",[x+side*.5,y,3.18],[.003,.003,.04],metal,contype="0",conaffinity="0")
    report=dict(room_size_m=[18.4,8.9,3.2],style="Three-sided cutaway clinical laboratory",
        counts=count,obstacles=obstacles,travel_lane_clear=True,
        limitations=["Provisional architectural layout, not a code-compliant building or ventilation design.",
                     "New furniture is static; no door swing or chair movement simulation."])
    for obs in obstacles:
        y=obs["center"][1];h=obs["halfsize"][1]
        assert y+h < -1.85 or y-h>.05,("Furniture intrudes on robot aisle",obs)
    (ROOT/"room_layout.json").write_text(json.dumps(report,indent=2))
    return report

if __name__=="__main__":
    for name in ["exhist.xml","exhist_operational.xml"]:
        root=ET.parse(ROOT/name).getroot();report=add_room(root)
        ET.indent(root);ET.ElementTree(root).write(ROOT/name,encoding="utf-8",xml_declaration=True)
    print(json.dumps(report["counts"],indent=2))
