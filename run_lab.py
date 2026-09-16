"""MuJoCo CAD inspection and scripted station tour (kinematic, not autonomous)."""
from pathlib import Path
import argparse,json,time,math
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent
def init():
    model=mujoco.MjModel.from_xml_path(str(ROOT/"exhist.xml"))
    data=mujoco.MjData(model)
    layout=json.loads((ROOT/"layout.json").read_text())
    data.qpos[model.joint("base_x").qposadr[0]]=layout["stations"][0]["x"]
    data.qpos[model.joint("base_yaw").qposadr[0]]=math.pi/2
    from nori_posture import initialize_bench_view
    initialize_bench_view(model,data)
    mujoco.mj_forward(model,data)
    return model,data,layout
def camera(x=0,close=False):
    cam=mujoco.MjvCamera()
    cam.lookat[:]=[x,0.25,.95] if close else [0,-1.8,1.15]
    cam.azimuth=90 if close else 100;cam.elevation=-25 if close else -38
    cam.distance=3.0 if close else 16.6
    return cam
def check(model,data,layout):
    assert model.neq==4,"Nori parallel-gripper constraints were not preserved"
    assert len(layout["stations"])==8
    assert np.isfinite(data.qpos).all()
    for st in layout["stations"]:
        assert model.site(st["name"]+"_dock").id>=0
        assert model.site(st["name"]+"_handoff").id>=0
    # Geometry-only whole-base transit check against bench geoms.
    # This is NOT a reach or grasp test. Ignores existing robot self-contacts.
    collisions=[]
    for x in np.linspace(layout["stations"][0]["x"],layout["stations"][-1]["x"],80):
        data.qpos[model.joint("base_x").qposadr[0]]=x
        mujoco.mj_forward(model,data)
        for contact in data.contact:
            a=model.geom(contact.geom1).name;b=model.geom(contact.geom2).name
            if contact.dist < -0.001 and any(t in a+b for t in ["_top","_leg_","_envelope"]):
                collisions.append([float(x),a,b,float(contact.dist)])
    report=dict(compiled=True,engine=mujoco.__version__,bodies=model.nbody,joints=model.njnt,
        meshes=model.nmesh,geoms=model.ngeom,nori_mimic_constraints=model.neq,tables=8,
        cad_instances=len(layout["placements"]),aisle_transit_samples=80,
        aisle_environment_penetrations=collisions,
        scope="Kinematic straight-aisle geometry check only; no docking, reach, grasp or dynamics validation.")
    (ROOT/"validation.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    assert not collisions,"Straight-aisle transit intersects environment"
def render(model,data,layout):
    data.qpos[model.joint("base_x").qposadr[0]]=layout["stations"][0]["x"]
    mujoco.mj_forward(model,data)
    with mujoco.Renderer(model,height=1080,width=1920) as renderer:
        for name,cam in [("overview",camera()),("sorting",camera(layout["stations"][0]["x"],True)),
                          ("sendout",camera(layout["stations"][5]["x"],True)),
                          ("imaging",camera(layout["stations"][6]["x"],True))]:
            renderer.update_scene(data,cam)
            img=Image.fromarray(renderer.render())
            draw=ImageDraw.Draw(img)
            draw.rectangle([0,0,1920,76],fill="#10283c")
            draw.text((28,12),"ExHist Labs | CAD assembly v01",font=ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf",28),fill="white")
            draw.text((28,46),"MuJoCo layout preview | Equipment static | Table heights provisional | Reach not yet validated",
                      font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",18),fill="#bcd7e8")
            img.save(ROOT/f"{name}.png")
    print("Rendered overview and three detail views.")
def view(model,data,layout,tour=False):
    import mujoco.viewer
    state={"tour":tour,"station":0,"focus":False}
    def key(k):
        if k==32:state["tour"]=not state["tour"]
        elif 49<=k<=56:
            state["station"]=k-49;state["tour"]=False;state["focus"]=True
        elif k in [79,111]:state["focus"]=False
    print("SPACE: toggle scripted tour | 1-8: station close-up | O: overview | mouse: orbit/zoom")
    with mujoco.viewer.launch_passive(model,data,key_callback=key) as v:
        v.cam.lookat[:]=camera().lookat;v.cam.distance=12.3;v.cam.azimuth=90;v.cam.elevation=-25
        start=time.monotonic();last_focus=None
        while v.is_running():
            if state["tour"]:
                t=(time.monotonic()-start)*0.20
                n=len(layout["stations"]);phase=t%(2*(n-1))
                p=phase if phase<n-1 else 2*(n-1)-phase
                i=min(int(p),n-2);u=p-i
                x=(1-u)*layout["stations"][i]["x"]+u*layout["stations"][i+1]["x"]
                data.qpos[model.joint("base_x").qposadr[0]]=x
            elif state["focus"]:
                data.qpos[model.joint("base_x").qposadr[0]]=layout["stations"][state["station"]]["x"]
            focus=state["station"] if state["focus"] else -1
            if focus!=last_focus:
                cam=camera(layout["stations"][focus]["x"],True) if focus>=0 else camera()
                v.cam.lookat[:]=cam.lookat;v.cam.distance=cam.distance
                last_focus=focus
            mujoco.mj_forward(model,data)
            v.sync()
            time.sleep(1/30)
if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--check",action="store_true");ap.add_argument("--render",action="store_true")
    ap.add_argument("--tour",action="store_true")
    args=ap.parse_args();m,d,l=init()
    if args.check:check(m,d,l)
    if args.render:render(m,d,l)
    if not args.check and not args.render:view(m,d,l,args.tour)
