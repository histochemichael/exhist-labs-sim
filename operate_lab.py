"""Default: read-only physics readiness hold. --preview: legacy animation only."""
import argparse, json, math, time
from collections import deque
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
from run_lab import ROOT, camera
from workflow import Lab, CAPACITY
from nori_posture import initialize_bench_view,height_measurements

class Scene:
    def __init__(self,preview=False):
        self.preview=preview;self.initialized=False
        self.layout=json.loads((ROOT/"layout.json").read_text())
        self.m=mujoco.MjModel.from_xml_path(str(ROOT/"exhist_operational.xml"))
        self.d=mujoco.MjData(self.m)
        self.lab=Lab(self.layout["stations"])
        self.lab.paused=not preview
        self.d.qpos[self.m.joint('base_yaw').qposadr[0]]=math.pi/2
        initialize_bench_view(self.m,self.d)
        self.sync()
        self.initialized=True
        if not preview:return
        from robot_motion import Motion
        self.motion=Motion(self.m,self.d,self.lab)
        from machine_motion import Machines
        self.machines=Machines(self.m,self.d)
    def stage(self,station,slot):
        return np.array([self.lab.x[station]+(slot-(CAPACITY[station]-1)/2)*.19,.15,.805])
    def sync(self):
        if self.initialized and not self.preview:
            self.lab.paused=True
            return
        lab=self.lab;tr=lab.transport;rx=lab.robot_x;payload=None
        if tr:
            t=tr["elapsed"];a=tr["approach"];v=tr["travel"]
            sx=lab.x[tr["source"]];tx=lab.x[tr["target"]]
            if t<a:rx=tr["robot_start"]+(sx-tr["robot_start"])*t/max(a,1e-9)
            elif t<a+2:rx=sx
            elif t<a+2+v:rx=sx+(tx-sx)*(t-a-2)/max(v,1e-9)
            else:rx=tx
            payload=np.array([rx,-.75,.78])
            if a<=t<a+2:
                u=(t-a)/2;payload=(1-u)*self.stage(tr["source"],tr["source_slot"])+u*payload
            elif t>=a+2+v:
                u=min(1,(t-a-2-v)/2);payload=(1-u)*payload+u*self.stage(tr["target"],tr["slot"])
            elif t<a:payload=self.stage(tr["source"],tr["source_slot"])
        self.d.qpos[self.m.joint("base_x").qposadr[0]]=rx
        self.d.mocap_pos[:]=[0,0,-10]
        for idx,j in enumerate(lab.jobs):
            pos=self.stage(j.location if j.location!="nori" else "sorting",j.slot)
            if j.state=="WAITING":pos=np.array([lab.x["sorting"]-.65+idx*.19,.02,.805])
            if j.state=="DONE":pos=np.array([lab.x["sendout"]-.7+idx*.46,.2,.805])
            if tr and tr["job"]==j.id:pos=payload
            bid=self.m.body(j.id+"_"+j.carrier).id
            self.d.mocap_pos[self.m.body_mocapid[bid]]=pos
        for station in CAPACITY:
            color=[.95,.15,.1,1] if station in lab.faults else ([1,.65,.1,1] if lab.occupancy[station] else [.1,.75,.5,1])
            self.m.site_rgba[self.m.site(station+"_handoff").id]=color
        if hasattr(self,"motion"):self.motion.update(lab)
        if hasattr(self,"machines"):self.machines.update(lab)
        mujoco.mj_forward(self.m,self.d)
        assert np.isfinite(self.d.mocap_pos).all()
        self.rx=rx
    def status(self,speed):
        if not self.preview:
            height=height_measurements(self.m,self.d)
            return ("ExHist Labs | PHYSICS READINESS HOLD\n"
                    f"Bench {height['bench_top_m']*1000:.0f} mm | shoulders {height['shoulder_height_m']*1000:.0f} mm | lift {height['lift_m']*1000:.0f} mm\n"
                    "No workflow completion or autonomous object motion permitted.\n"
                    "Missing: calibrated grasp frames, contact geometry, arm actuation/inertia.\n"
                    "Missing: wheel dynamics, slot/bucket fit, handle retention and door interactions.\n"
                    "5 passive CAD door joints imported; collision/mass validation pending.\n"
                    "Original handle/slide groove contacts retained; estimated gripper 0.257 kg.\n"
                    "8 proposed drawer pulls follow existing sliders; no automatic drawer drive.\n"
                    "Contact bench is separate; this full-lab model is not physical yet.\n"
                    "See PHYSICS_READINESS.md. --preview explicitly enables old animation.")
        lab=self.lab
        lines=[f"ExHist Labs | ANIMATION ONLY | {'PAUSED' if lab.paused else 'COMPLETE' if lab.completed==36 else 'RUNNING'}",
               f"{lab.completed}/36 slides | {lab.time:.1f}s demo time | {speed}x"]
        for j in lab.jobs:
            step=j.route[j.index].name if j.index<len(j.route) else "packaged"
            lines.append(f"{j.id} {j.protocol}: {step} | {j.state} | {j.location}")
        if lab.faults:lines.append("FAULT: "+", ".join(sorted(lab.faults))+" | C to recover")
        lines.extend(["Special-stain bucket CAD pending; purple targets only.",
                      "CAD-joint robot motion trials / logical processing. Grasps not validated."])
        if hasattr(self,"machines"):lines.append(self.machines.status())
        return "\n".join(lines)
    def save(self):
        if self.preview:
            report=self.lab.report();report["mode"]="KINEMATIC_ANIMATION_ONLY";report["physical_success"]=False
            (ROOT/"workflow_live.json").write_text(json.dumps(report,indent=2))

def validate_render():
    scene=Scene(preview=True);samples=0
    with mujoco.Renderer(scene.m,height=1080,width=1920) as renderer:
        for step in range(4000):
            scene.lab.tick(.1);scene.sync();samples+=1
            if step in [1100,2500] or scene.lab.completed==36:
                renderer.update_scene(scene.d,camera())
                img=Image.fromarray(renderer.render());draw=ImageDraw.Draw(img)
                draw.rectangle([0,0,1920,250],fill="#10283c")
                draw.multiline_text((24,15),scene.status(8),font=ImageFont.truetype("C:/Windows/Fonts/arial.ttf",20),fill="white",spacing=3)
                name="complete" if scene.lab.completed==36 else str(step)
                img.save(ROOT/f"operational_{name}.png")
            if scene.lab.completed==36:break
    assert scene.lab.completed==36
    report={"compiled":True,"mujoco_version":mujoco.__version__,"visual_steps":samples,"completed_slides":36,
            "mocap_carriers":scene.m.nmocap,"finite_transforms":True,
            "scope":"Workflow and kinematic visualization only; no grasp/contact validation"}
    (ROOT/"operational_validation.json").write_text(json.dumps(report,indent=2))
    scene.save();print(json.dumps(report,indent=2))

def view(speed,preview=False):
    import mujoco.viewer
    scene=Scene(preview=preview);keys=deque();follow=False;saved=False
    with mujoco.viewer.launch_passive(scene.m,scene.d,key_callback=keys.append) as v:
        cam=camera()
        v.cam.lookat[:]=cam.lookat;v.cam.distance=cam.distance;v.cam.azimuth=cam.azimuth;v.cam.elevation=cam.elevation
        last=time.monotonic()
        while v.is_running():
            now=time.monotonic();dt=min(.1,now-last)*speed;last=now
            with v.lock():
                while keys:
                    k=keys.popleft()
                    if not preview and k not in [79,*range(49,57)]:continue
                    if k==32:
                        scene.machines.inspect=False
                        scene.lab.paused=not scene.lab.paused
                    elif k==82:
                        scene.lab=Lab(scene.layout["stations"]);saved=False
                        scene.motion.prev_time=0;scene.motion.elapsed={n:0 for n in scene.motion.elapsed}
                        scene.machines.last_time=0;scene.machines.elapsed={n:0 for n in scene.machines.elapsed}
                        scene.machines.inspect=False
                        for row,dof in scene.machines.dofs:
                            scene.d.qpos[scene.m.joint(dof["name"]).qposadr[0]]=0
                    elif k==70:scene.lab.inject_scan_fault=True
                    elif k==67:scene.lab.recover()
                    elif k==84:follow=not follow
                    elif k==88:scene.machines.toggle_xray()
                    elif k==74:
                        scene.machines.inspect=not scene.machines.inspect
                        scene.lab.paused=True
                    elif k==91:scene.machines.select(-1)
                    elif k==93:scene.machines.select(1)
                    elif k==71:scene.machines.jog()
                    elif k==72:scene.machines.jog(reset=True)
                    elif k in [61,334]:speed=min(32,speed*2)
                    elif k in [45,333]:speed=max(.5,speed/2)
                    elif 49<=k<=56:
                        cam=camera(scene.layout["stations"][k-49]["x"],True);v.cam.lookat[:]=cam.lookat;v.cam.distance=cam.distance;v.cam.azimuth=cam.azimuth;v.cam.elevation=cam.elevation;follow=False
                    elif k==79:
                        cam=camera();v.cam.lookat[:]=cam.lookat;v.cam.distance=cam.distance;v.cam.azimuth=cam.azimuth;v.cam.elevation=cam.elevation;follow=False
                while preview and dt>0 and scene.lab.completed<36:
                    part=min(dt,.05);scene.lab.tick(part);dt-=part
                scene.sync()
                if follow:v.cam.lookat[0]=scene.rx;v.cam.distance=3.5
            v.set_texts([(100,mujoco.mjtGridPos.mjGRID_TOPLEFT,scene.status(speed),""),
                         (100,mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,"1-8 station | O overview | mouse orbit/zoom" if not preview else "ANIMATION ONLY: SPACE pause | R reset | F fail scan | C recover\n1-8 station | O overview | T follow Nori | +/- speed\nX X-ray | J joint inspection | [ ] select | G jog | H home","")])
            v.sync()
            if scene.lab.completed==36 and not saved:scene.save();saved=True
            time.sleep(1/30)
    scene.save()

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--check-render",action="store_true");p.add_argument("--speed",type=float,default=2)
    p.add_argument("--preview",action="store_true",help="Explicitly enable nonphysical workflow animation; not a successful grasp simulation")
    args=p.parse_args()
    if args.check_render:validate_render()
    else:view(args.speed,args.preview)
