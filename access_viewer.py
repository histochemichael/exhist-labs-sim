"""Real-time passive-equipment access fixture; no hardware connections."""
import argparse,time
import mujoco,mujoco.viewer
from equipment_access import Access,KINDS
from render_access_review import lighting,camera

def view(kind):
    a=Access(kind);lighting(a);finished=False
    with mujoco.viewer.launch_passive(a.m,a.d) as v:
        c=camera(a);v.cam.lookat[:]=c.lookat;v.cam.distance=c.distance;v.cam.azimuth=c.azimuth;v.cam.elevation=c.elevation;v.opt.sitegroup[:]=0
        while v.is_running():
            start=time.monotonic()
            # Continue live motor/contact integration at the terminal stance.
            # A frozen final frame cannot reveal drift or a dropped object.
            for _ in range(round(.016/a.m.opt.timestep)):
                if a.step():finished=True
            status='PASS' if a.phase=='COMPLETE' else 'STOP' if finished else 'RUNNING'
            text=f'ExHist equipment access | {kind} | {status}: {a.phase}\nActual arm + passive joint; fixed dock; provisional dynamics\nNo rack insertion, mobile navigation or hardware validation claimed.'
            v.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,text,'')]);v.sync()
            time.sleep(max(0,.016-(time.monotonic()-start)))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--kind',choices=KINDS,default='quincy');view(p.parse_args().kind)
