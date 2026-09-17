import unittest,math,json
from functools import lru_cache
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from front_docking import facing_yaw,docking_ready,write_station_docks,PARAMETERS
import arm_transfer as arm
import equipment_dock_bench as dock
from build_scene import ROOT

@lru_cache(maxsize=1)
def reference_transfer():return arm.run(front=True,render=True)

class FacingTests(unittest.TestCase):
    def test_station_headings(self):
        rows=write_station_docks();self.assertEqual(len(rows),8)
        for row in rows:self.assertAlmostEqual(row['required_yaw_rad'],math.pi/2)
    def test_sideways_and_moving_rejected(self):
        self.assertFalse(docking_ready([0,0],math.pi/2,[1,0],0,0))
        self.assertFalse(docking_ready([0,0],0,[1,0],.05,0))
        self.assertFalse(docking_ready([0,0],0,[1,0],0,.1))
        self.assertTrue(docking_ready([0,0],0,[1,0],0,0))
    def test_nonfinite_rejected(self):
        self.assertFalse(docking_ready([0,0],float('nan'),[1,0],0,0))
        with self.assertRaises(ValueError):facing_yaw([0,0],[0,0])
    def test_head_cameras_follow_lift_and_base(self):
        root=arm.build(front=True)
        self.assertIsNone(root.find("equality/joint[@name='TEST_FIXTURE_lift_brake']"))
        m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        for name in ['nori_head_left','nori_head_right','nori_head_wide']:
            self.assertEqual(m.cam_bodyid[m.camera(name).id],m.body('lift_top_link').id)
            self.assertEqual(m.cam_mode[m.camera(name).id],mujoco.mjtCamLight.mjCAMLIGHT_FIXED)
        self.assertEqual(m.actuator_forcerange[m.actuator('provisional_lift_motor').id,1],150)
        d,controller=arm.mobile.initialize(m);cid=m.camera('nori_head_left').id;start=d.cam_xpos[cid].copy()
        d.qpos[m.joint('lift_extension_joint').qposadr[0]]+=.1
        d.qpos[m.joint('lift_middle_joint').qposadr[0]]+=.05
        mujoco.mj_forward(m,d);np.testing.assert_allclose(d.cam_xpos[cid]-start,[0,0,.1],atol=1e-7)
        d.qpos[3:7]=[math.sqrt(.5),0,0,math.sqrt(.5)];mujoco.mj_forward(m,d)
        forward=-d.cam_xmat[cid].reshape(3,3)[:,2]
        self.assertGreater(forward[1],.9);self.assertLess(abs(forward[0]),1e-6)
    def test_wheel_driven_facing_dock(self):
        r=dock.run();self.assertTrue(r['passed'],r);self.assertLess(abs(r['heading_error_deg']),3);self.assertFalse(r['lift_brake'])
    def test_front_transfer_and_cameras(self):
        r=reference_transfer()
        self.assertTrue(r['passed'],{k:v for k,v in r.items() if k!='trace'});self.assertTrue(r['camera_check_passed'],r['head_camera_samples'])
        self.assertLess(r['max_heading_error_deg'],3);self.assertLessEqual(r['maximum_provisional_lift_force_N'],150)
        self.assertLess(r['final_position_error_m'],.0015)
        self.assertGreaterEqual(len(r['head_camera_samples']),5)
    def test_front_timestep_convergence(self):
        reference=reference_transfer()
        r=arm.run(front=True,timestep=.0005,save=False)
        (ROOT/'front_transfer_halfstep_validation.json').write_text(json.dumps({k:v for k,v in r.items() if k!='trace'},indent=2))
        self.assertTrue(r['passed'],{k:v for k,v in r.items() if k!='trace'});self.assertTrue(r['camera_check_passed'])
        self.assertLess(np.linalg.norm(np.array(r['final_slide_position_m'])-reference['final_slide_position_m']),.001)
        (ROOT/'front_transfer_halfstep_validation.json').write_text(json.dumps({k:v for k,v in r.items() if k!='trace'},indent=2))

if __name__=='__main__':unittest.main()
