"""Development physics regressions; passing negative tests is NOT transfer success."""
import unittest,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import mobile_base_bench as mobile
import drawer_contact_bench as drawer

class MobileTests(unittest.TestCase):
    def test_drive_turn_and_stop(self):
        r=mobile.run(save=False)
        self.assertTrue(r['passed'],r)
        self.assertEqual(len(r['events']),2)
        self.assertLess(abs(r['yaw_rad']-math.pi/2),.03)
        self.assertLess(np.linalg.norm(np.array(r['pose'][:2])-[.45,.35]),.015)
        self.assertLess(r['max_lateral_velocity_mps'],.01)
        self.assertLess(r['final_linear_speed_mps'],.005)
        for side in ('left','right'):
            self.assertLessEqual(r['actuator_peak_effort'][side+'_wheel_motor']['value'],1.500001)
        for name,effort in r['actuator_peak_effort'].items():
            if name.startswith('hold_'):self.assertLessEqual(effort['value'],4.000001)

    def test_no_motors_no_navigation(self):
        r=mobile.run(motors_enabled=False,duration_s=5,save=False)
        self.assertFalse(r['passed']);self.assertFalse(r['events'])
        self.assertLess(np.linalg.norm(r['pose'][:2]),.003)
        for side in ('left','right'):self.assertEqual(r['actuator_peak_effort'][side+'_wheel_motor']['value'],0)

    def test_tilt_fault_latches(self):
        m=mujoco.MjModel.from_xml_string(ET.tostring(mobile.build(),encoding='unicode'));d,c=mobile.initialize(m)
        # Test-injected bad orientation, not a production motion command.
        d.qpos[3:7]=[math.cos(.2),math.sin(.2),0,0];mujoco.mj_forward(m,d);c.update()
        self.assertEqual(c.phase,'TILT_FAULT');self.assertTrue(np.all(d.ctrl[c.wheels]==0))
        d.qpos[3:7]=[1,0,0,0];d.time=3;mujoco.mj_forward(m,d);c.update()
        self.assertEqual(c.phase,'TILT_FAULT');self.assertTrue(np.all(d.ctrl[c.wheels]==0))

class DrawerTests(unittest.TestCase):
    def test_drawer_has_no_actuator(self):
        root=drawer.build()
        self.assertTrue(all(a.get('joint')!='CAD_drawer_slide' for a in root.find('actuator')))
        self.assertFalse(root.findall('.//weld'));self.assertFalse(root.findall(".//body[@mocap='true']"))

    def test_slip_not_success(self):
        r=drawer.run();self.assertFalse(r['passed']);self.assertEqual(r['phase'],'GRASP_LOST')
        self.assertGreater(r['peak_relative_slip_m'],.004)
        self.assertLessEqual(r['max_fixture_force_N'],r['force_limit_N']+1e-9)
        self.assertGreaterEqual(r['post_stop_simulation_s'],.5)
        self.assertLess(r['post_stop_drawer_drift_m'],.004)

    def test_blocked_drawer_force_stop(self):
        r=drawer.run(friction=30,force_limit=1)
        self.assertFalse(r['passed']);self.assertEqual(r['phase'],'FORCE_STOP')
        self.assertLess(r['max_open_m'],.001);self.assertLessEqual(r['max_fixture_force_N'],1.000001)
        self.assertGreaterEqual(r['post_stop_simulation_s'],.5)
        self.assertLess(r['post_stop_drawer_drift_m'],.001)

if __name__=='__main__':unittest.main()
