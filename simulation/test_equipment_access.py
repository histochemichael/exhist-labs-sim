"""Equipment-access safety regressions, not assertions of complete lab readiness."""
import unittest,math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import equipment_access as access
from build_scene import ROOT

class AccessStructureTests(unittest.TestCase):
    def test_six_access_types_are_passive(self):
        for kind in access.KINDS:
            root,meta=access.build(kind)
            self.assertFalse(root.findall('.//weld'))
            self.assertFalse(root.findall(".//body[@mocap='true']"))
            self.assertTrue(all(a.get('joint')!=meta['joint'] for a in root.find('actuator')))
            self.assertTrue(all(j.get('joint1')!=meta['joint'] and j.get('joint2')!=meta['joint'] for j in root.find('equality')))
            model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
            self.assertEqual(model.joint(meta['joint']).type[0],mujoco.mjtJoint.mjJNT_HINGE if kind=='quincy' else mujoco.mjtJoint.mjJNT_SLIDE)
            self.assertIsNone(root.find("worldbody/body[@name='mobile_chassis']/freejoint")) # explicit isolated dock
            for n in access.ARM:np.testing.assert_allclose(model.actuator_forcerange[model.actuator('hold_'+n).id],[-4,4])

    def test_canopy_and_wider_pulls(self):
        root=ET.parse(ROOT/'exhist_first_pass.xml').getroot()
        b=root.find("worldbody/body[@name='special_canopy']");self.assertIsNotNone(b)
        self.assertAlmostEqual(float(b.get('pos').split()[2]),1.70)
        self.assertIsNone(b.find("geom[@name='special_canopy_floor']"))
        pulls=[g for g in root.findall('.//geom') if g.get('name','').endswith('_pull_bar')]
        self.assertEqual(len(pulls),8)
        for bar in pulls:self.assertAlmostEqual(float(bar.get('size').split()[0]),.041)

class AccessDynamicsTests(unittest.TestCase):
    def run_case(self,**kwargs):
        a=access.Access(**kwargs)
        for _ in range(round(16/a.m.opt.timestep)):
            if a.step():break
        return a

    def test_no_grip_no_open(self):
        a=self.run_case(kind='quincy',miss=True)
        self.assertFalse(a.result()['passed'])
        self.assertNotIn('OPEN',[x['phase'] for x in a.events])
        self.assertIsNotNone(a.terminal)
        self.assertGreaterEqual(a.d.time-a.terminal,.5)

    def test_jam_stops_without_success(self):
        a=self.run_case(kind='st_load',jammed=True)
        self.assertFalse(a.result()['passed'])
        self.assertIn(a.phase,('JAM_STOP','FORCE_STOP','GRASP_LOST','CONTACT_LOST'))
        self.assertLess(a.max_open,.005)
        self.assertLessEqual(a.max_torque,4.000001)
        self.assertLessEqual(a.max_lift_force,150.000001)
        self.assertIsNotNone(a.terminal)
        self.assertGreaterEqual(a.d.time-a.terminal,.5)

if __name__=='__main__':unittest.main(verbosity=2)
