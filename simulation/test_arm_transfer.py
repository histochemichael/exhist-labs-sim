"""Actual-arm development transfer and negative cases, not lab-wide certification."""
import json,unittest
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import arm_transfer as arm
from pose_control import rotation_error,solve_pose
from cad_jar import add_jar
from build_scene import ROOT,el

class ArmTransferTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases=[arm.run(save=False),arm.run(timestep=.0005,save=False),arm.run(grip_force=0,save=False),arm.run(slide_offset=.02,save=False)]
        (ROOT/'arm_transfer_regression.json').write_text(json.dumps([{k:v for k,v in r.items() if k!='trace'} for r in cls.cases],indent=2))
    def test_actual_arm_transfer(self):
        for r in self.cases[:2]:
            self.assertTrue(r['passed'],r)
            self.assertGreater(r['peak_lift_m'],.03)
            self.assertLess(r['relative_slip_m'],.002)
            self.assertGreater(r['bilateral_contact_fraction'],.99)
            self.assertLess(r['final_position_error_m'],.0015)
            self.assertLessEqual(r['max_arm_torque_Nm'],4.000001)
    def test_timestep_agreement(self):
        self.assertLess(np.linalg.norm(np.array(self.cases[0]['final_slide_position_m'])-self.cases[1]['final_slide_position_m']),.001)
    def test_no_grip_no_transfer(self):
        self.assertFalse(self.cases[2]['passed']);self.assertEqual(self.cases[2]['phase'],'NO_GRASP')
    def test_misalignment_rejected(self):
        self.assertFalse(self.cases[3]['passed']);self.assertEqual(self.cases[3]['phase'],'PAYLOAD_POSE_REJECTED')

class GeometryTests(unittest.TestCase):
    def test_payload_is_passive(self):
        root=arm.build()
        self.assertIsNotNone(root.find(".//body[@name='transfer_slide']/freejoint"))
        self.assertFalse(root.findall('.//weld'));self.assertFalse(root.findall(".//body[@mocap='true']"))
        self.assertFalse(any(a.get('joint')=='transfer_slide_free' for a in root.find('actuator')))
    def test_rotation_half_turn(self):
        self.assertAlmostEqual(np.linalg.norm(rotation_error(np.diag([1,-1,-1]),np.eye(3))),np.pi)
    def test_ik_does_not_mutate_input(self):
        m=mujoco.MjModel.from_xml_string(ET.tostring(arm.build(),encoding='unicode'));d,c=arm.mobile.initialize(m)
        before=d.qpos.copy();solve_pose(m,d.qpos,arm.SITE,arm.JOINTS,arm.PICK,arm.ROTATION,iterations=2)
        np.testing.assert_array_equal(before,d.qpos)
    def test_jar_cavity_is_open(self):
        root=ET.Element('mujoco');el(root,'asset');world=el(root,'worldbody')
        add_jar(root,'jar',[0,0,0],free=False)
        probe=el(world,'body',name='probe',pos='0 0 .04');el(probe,'freejoint');el(probe,'geom',name='probe_geom',type='sphere',size='.003',mass='.001')
        m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        probe_id=m.geom('probe_geom').id
        self.assertFalse(any(probe_id in (c.geom1,c.geom2) for c in d.contact))

if __name__=='__main__':unittest.main()
