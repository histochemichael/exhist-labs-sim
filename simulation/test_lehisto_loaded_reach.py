import unittest,copy
import numpy as np
from lehisto_loaded_reach import profile,radial_check,transfer_check,row_position,rig_frame

class LoadedReachTests(unittest.TestCase):
    def test_bounds_are_not_design_margin(self):
        p=profile();self.assertEqual(p['tested_radius_m'],[.2,.33])
        for r in (.2,.33):self.assertIn('inside_tested_bound_but_outside_design_margin',radial_check([0,-r,9],[0,0,-4],p=p)['reasons'])
        for r in (.195,.335):self.assertIn('outside_tested_loaded_radius',radial_check([0,-r,0],[0,0,0])['reasons'])
        self.assertFalse(radial_check([0,-.223,0],[0,0,0])['reasons'])
    def test_transform_and_horizontal_distance(self):
        a=radial_check([2.223,3,10],[2,3,0],[1,0,0]);self.assertAlmostEqual(a['radius_m'],.223);self.assertAlmostEqual(a['yaw_deg'],0)
        self.assertIn('outside_front_horseshoe',radial_check([0,.223,0],[0,0,0])['reasons'])
    def test_radial_check_does_not_admit_unknown_physics(self):
        p=profile();args=dict(robot='LeHisto',center_world=[0,-.223,0],shoulder_world=[0,0,0],front_world=[0,-1,0],conditions=p['conditions'])
        self.assertFalse(transfer_check(**args).allowed)
        verified=dict(trajectory_validated=True,collision_free=True,bottom_clearance_m=.04,grasp_frame_matched=True)
        self.assertTrue(transfer_check(**args,**verified).allowed)
        for key,value in [('payload_kg',.2),('rail_m',.01),('jar_origin_relative_shoulder_z_m',-.117),('rack_tilt_deg',30)]:
            other=copy.deepcopy(args);other['conditions'][key]=value;self.assertFalse(transfer_check(**other,**verified).allowed)
        args['robot']='Nori';self.assertFalse(transfer_check(**args,**verified).allowed)
    def test_profile_controls_placement(self):
        np.testing.assert_allclose(row_position(6),[1.15,.34])
        with self.assertRaises(ValueError):row_position(0)
    def test_actual_rig_frame_tracks_carriage_and_placement(self):
        import mujoco
        from build_scene import ROOT
        m=mujoco.MjModel.from_xml_path(str(ROOT/'exhist_first_pass.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        p0,f0=rig_frame(m,d,'special_lehisto')
        d.qpos[m.joint('special_lehisto_carriage').qposadr[0]]=.05;mujoco.mj_forward(m,d)
        p1,f1=rig_frame(m,d,'special_lehisto');np.testing.assert_allclose(p1-p0,[.05,0,0],atol=1e-8)
        m.body('special_lehisto').pos[:]+=[.4,.2,.1];mujoco.mj_forward(m,d)
        p2,_=rig_frame(m,d,'special_lehisto');np.testing.assert_allclose(p2-p1,[.4,.2,.1],atol=1e-8)

if __name__=='__main__':unittest.main(verbosity=2)
