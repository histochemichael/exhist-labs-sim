import json,math,unittest
import numpy as np
import mujoco
from first_pass import Scene,carrier_rotation,carrier_side,kinematics
from review_correction_qa import mesh_world_bounds
from special_rack_motion import ACTIVE_JARS

class CorrectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.s=Scene()

    def setUp(self):self.s.reset();self.s.guarded=True

    def test_blocks_have_table_support(self):
        s=self.s
        for i in range(1,9):
            lo,hi=mesh_world_bounds(s.m,s.d,s.m.body(f'qc_cassette_{i}').id)
            self.assertAlmostEqual(lo[2],.8,places=7)
            self.assertGreater(lo[0],1.875);self.assertLess(hi[0],3.825)
            self.assertGreater(lo[1],.05);self.assertLess(hi[1],1.05)

    def test_initial_racks_inside_sorting_table(self):
        s=self.s
        for j in s.lab.jobs:
            lo,hi=mesh_world_bounds(s.m,s.d,s.m.body(j.id+'_'+j.carrier).id)
            self.assertAlmostEqual(lo[2],.8,places=6)
            self.assertGreater(lo[0],-7.425);self.assertLess(hi[0],-5.575)
            self.assertGreater(lo[1],.05);self.assertLess(hi[1],1.05)

    def test_perpendicular_jars_and_handle_axes(self):
        s=self.s
        np.testing.assert_allclose(carrier_rotation('rack24')@[1,0,0],[0,1,0],atol=1e-12)
        for i in range(1,12):
            b=s.m.body('special_staining_jar_'+str(i)).id
            r=s.d.xmat[b].reshape(3,3)
            np.testing.assert_allclose(r@[1,0,0],[0,1,0],atol=1e-7)
            color=s.m.geom('special_staining_jar_'+str(i)+'_CAD').rgba
            np.testing.assert_allclose(color,[.55,.68,.73,1] if i in ACTIVE_JARS else [.42,.47,.49,1],atol=1e-7)
        self.assertEqual(ACTIVE_JARS,(4,5,6,7,8))
        self.assertLess(max(e[2] for e in s.special.errors),.001)
        self.assertLess(max(e[3] for e in s.special.errors),.01)

    def test_left_carriers_and_rest_pose(self):
        s=self.s
        for kind in ('scanner_cassette','slide_folder','output_magazine'):self.assertEqual(carrier_side(kind),'left')
        for side in ('left','right'):
            self.assertLess(s.d.xpos[s.m.body(side+'_wrist_roll_link').id,2],s.d.xpos[s.m.body(side+'_shoulder_pitch_link').id,2]-.25)
        self.assertFalse(s.machines.cutaway)

    def test_unvalidated_access_holds_without_door_motion(self):
        s=self.s
        for _ in range(1500):
            s.tick(.1)
            for name in ('quincy_1_door_joint','quincy_2_door_joint','quincy_3_door_joint','imaging_a_s60_pc_door_joint','imaging_b_s60_pc_door_joint'):
                self.assertEqual(s.d.qpos[s.q[name]],0.)
            if s.interaction_hold:break
        self.assertIsNotNone(s.interaction_hold)
        self.assertEqual(s.interaction_hold['target'],'baking')
        before=s.d.qpos.copy();positions=s.d.mocap_pos.copy();t=s.lab.time
        s.lab.paused=False;s.tick(10)
        np.testing.assert_array_equal(before,s.d.qpos);np.testing.assert_array_equal(positions,s.d.mocap_pos);self.assertEqual(s.lab.time,t)
        report=s.save('guarded_review_validation.json');self.assertFalse(report['full_lab_transfers_validated'])
        self.assertEqual(s.lab.completed,0)

if __name__=='__main__':unittest.main(verbosity=2)
