import unittest
import numpy as np,mujoco
from nori_posture import initialize_bench_view,height_measurements,BENCH_VIEW_LIFT_M
from operate_lab import Scene

class PostureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.s=Scene()

    def test_table_stays_at_800mm_and_shoulders_clear_it(self):
        s=self.s;r=height_measurements(s.m,s.d)
        self.assertAlmostEqual(r['bench_top_m'],.8)
        self.assertAlmostEqual(r['shoulder_height_m'],1.0659107)
        self.assertGreater(r['shoulder_above_bench_m'],.25)
        for st in s.layout['stations']:
            self.assertAlmostEqual(height_measurements(s.m,s.d,st['name']+'_top')['bench_top_m'],.8)

    def test_mimic_limits_and_relaxed_arms(self):
        s=self.s;m=s.m;d=s.d
        self.assertAlmostEqual(d.qpos[m.joint('lift_extension_joint').qposadr[0]],BENCH_VIEW_LIFT_M)
        self.assertAlmostEqual(d.qpos[m.joint('lift_middle_joint').qposadr[0]],BENCH_VIEW_LIFT_M/2)
        for name in ('lift_extension_joint','lift_middle_joint'):
            j=m.joint(name);self.assertTrue(j.range[0]<=d.qpos[j.qposadr[0]]<=j.range[1])
        for side in ('left','right'):
            self.assertLess(d.body(side+'_wrist_roll_link').xpos[2],d.body(side+'_shoulder_pitch_link').xpos[2]-.25)

    def test_no_idle_pose_drift_or_reinitialization(self):
        s=self.s;before=s.d.qpos.copy()
        for _ in range(100):s.lab.tick(.1);s.sync()
        np.testing.assert_array_equal(s.d.qpos,before);self.assertEqual(s.lab.time,0.)
        d=mujoco.MjData(s.m);d.time=.1;before=d.qpos.copy()
        with self.assertRaises(ValueError):initialize_bench_view(s.m,d)
        np.testing.assert_array_equal(d.qpos,before)

    def test_cad_inspector_uses_same_height(self):
        from run_lab import init
        m,d,_=init();self.assertAlmostEqual(height_measurements(m,d)['shoulder_height_m'],1.0659107)

if __name__=='__main__':unittest.main(verbosity=2)
