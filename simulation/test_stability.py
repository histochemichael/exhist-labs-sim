"""Pose-latch and support regressions, separate from complete lab readiness."""
import json,unittest,xml.etree.ElementTree as ET
import numpy as np,mujoco
from equipment_access import Access,MOTION_PHASES
from build_scene import ROOT
from passive_stability import build
from special_layout import SNAPSHOT

class StanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.a=Access('st_load')

    def test_stationary_phases_do_not_replan(self):
        self.assertEqual(set(MOTION_PHASES),{'APPROACH','OPEN','CLOSE','RETREAT'})
        a=self.a
        for phase in ('SETTLE','GRIP','HOLD','RELEASE'):
            if phase=='HOLD':a.reference=a.d.site_xmat[a.sid].reshape(3,3).T@(a.d.site_xpos[a.hsid]-a.d.site_xpos[a.sid])
            a.change(phase,duration=10);q=a.qtarget.copy();lift=a.lift_command
            for _ in range(100):a.step()
            np.testing.assert_array_equal(a.qtarget,q)
            self.assertEqual(a.lift_command,lift)

    def test_stop_preserves_loaded_command_not_measured_pose(self):
        a=self.a;a.qtarget=a.d.qpos[a.qa]+.002
        q=a.qtarget.copy();lift=a.lift_command
        a.change('FORCE_STOP')
        np.testing.assert_array_equal(a.qtarget,q);self.assertEqual(a.lift_command,lift)
        for _ in range(100):a.step()
        np.testing.assert_array_equal(a.qtarget,q)
        self.assertGreater(a.d.time,a.terminal) # still live, not a paused frame

class PassiveTests(unittest.TestCase):
    def test_no_support_means_fall(self):
        root=build();world=root.find('worldbody');world.remove(world.find("body[@name='special_table']"))
        m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        z=float(d.body('test_rack').xpos[2])
        for _ in range(200):mujoco.mj_step(m,d)
        self.assertGreater(z-d.body('test_rack').xpos[2],.15)
        self.assertEqual(m.nu,0);self.assertEqual(m.nmocap,0)

    def test_exact_active_layout_transform_and_support(self):
        m=mujoco.MjModel.from_xml_path(str(ROOT/'exhist_operational.xml'));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        layout=json.loads(SNAPSHOT.read_text());rotation=np.asarray(layout['world_rotation']);rq=np.zeros(4);mujoco.mju_mat2Quat(rq,rotation.ravel())
        for s in layout['stations']:
            b=d.body('special_staining_jar_'+str(s['id']));q=np.zeros(4);mujoco.mju_mulQuat(q,rq,np.array(s['quat_wxyz']))
            np.testing.assert_allclose(b.xpos,s['world_position_m'],atol=1e-8)
            self.assertGreater(abs(float(b.xquat@q)),1-1e-10)
            riser=d.body('special_staining_jar_'+str(s['id'])+'_riser');self.assertAlmostEqual(riser.xpos[2],.804)
            fp=np.asarray(s['world_footprint_xy_m']);self.assertTrue(np.all(fp.min(0)>[.595,.07]));self.assertTrue(np.all(fp.max(0)<[1.705,1.03]))
        shoulder=m.joint('special_lehisto_j1').id
        np.testing.assert_allclose(d.xanchor[shoulder],[1.0683823637,.5403695630398401,.922643457],atol=1e-7)

if __name__=='__main__':unittest.main(verbosity=2)
