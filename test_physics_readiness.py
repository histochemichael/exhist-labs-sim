"""Regression checks for the admission layer, passive CAD joints and contact bench.
These tests intentionally do not call a full-lab animation a physics pass.
"""
import json,math,unittest
from dataclasses import replace
import numpy as np
import mujoco
from handling_contracts import pose_check,grasp_check,insertion_check,release_check
from contact_bench import run_case,build
from operate_lab import ROOT,Scene
import xml.etree.ElementTree as ET

class Contracts(unittest.TestCase):
    def test_pose(self):
        def check(p=(0,0,0),r=None,calibrated=True):
            return pose_check(p,np.eye(3) if r is None else r,[.001]*3,math.radians(2),calibrated)
        self.assertTrue(check().allowed)
        self.assertFalse(check(p=(.002,0,0)).allowed)
        self.assertFalse(check(r=np.array([[0,-1,0],[1,0,0],[0,0,1]])).allowed)
        self.assertFalse(check(r=-np.eye(3)).allowed)
        self.assertFalse(check(p=(float("nan"),0,0)).allowed)
        self.assertFalse(check(calibrated=False).allowed)
    def test_grasp(self):
        valid=dict(normal_forces=[.2,.2],normal_force_range=[.02,.8],bilateral_dwell_s=.5,minimum_dwell_s=.2,
            relative_speed_mps=.001,max_speed_mps=.005,relative_slip_m=.0002,max_slip_m=.001,
            lifted_m=.01,minimum_lift_m=.005,joint_limits_ok=True,effort_limits_ok=True,unexpected_contact=False)
        self.assertTrue(grasp_check(**valid).allowed)
        for key,value in [("normal_forces",[.2,0]),("normal_forces",[1,.2]),("bilateral_dwell_s",.1),
                          ("relative_speed_mps",.1),("relative_slip_m",.005),("lifted_m",0),
                          ("joint_limits_ok",False),("effort_limits_ok",None),("unexpected_contact",True),
                          ("lifted_m",float("nan"))]:
            with self.subTest(key=key,value=value):self.assertFalse(grasp_check(**(valid|{key:value})).allowed)
    def test_insertion_release(self):
        valid=dict(object_extents=[.025,.075,.001],container_clear_extents=[.030,.080,.006],relative_rotation=np.eye(3),
            center_error=[0,0,0],clearance_m=.001,geometry_verified=True,swept_path_clear=True,orientation_ok=True)
        accepted=insertion_check(**valid);self.assertTrue(accepted.allowed)
        for key,value in [("container_clear_extents",None),("geometry_verified",False),("swept_path_clear",False),
                          ("orientation_ok",False),("center_error",[.005,0,0]),
                          ("relative_rotation",np.array([[0,-1,0],[1,0,0],[0,0,1]]))]:
            with self.subTest(key=key):self.assertFalse(insertion_check(**(valid|{key:value})).allowed)
        release=dict(insertion_decision=accepted,support_contact=True,settled_s=.5,minimum_settled_s=.2,relative_speed_mps=0,max_speed_mps=.001)
        self.assertTrue(release_check(**release).allowed)
        self.assertFalse(release_check(**(release|dict(support_contact=False))).allowed)
        self.assertFalse(release_check(**(release|dict(settled_s=0))).allowed)
        self.assertFalse(release_check(**(release|dict(relative_speed_mps=.01))).allowed)

class PhysicalBench(unittest.TestCase):
    def test_free_payload_not_actuated_or_welded(self):
        root=build(padded=True)
        self.assertIsNotNone(root.find(".//body[@name='payload']/freejoint"))
        self.assertFalse(root.findall(".//body[@mocap='true']"))
        self.assertFalse(root.findall("equality/weld"))
        self.assertTrue(all("payload" not in str(e.attrib) for e in root.find("actuator")))
    def test_contact_and_failure_cases(self):
        results=[]
        for name,offset,friction,release,padded in [("stock",0,.6,True,False),("padded",0,.6,True,True),
                ("missed",.065,.6,True,True),("slippery",0,0,False,True)]:
            report,trace=run_case(name,offset,friction,release,padded,render=name=="padded")
            results.append(report)
            (ROOT/f"contact_{name}_trace.json").write_text(json.dumps(trace,indent=2))
        stock,pad,miss,slip=results
        self.assertTrue(stock["passed_grasp"]);self.assertTrue(pad["passed_grasp"])
        self.assertLess(pad["relative_hold_slip_m"],stock["relative_hold_slip_m"])
        self.assertGreater(pad["release_drop_m"],.05)
        self.assertFalse(miss["passed_grasp"]);self.assertEqual(miss["maximum_fixture_lift_m"],0)
        self.assertTrue(miss["pose_rejection"])
        self.assertFalse(slip["passed_grasp"])
        self.assertIn("GRASP_LOST",[e["event"] for e in slip["events"]])
        self.assertTrue(all(r["effort_limits_ok"] and r["joint_limits_within_fixture_tolerance"] for r in results))
        (ROOT/"contact_bench_validation.json").write_text(json.dumps(dict(cases=results,full_lab_validated=False,
            scope="Actual CAD convex jaw contacts / nominal glass / test fixture only",
            limitations=["3x18x5 mm tip pads are an unbuilt concept, with assumed friction, attachment and compliance.",
                "Two-second hold only, not extraction from a loaded rack or a moving Nori arm.",
                "Intentional release/drop tests; not supported placement or glass fracture validation.",
                "No payload actuator, mocap, attachment weld or position reset after initialization."]),indent=2))

class LabHold(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.scene=Scene()
    def test_default_hold(self):
        s=self.scene;q=s.d.qpos.copy();p=s.d.mocap_pos.copy()
        for _ in range(100):s.lab.tick(.1);s.sync()
        self.assertTrue(np.array_equal(q,s.d.qpos));self.assertTrue(np.array_equal(p,s.d.mocap_pos))
        self.assertEqual(s.lab.time,0);self.assertEqual(s.lab.completed,0)
    def test_passive_doors_and_cad_limits(self):
        s=self.scene;m=s.m;d=s.d
        rows=json.loads((ROOT/"passive_door_articulation.json").read_text())["doors"]
        self.assertEqual(len(rows),5)
        initial=d.qpos.copy()
        for row in rows:
            ji=m.joint(row["joint"]).id;adr=m.jnt_qposadr[ji];bid=m.jnt_bodyid[ji]
            self.assertFalse(any(m.actuator_trnid[i,0]==ji for i in range(m.nu)))
            self.assertTrue(np.allclose(m.jnt_range[ji],row["limits"]))
            self.assertAlmostEqual(m.qpos0[adr],row["cad_pose_qpos0"])
            self.assertTrue(np.allclose(m.jnt_axis[ji],row["axis_local"]))
            d.qpos[:]=initial;mujoco.mj_forward(m,d)
            before=np.r_[d.xpos[bid],d.xmat[bid]].copy();anchor=d.xanchor[ji].copy()
            value=sum(row["limits"])/2;d.qpos[adr]=value;mujoco.mj_forward(m,d)
            self.assertGreater(np.linalg.norm(np.r_[d.xpos[bid],d.xmat[bid]]-before),.001)
            if m.jnt_type[ji]==mujoco.mjtJoint.mjJNT_HINGE:self.assertTrue(np.allclose(d.xanchor[ji],anchor))
        d.qpos[:]=initial;mujoco.mj_forward(m,d)

if __name__=="__main__":unittest.main(verbosity=2)
