"""Whole-lab loop integration guards, not whole-lab operational certification."""
import ast
import json
import unittest
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from build_scene import ROOT
from integrate_rail_loop import REFERENCE, MODEL, PREFIX
from lab_rail_loop import verify_snapshot


class IntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=mujoco.MjModel.from_xml_path(str(REFERENCE/'station.xml'))
        cls.m=mujoco.MjModel.from_xml_path(str(MODEL))
        cls.root=ET.parse(MODEL).getroot()
        cls.layout=json.loads((REFERENCE/'rail_loop_layout.json').read_text())

    def test_pinned_source_and_route(self):
        verify_snapshot()
        self.assertEqual(self.layout['route'],list(range(1,12))+[1])
        current=json.loads((ROOT/'special_rail_loop_layout.json').read_text())
        self.assertEqual(current,self.layout)

    def test_fresh_integrated_route_evidence(self):
        import hashlib
        result=json.loads((ROOT/'rail_loop_lab_validation.json').read_text())
        self.assertTrue(result['passed']);self.assertTrue(all(result['integration_checks'].values()))
        self.assertEqual(result['model_sha256'],hashlib.sha256(MODEL.read_bytes()).hexdigest())
        self.assertEqual(result['trace_sha256'],hashlib.sha256((ROOT/'rail_loop_lab_trace.json').read_bytes()).hexdigest())
        route=list(range(1,12))+[1]
        self.assertEqual([(r['source'],r['destination']) for r in result['completed_transfers']],list(zip(route[:-1],route[1:])))
        self.assertTrue(all(r['support_force_N']>.1 and r['lateral_error_m']<.003 for r in result['completed_transfers']))
        self.assertLess(result['max_slip_m'],.005)
        self.assertGreater(result['minimum_rail_clearance_m'],.015)
        self.assertGreater(result['minimum_transfer_clearance_m'],.015)
        self.assertEqual(result['max_rail_contact_N'],0)
        self.assertLess(result['released_rack_span_m'],.0001)

    def test_original_joint_force_and_contact_limits(self):
        a,b=self.original,self.m
        self.assertEqual((a.nq,a.nv,a.nu,a.neq),(b.nq,b.nv,b.nu,b.neq))
        for field in ('qpos0','jnt_range','jnt_type','jnt_axis','dof_damping','dof_frictionloss','dof_armature','actuator_forcerange','actuator_gainprm','actuator_biasprm','eq_data'):
            np.testing.assert_array_equal(getattr(a,field),getattr(b,field),err_msg=field)
        for i in range(a.nbody):
            name=a.body(i).name
            if i==0:continue
            ai=a.body(name).id;bi=b.body(name).id
            for field in ('body_mass','body_inertia','body_pos','body_quat'):
                np.testing.assert_array_equal(getattr(a,field)[ai],getattr(b,field)[bi],err_msg=name+' '+field)
        for i in range(a.ngeom):
            name=a.geom(i).name
            if name=='floor':continue
            ai=i
            if name:bi=b.geom(name).id
            else:
                owner=int(a.geom_bodyid[i]);self.assertNotEqual(owner,0)
                target=b.body(a.body(owner).name).id
                bi=int(b.body_geomadr[target]+i-a.body_geomadr[owner])
            for field in ('geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp','geom_size'):
                np.testing.assert_array_equal(getattr(a,field)[ai],getattr(b,field)[bi],err_msg=name+' '+field)
        self.assertEqual(b.nmocap,0)
        self.assertFalse(self.root.findall('.//weld'))
        self.assertAlmostEqual(float(b.body('rack').mass[0]),.145)

    def test_full_lab_visible_without_duplicate_station(self):
        wrapper=self.root.find("worldbody/body[@name='full_lab_context']")
        self.assertIsNotNone(wrapper)
        self.assertFalse(wrapper.findall('.//joint'))
        self.assertFalse(wrapper.findall('.//freejoint'))
        names={b.get('name') for b in wrapper.iter('body')}
        self.assertNotIn(PREFIX+'special_lehisto',names)
        for n in ('sorting_table','baking_table','routine_a_workstation','routine_b_workstation','special_table','special_canopy','sendout_table','imaging_a_s60_pc','imaging_b_s60_pc','lab_room','nori_mobile_base'):
            self.assertIn(PREFIX+n,names)
        for g in wrapper.iter('geom'):
            self.assertEqual(g.get('contype'),'0');self.assertEqual(g.get('conaffinity'),'0')
        lids=[g for g in wrapper.iter('geom') if 'art_leica_' in g.get('mesh','') and 0<float(g.get('rgba','1 1 1 1').split()[3])<.3]
        self.assertGreaterEqual(len(lids),6)

    def test_bounded_table_and_real_floor(self):
        m=self.m;d=mujoco.MjData(m)
        qa=m.joint('rack_free').qposadr[0]
        # Put the rack off the far end: no hidden bench-height plane may catch it.
        d.qpos[qa:qa+3]=[.6,.7,self.layout['seat_z']+.02]
        z=float(d.qpos[qa+2])
        for _ in range(400):mujoco.mj_step(m,d)
        self.assertGreater(z-d.body('rack').xpos[2],.2)
        self.assertIsNone(self.root.find("worldbody/geom[@name='floor']"))
        self.assertIsNotNone(self.root.find("worldbody/geom[@name='actual_room_floor_contact']"))

    def test_current_inspectors_match_bath_transforms(self):
        from special_layout import apply_static
        for filename in ('exhist.xml','exhist_operational.xml'):
            root=ET.parse(ROOT/filename).getroot()
            R=np.array(self.layout['world_rotation']);rq=np.zeros(4);mujoco.mju_mat2Quat(rq,R.ravel())
            for station in self.layout['stations']:
                name='special_staining_jar_'+str(station['id'])
                body=root.find(f"worldbody/body[@name='{name}']")
                self.assertIsNotNone(body)
                np.testing.assert_allclose(np.fromstring(body.get('pos'),sep=' '),station['world_position_m'],atol=1e-7)
                expected=np.zeros(4);mujoco.mju_mulQuat(expected,rq,np.array(station['quat_wxyz']))
                np.testing.assert_allclose(np.fromstring(body.get('quat'),sep=' '),expected,atol=1e-7)
            if filename=='exhist.xml':
                self.assertFalse([g for g in root.find('worldbody') if g.get('name','').startswith(('bucket_','reagent_'))])

    def test_raised_nori_posture_in_room_context(self):
        d=mujoco.MjData(self.m);mujoco.mj_forward(self.m,d)
        R=np.array(self.layout['world_rotation']);t=np.array(self.layout['world_translation'])
        shoulder=R@d.body(PREFIX+'left_shoulder_pitch_link').xpos+t
        top=R@d.geom(PREFIX+'special_top').xpos+t
        topz=top[2]+float(self.m.geom(PREFIX+'special_top').size[2])
        self.assertGreater(shoulder[2]-topz,.25)

    def test_no_new_runtime_payload_pose_assignments(self):
        # The adapter inserts a yield only; all source qpos writes precede the loop.
        tree=ast.parse((REFERENCE/'rail_loop_controller.py').read_text())
        loop=next(n for n in ast.walk(tree) if isinstance(n,ast.While))
        assignments=[n for n in ast.walk(loop) if isinstance(n,(ast.Assign,ast.AugAssign,ast.AnnAssign))]
        for node in assignments:
            targets=node.targets if isinstance(node,ast.Assign) else [node.target]
            self.assertFalse(any('qpos' in ast.unparse(t) for t in targets),ast.unparse(node))


if __name__=='__main__':unittest.main(verbosity=2)
