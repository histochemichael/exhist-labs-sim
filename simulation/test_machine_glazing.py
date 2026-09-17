"""Visual glazing regressions: unchanged mechanics, solid fittings, repeatability."""
import copy
import unittest
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from build_scene import ROOT
from machine_glazing import apply, glazing_color


def original_view(root):
    """Undo only generated visual splits, for a before/after physics comparison."""
    root = copy.deepcopy(root)
    for parent in root.iter():
        for g in list(parent):
            if g.tag != 'geom':
                continue
            mesh = g.get('mesh', '')
            if mesh.startswith(('art_leica_', 'leica_workstation_')):
                if mesh.endswith('_glazing'):
                    parent.remove(g)
                elif mesh.endswith('_opaque'):
                    g.set('mesh', mesh[:-7])
    return root


def physics_equal(before, after):
    """Compare compiled physics parameters; visual geom indices may differ."""
    fields = ['qpos0', 'body_mass', 'body_inertia', 'body_ipos', 'body_iquat',
              'body_pos', 'body_quat', 'body_parentid', 'body_gravcomp',
              'jnt_type', 'jnt_axis', 'jnt_pos', 'jnt_range', 'jnt_solref',
              'jnt_solimp', 'dof_damping', 'dof_frictionloss', 'dof_armature',
              'actuator_gear', 'actuator_gainprm', 'actuator_biasprm',
              'actuator_forcerange', 'actuator_ctrlrange', 'eq_data',
              'eq_type', 'eq_obj1id', 'eq_obj2id', 'eq_active0']
    for field in fields:
        np.testing.assert_array_equal(getattr(before, field), getattr(after, field), err_msg=field)
    for field in ['geom_type', 'geom_size', 'geom_pos', 'geom_quat', 'geom_friction',
                  'geom_solref', 'geom_solimp', 'geom_contype', 'geom_conaffinity', 'geom_margin', 'geom_gap']:
        a = (before.geom_contype != 0) | (before.geom_conaffinity != 0)
        b = (after.geom_contype != 0) | (after.geom_conaffinity != 0)
        np.testing.assert_array_equal(getattr(before, field)[a], getattr(after, field)[b], err_msg=field)


class GlazingTests(unittest.TestCase):
    def test_exact_panel_selection(self):
        self.assertIsNotNone(glazing_color('CV/PHOTO_curved_canopy'))
        self.assertIsNone(glazing_color('CV/PHOTO_hood_front_edge'))
        self.assertIsNone(glazing_color('CV/QC_cover_knuckle_216'))
        self.assertIsNone(glazing_color('ST/Shallow console screen display'))
        self.assertIsNone(glazing_color('CV/CYCLE_cover_01'))

    def test_both_scenes_visual_only_and_idempotent(self):
        for filename, articulated in [('exhist.xml', False), ('exhist_operational.xml', True)]:
            with self.subTest(scene=filename):
                root = original_view(ET.parse(ROOT/filename).getroot())
                before = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
                rows = apply(root, articulated)
                self.assertTrue(rows)
                after = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
                physics_equal(before, after)
                first = ET.tostring(root)
                apply(root, articulated)
                self.assertEqual(first, ET.tostring(root))
                # The explicit existing shell and hinge topology still exists.
                for station in ('routine_a', 'routine_b'):
                    body = root.find(f"worldbody/body[@name='{station}_workstation']")
                    translucent = [g for g in body.iter('geom') if 0 < float(g.get('rgba', '1 1 1 1').split()[3]) < .3]
                    self.assertGreaterEqual(len(translucent), 3)

    def test_hidden_layers_stay_hidden(self):
        root = original_view(ET.parse(ROOT/'exhist_operational.xml').getroot())
        for station in ('routine_a', 'routine_b'):
            for g in root.find(f"worldbody/body[@name='{station}_workstation']").iter('geom'):
                g.set('rgba', '0 0 0 0')
        self.assertEqual(apply(root), [])

    def test_access_mechanics_identical(self):
        from equipment_access import build
        root, meta = build('st_unload')
        root = original_view(root)
        before = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        apply(root)
        after = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        physics_equal(before, after)
        a, b = mujoco.MjData(before), mujoco.MjData(after)
        for _ in range(100):
            mujoco.mj_step(before, a)
            mujoco.mj_step(after, b)
        np.testing.assert_allclose(a.qpos, b.qpos, atol=1e-13, rtol=0)
        np.testing.assert_allclose(a.qvel, b.qvel, atol=1e-13, rtol=0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
