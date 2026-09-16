"""Static layout regressions only; these tests do not exercise Nori grasping."""
import copy,json,unittest
import xml.etree.ElementTree as ET
import mujoco,numpy as np
import qc_folder_cradle as qc

class QCCradleTests(unittest.TestCase):
    def test_source_is_coherent(self):
        s=json.loads((qc.SOURCE/'folder_stand_45.json').read_text())
        r=ET.parse(qc.SOURCE/'folder_slide_45.xml').getroot()
        f=r.find("worldbody/body[@name='folder_fixture']")
        np.testing.assert_allclose(np.fromstring(f.get('pos'),sep=' '),s['folder_center_m'],atol=1e-8)
        self.assertEqual(s['tilt_from_table_deg'],45)
        self.assertEqual(len(s['parts']),10)

    def test_integrity_and_only_fixed_additions(self):
        qc.verify_variant()
        r=ET.parse(qc.MODEL).getroot();f=r.find("worldbody/body[@name='full_lab_context']/body[@name='"+qc.PREFIX+"fixture']")
        self.assertIsNotNone(f)
        self.assertFalse(f.findall('.//joint'));self.assertFalse(f.findall('.//freejoint'))
        self.assertEqual(len(f.findall('.//geom')),13)
        for g in f.iter('geom'):
            self.assertEqual(g.get('contype'),'0');self.assertEqual(g.get('conaffinity'),'0')

    def test_guard_detects_unrelated_change(self):
        r=ET.parse(qc.MODEL).getroot();r.find('option').set('gravity','0 0 0')
        self.assertNotEqual(qc.strip_addition(r),qc.strip_addition(ET.parse(qc.BASE).getroot()))

    def test_standalone_asset(self):
        m=mujoco.MjModel.from_xml_path(str(qc.ROOT/'models/assets/qc_folder45.xml'))
        self.assertEqual(m.nq,0);self.assertEqual(m.ngeom,13)

    def test_scoped_layout_report_matches_model(self):
        r=json.loads((qc.ROOT/'qc_folder45_validation.json').read_text())
        self.assertTrue(r['passed']);self.assertTrue(all(r['checks'].values()))
        self.assertEqual(r['model_sha256'],qc.sha(qc.MODEL))
        self.assertTrue(all(not c['obstructions'] for c in r['access']))

if __name__=='__main__':unittest.main(verbosity=2)
