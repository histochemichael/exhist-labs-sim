import json,unittest
import numpy as np
import mujoco
import xml.etree.ElementTree as ET
from build_scene import ROOT,el,vec
from gripper_physics import section,triangles,triangulate,cross2,REGIONS
from groove_bench import build
from operate_lab import Scene

class GrooveUpdates(unittest.TestCase):
    def test_partition_area(self):
        for index in [14,15]:
            for low,high in REGIONS.values():
                p=section(triangles(index),(low+high)/2)
                area=sum(cross2(a,b) for a,b in zip(p,np.roll(p,-1,axis=0)))/2
                tri_area=sum(cross2(p[b]-p[a],p[c]-p[a])/2 for a,b,c in triangulate(p))
                self.assertAlmostEqual(area,tri_area,places=12)
    def test_no_convex_fill_inside_groove(self):
        root=build();b=el(root.find('worldbody'),'body',name='probe',pos='.014 0 .0915')
        el(b,'freejoint');el(b,'geom',name='probe',type='sphere',size='.0002',mass='.001')
        m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
        for name in ['lehisto_jaw_a_slide','lehisto_jaw_b_slide']:d.qpos[m.joint(name).qposadr[0]]=-.03
        d.qpos[m.joint('lehisto_pinion_joint').qposadr[0]]=-.03/.0072
        mujoco.mj_forward(m,d);probe=m.geom('probe').id
        hits=[c for c in d.contact if probe in (c.geom1,c.geom2)]
        self.assertEqual(len(hits),0,'Groove was filled by a convex contact hull')
    def test_model_mass_frames_and_pulls(self):
        s=Scene();m=s.m;d=s.d
        mass=json.loads((ROOT/'gripper_mass_model.json').read_text())
        self.assertLess(mass['total_kg'],.4);self.assertGreater(mass['total_kg'],.055)
        actual=sum(m.body_mass[m.body(name).id] for name in mass['body_inertials'])
        self.assertAlmostEqual(actual,mass['total_kg'],places=8)
        for robot in ['nori_right','sorter_1','sorter_2','special_lehisto','sendout_lehisto_1','sendout_lehisto_2']:
            for mode in ['handle','slide']:self.assertGreaterEqual(m.site(robot+'_'+mode+'_groove').id,0)
        kin=json.loads((ROOT/'machine_articulation.json').read_text())['joint_map']
        pulls=json.loads((ROOT/'drawer_pull_design.json').read_text())['pulls']
        q=d.qpos.copy();count=0
        for station in ['routine_a','routine_b']:
            for pull in pulls:
                row=next(r for r in kin if r['station']==station and r['cad_name']==pull['cad_joint'])
                ji=m.joint(row['dofs'][0]['name']).id;site=m.site(station+'_'+pull['cad_joint']+'_pull_grasp').id
                self.assertEqual(m.site_bodyid[site],m.jnt_bodyid[ji])
                self.assertFalse(any(m.actuator_trnid[i,0]==ji for i in range(m.nu)))
                d.qpos[:]=q;mujoco.mj_forward(m,d);p=d.site_xpos[site].copy();axis=d.xaxis[ji].copy()
                delta=sum(m.jnt_range[ji])/2;d.qpos[m.jnt_qposadr[ji]]+=delta;mujoco.mj_forward(m,d)
                self.assertTrue(np.allclose(d.site_xpos[site]-p,axis*delta,atol=1e-7));count+=1
        self.assertEqual(count,8)
    def test_groove_fixture_results(self):
        cases=json.loads((ROOT/'groove_bench_validation.json').read_text())
        self.assertEqual([r['passed'] for r in cases],[True,True,False,False])
        self.assertLess(cases[0]['relative_slip_m'],.001)
        self.assertLess(cases[1]['relative_slip_m'],.001)

if __name__=='__main__':unittest.main(verbosity=2)
