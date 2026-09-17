"""Regression checks for review choreography only; never mark physics success."""
import json,unittest
import numpy as np
import mujoco
from first_pass import Scene,ROOT

class FirstPassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.scene=Scene(guarded=False)

    def test_01_full_route_and_limits(self):
        s=self.scene;s.reset();max_limit=0.;pick_error=[];previous=None;closing_error=0.;retention_error=0.
        limited=np.where(s.m.jnt_limited)[0]
        for _ in range(12000):
            s.tick(.1)
            for ji in limited:
                q=s.d.qpos[s.m.jnt_qposadr[ji]];lo,hi=s.m.jnt_range[ji]
                max_limit=max(max_limit,lo-q,q-hi)
            tr=s.lab.transport
            for j in s.lab.jobs:
                if j.index<len(j.route) and j.route[j.index].name=='close_folder' and j.state=='PROCESSING':
                    for side,sign in [('L',-1),('R',1)]:
                        angle=abs(s.d.qpos[s.q[j.id+'_folder_hinge_'+side]])
                        if .01<angle<np.pi-.01:
                            body=s.m.body(j.id+'_folder_flap_'+side).id
                            target=s.d.xpos[body]+s.d.xmat[body].reshape(3,3)@np.array([sign*.075,-.13,0])
                            closing_error=max(closing_error,float(np.linalg.norm(s.d.site_xpos[s.m.site('nori_right_handle_groove').id]-target)))
                if j.folder_closed:
                    from folder_finish import SLOT_POINTS
                    root=s.m.body(j.id+'_slide_folder').id;r=s.d.xmat[root].reshape(3,3)
                    for n in range(j.folder_loaded):
                        bid=s.m.body(f'{j.id}_folder_slot_{n+1}').id
                        retention_error=max(retention_error,float(np.linalg.norm((s.d.xpos[bid]-s.d.xpos[root])@r-SLOT_POINTS[n])))
            if tr:
                moving=next(j for j in s.lab.jobs if j.id==tr['job'])
                if moving.carrier=='slide_folder':
                    self.assertTrue(moving.folder_closed)
                    self.assertAlmostEqual(s.d.qpos[s.q[moving.id+'_folder_hinge_L']],np.pi)
                    self.assertAlmostEqual(s.d.qpos[s.q[moving.id+'_folder_hinge_R']],-np.pi)
            if tr and tr['approach']+.6<=tr['elapsed']<=tr['approach']+.8:
                j=next(j for j in s.lab.jobs if j.id==tr['job'])
                from first_pass import grasp_offset,carrier_side
                sid=s.m.site('nori_left_carrier_grasp' if carrier_side(j.carrier)=='left' else 'nori_right_handle_groove').id
                expected=s.stage(tr['source'],tr['source_slot'],j.carrier)+grasp_offset(j.carrier)
                pick_error.append(float(np.linalg.norm(s.d.site_xpos[sid]-expected)))
            if s.lab.completed==36:break
        report=s.save();report['review_checks']=dict(max_joint_limit_violation=max_limit,max_near_pick_error_m=max(pick_error),joint_limits_checked=len(limited),max_flap_contact_tracking_error_m=closing_error,max_slide_relative_drift_m=retention_error)
        (ROOT/'first_pass_validation.json').write_text(json.dumps(report,indent=2))
        self.assertEqual(s.lab.completed,36)
        self.assertEqual(len(s.custody),50)
        self.assertTrue(all(j.folder_closed and j.folder_loaded==len(j.slides) and j.location=='finished_bench' for j in s.lab.jobs))
        self.assertEqual(len({j.slot for j in s.lab.jobs}),4)
        self.assertEqual(s.machines.slide_counts,{'B01':8,'B02':6,'B03':12,'B04':10})
        self.assertLess(max_limit,1e-7)
        self.assertLess(max(pick_error),.006)
        self.assertEqual(s.max_heading_error,0.)
        self.assertLess(closing_error,.002)
        self.assertLess(retention_error,1e-8)
        for batch in ('B01','B03','B04'):
            rows=[r for r in s.machines.rack_trace if r['batch']==batch]
            for bucket in range(2,26):
                self.assertTrue(any(r['detail']==f'Lift, travel, lower into bucket {bucket}' for r in rows),(batch,bucket))
            self.assertTrue({'stain','transfer','cv','empty'}.issubset({r['mode'] for r in rows}))
            points=np.array([r['rack_m'] for r in rows]);self.assertGreater(np.ptp(points[:,0]),.7)
        self.assertFalse(report['physical_success'])

    def test_02_pause_and_reset(self):
        s=self.scene;s.reset()
        for _ in range(200):s.tick(.1)
        s.lab.paused=True;s.sync();q=s.d.qpos.copy();p=s.d.mocap_pos.copy();t=s.lab.time
        s.tick(5)
        np.testing.assert_array_equal(s.d.qpos,q);np.testing.assert_array_equal(s.d.mocap_pos,p);self.assertEqual(s.lab.time,t)
        s.reset();self.assertEqual(s.lab.time,0);self.assertFalse(s.transfers);self.assertFalse(s.custody)
        self.assertAlmostEqual(s.d.qpos[s.q['base_x']],s.lab.x['sorting'])

    def test_03_cutaway_and_head_cameras(self):
        s=self.scene;s.reset()
        for name in ('nori_head_left','nori_head_right','nori_head_wide'):self.assertGreaterEqual(s.m.camera(name).id,0)
        before=s.d.mocap_pos.copy();s.machines.toggle_xray();s.sync()
        self.assertFalse(np.array_equal(s.d.mocap_pos,before));s.machines.toggle_xray();s.sync()

    def test_04_folder_gates_and_slot_fit(self):
        from folder_finish import FolderLab,SLOT_POINTS
        lab=FolderLab(self.scene.layout['stations']);job=lab.jobs[0]
        job.index=len(job.route)-2;job.holds='sendout';job.state='READY';job.slot=0
        with self.assertRaises(AssertionError):lab.start(job,'sendout',0)
        with self.assertRaises(AssertionError):lab.reserve(job,'finished_bench')
        job.folder_loaded=len(job.slides);job.folder_closed=True
        self.assertEqual(lab.reserve(job,'finished_bench'),0)
        self.assertEqual(len(SLOT_POINTS),20)
        self.assertGreater((.077-.075)/2,0)
        self.assertGreater((.027-.02495)/2,0)

if __name__=='__main__':unittest.main(verbosity=2)
