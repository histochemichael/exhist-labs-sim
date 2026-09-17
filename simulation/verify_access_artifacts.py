"""Fail if any nominal report/recording is missing, stale or inconsistent."""
import json
import numpy as np
from build_scene import ROOT
from equipment_access import KINDS
from access_suite import source_hashes

def verify():
    rows=[];hashes=source_hashes()
    for kind in KINDS:
        r=json.loads((ROOT/f'access_{kind}_validation.json').read_text());s=np.load(ROOT/f'access_{kind}_states.npz')
        assert r['validation_ok'] and r['passed'] and r['source_sha256']==hashes,kind
        assert np.isfinite(s['qpos']).all() and np.all(np.diff(s['time'])>0) and s['phase'][-1]=='COMPLETE',kind
        for phase in ('APPROACH','GRIP','OPEN','HOLD','CLOSE','RELEASE','RETREAT','COMPLETE'):assert phase in s['phase'],(kind,phase)
        tolerance=.015 if kind=='quincy' else .003
        assert abs(r['final_q'])<tolerance and r['max_open']>=abs(r['open_target'])-(.03 if kind=='quincy' else .004),kind
        rows.append(r)
    jam=json.loads((ROOT/'access_st_load_jammed_validation.json').read_text())
    assert jam['validation_ok'] and jam['source_sha256']==hashes and jam['max_open']<.005
    miss=json.loads((ROOT/'access_quincy_miss_validation.json').read_text())
    assert miss['validation_ok'] and miss['source_sha256']==hashes and miss['phase']=='NO_GRASP' and miss['max_open']==0.
    lines=['# Equipment access — nominal contact validation','',
      'Six isolated, fixed-dock MuJoCo cycles. Nori uses its actual seven-joint left arm, stock left claw and force-limited lift. Each equipment joint is passive. These are simulation results, not hardware certification or full-lab transfer readiness.','',
      '| Mechanism | Maximum opening | Closed residual | Peak arm torque | Relative grip slip | Bilateral contact |',
      '| --- | ---: | ---: | ---: | ---: | ---: |']
    for r in rows:
        angular=r['kind']=='quincy';factor=180/np.pi if angular else 1000;unit='deg' if angular else 'mm'
        lines.append(f"| {r['kind']} | {r['max_open']*factor:.3f} {unit} | {abs(r['final_q'])*factor:.4f} {unit} | {r['max_arm_torque_Nm']:.3f} Nm | {r['relative_slip_m']*1000:.3f} mm | {r['bilateral_contact_fraction']*100:.3f}% |")
    lines+=['','## Safeguards and reproduction','',
      '- 0.25 ms physics step. Original 4 Nm arm caps; 1.5 Nm claw cap; provisional 150 N lift cap and 3 N commanded assist.',
      '- Actual measured handle pull above 12 N for 30 ms stops motion. This is a sustained-force stop, not a guarantee that brief peaks remain below 12 N.',
      f"- Deliberately jammed ST load drawer stops with {jam['max_open']*1000:.3f} mm maximum travel; missed-grasp regression never enters OPEN.",
      '- Joint limits, finger mimic, bilateral contact, slip, collision, timeout and force checks stay active. No live equipment pose assignment, actuator, weld or mocap attachment.',
      '- Only nested telescopic proxy overlaps and proximal claw-gear/socket overlaps are internally exempted. Distal fingers and external equipment contacts remain active.',
      '- Quincy uses the separate proposed 35 mm stand-off / 10 mm round pull; Leica drawers use the v02 U-pulls. Scanner uses its existing CAD handle. Original machine and printed slide/rack-gripper designs are unchanged.',
      '- CAD concepts: ExHist-Quincy-Pull-Concept-v01.step/.f3d and ExHist-Robot-Drawer-Pulls-v02.step/.f3d. Mounting, materials, latch forces, thermal/load suitability and manufacturing approval remain open.',
      '- `python access_suite.py` reproduces the positive cycles and physics-state recordings. `python -m unittest test_equipment_access -v` checks passivity and failure handling. `python verify_access_artifacts.py` rejects stale results.',
      '- `launch.ps1 -Access quincy` opens the real-time fixture. Other kinds: s60, st_load, st_unload, cv_load, ts_drawer.',
      '- Video: ExHist-Equipment-Access-Contact-Review.mp4 is a 3x replay of recorded dynamic states. The fixed head-side inset is a proposed camera bracket, not a calibrated hardware mount; external views complement any arm occlusion.',
      '', '## Remaining scope', '',
      'The chassis is fixed at each dock. The fixture checks the arm, selected door/drawer panels, handle and bench; it is not exhaustive cabinet/interior collision validation. No mobile approach, rack insertion/extraction or full-lab production readiness is claimed. The main workflow remains guarded. The special-stain canopy and 800 mm tables / higher Nori work posture are retained.',
      '', '## Provenance', '',
      'Controller, collision geometry, hinge interface and all six reach plans share the source hashes stored in access_final_validation.json. The source-hashed recordings are access_<kind>_states.npz; each report retains events and force traces.']
    (ROOT/'ACCESS_VALIDATION.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    summary=dict(passed=True,nominal_types=6,source_sha256=hashes,results=[{k:r[k] for k in ('kind','passed','max_open','final_q','max_arm_torque_Nm','max_lift_force_N','relative_slip_m','bilateral_contact_fraction','max_joint_limit_violation','max_left_claw_mimic_error_rad','max_measured_handle_pull_N')} for r in rows],jam_max_travel_m=jam['max_open'],full_lab_transfers_validated=False)
    (ROOT/'access_final_validation.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))

if __name__=='__main__':verify()
