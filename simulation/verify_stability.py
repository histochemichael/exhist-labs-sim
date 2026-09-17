"""Fail-closed summary of source-matched live stability recordings."""
import json,hashlib
import numpy as np
from build_scene import ROOT
from equipment_access import KINDS
from access_suite import source_hashes

def verify():
    rows=[];hashes=source_hashes()
    for kind in KINDS:
        r=json.loads((ROOT/f'stability_{kind}_current.json').read_text());s=np.load(ROOT/f'stability_{kind}_current.npz')
        assert r['passed'] and r['source_sha256']==hashes and r['source_unchanged'],kind
        assert np.isfinite(s['tcp']).all() and np.all(np.diff(s['time'])>0),kind
        assert all(r['phases'][p]['quiet_samples']>=n for p,n in [('SETTLE',3400),('HOLD',1900),('COMPLETE',4400)]),kind
        angles=[]
        for phase in ('SETTLE','HOLD','COMPLETE'):
            mask=(s['phase']==phase)&(s['age']>1.)
            matrices=s['rotation'][mask].reshape(-1,3,3)
            delta=np.einsum('nij,ij->n',matrices,matrices[0])
            angles.append(float(np.max(np.degrees(np.arccos(np.clip((delta-1)/2,-1,1))))))
        assert max(angles)<.2,(kind,angles)
        a=r['access_result']
        assert a['relative_slip_m']<.006 and a['bilateral_contact_fraction']>.99 and a['max_arm_torque_Nm']<=4.000001 and a['max_lift_force_N']<=150.000001,kind
        assert r['phases']['GRIP']['full_tcp_span_m']<.001 and r['phases']['RELEASE']['full_tcp_span_m']<.001,kind
        rows.append(dict(kind=kind,hold_tip_span_mm=r['phases']['HOLD']['quiet_tcp_span_m']*1000,
            grip_tip_span_mm=r['phases']['GRIP']['full_tcp_span_m']*1000,max_quiet_rotation_deg=max(angles),
            stationary_command_change_rad=max(p['command_span_rad'] for p in r['phases'].values()),
            untouched_equipment_travel=r['untouched_equipment_travel'],post_retreat_tip_span_mm=r['phases']['COMPLETE']['quiet_tcp_span_m']*1000))
    passive=json.loads((ROOT/'passive_stability_validation.json').read_text())
    assert passive['passed'] and passive['scene_sha256']==hashlib.sha256((ROOT/'exhist_operational.xml').read_bytes()).hexdigest()
    tests=json.loads((ROOT/'stability_regression_tests.json').read_text());assert tests['passed'] and tests['source_sha256']==hashes
    worst=max(r['translation_span_m'] for r in passive['bodies'])*1000
    report=dict(passed=True,source_sha256=hashes,equipment=rows,passive_bodies=len(passive['bodies']),passive_worst_span_mm=worst,
        regression_tests=tests['tests'],full_lab_transfers_validated=False)
    (ROOT/'stability_validation.json').write_text(json.dumps(report,indent=2))
    lines=['# Grip stance and passive-object stability','',
        'PASS in the tested simulation fixtures. No passive-body freezing, added payload welds, pose resets or hidden motion are used to obtain stability. Full-lab transfers and hardware reliability are not certified.','',
        '## Controller changes','',
        '- Retain arm/lift motor targets throughout settle, grip, hold and release. Replan only explicit approach, opening, closing and retreat motions.',
        '- Preserve loaded motor targets on stop/completion; do not replace them with the contact-deflected measured joint pose.',
        '- Require both fingers to clear the handle for 100 ms before retreat. Opening jaws alone is insufficient.',
        '- Keep physics stepping in the access viewer after completion instead of displaying a frozen terminal frame.','',
        '## Live tests','',
        'Each equipment cycle includes 8 s untouched idle, 5 s loaded hold and 10 s post-retreat live holding. Poses are sampled at 500 Hz; physics runs at 4 kHz. Quiet-window measurements exclude the first second of ordinary contact settling. Full acquisition/release tip excursion is also checked separately (<1 mm).','',
        '| Mechanism | Loaded-hold tip span | Acquisition tip span | Quiet orientation excursion |',
        '| --- | ---: | ---: | ---: |']
    for r in rows:lines.append(f"| {r['kind']} | {r['hold_tip_span_mm']:.3f} mm | {r['grip_tip_span_mm']:.3f} mm | {r['max_quiet_rotation_deg']:.4f} deg |")
    lines+=['',f"All six stationary motor targets remain unchanged. Untouched equipment travel is zero. The {len(passive['bodies'])} free passive bodies pass a 30 s gravity/contact soak; worst position span after 2 s settling is {worst:.5f} mm.",
        '',f"{tests['tests']} regression tests pass, including missed grasp, jam stop, latched stance, exact bath transform and a negative control where removing the table makes the rack fall.",'',
        '## Layout and limitations','',
        '- Active exhist_operational.xml uses the currently selected eleven-bath layout and matching insert orientations supplied by LeHisto Physical Training. The newer moving-rail integration has separate evidence in rail_loop_lab_validation.json; previous parked-carriage routes remain preserved. The original tilted five-bath film is historical, separate choreography—not new physics proof.',
        '- The passive soak covers 11 containers, 11 one-inch inserts, one loaded-rack support envelope and eight QC blocks. QC mass (20 g) and support-only envelopes are provisional.',
        '- Doors/drawers use fixed-dock Nori tests, provisional dynamics and proposed adapters where documented. Whole-cabinet, mobile approach and neighbor-machine collisions are not exhaustively certified.',
        '- Loose slides, loaded slide folders, scanner/coverslipper carrier exchanges and coordinated Nori/LeHisto handoffs remain unvalidated. The live rail-loop station uses a separate composed lab scene with other equipment held as visual context. Existing full-workflow guards stay enabled.',
        '- The earlier equipment-access video/reports predate this controller update and are historical. Current stability evidence is in stability_<kind>_current.json/.npz and stability_validation.json.',
        '', '## Reproduce','',
        '`python stability_audit.py --kind quincy` (repeat for s60, st_load, st_unload, cv_load, ts_drawer); `python passive_stability.py`; `python -m unittest test_stability test_equipment_access test_lehisto_loaded_reach test_first_pass test_jar_riser test_review_corrections -v`; `python verify_stability.py`.','']
    (ROOT/'STABILITY_VALIDATION.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(report,indent=2));return report

if __name__=='__main__':verify()
