# Grip stance and passive-object stability

PASS in the tested simulation fixtures. No passive-body freezing, added payload welds, pose resets or hidden motion are used to obtain stability. Full-lab transfers and hardware reliability are not certified.

## Controller changes

- Retain arm/lift motor targets throughout settle, grip, hold and release. Replan only explicit approach, opening, closing and retreat motions.
- Preserve loaded motor targets on stop/completion; do not replace them with the contact-deflected measured joint pose.
- Require both fingers to clear the handle for 100 ms before retreat. Opening jaws alone is insufficient.
- Keep physics stepping in the access viewer after completion instead of displaying a frozen terminal frame.

## Live tests

Each equipment cycle includes 8 s untouched idle, 5 s loaded hold and 10 s post-retreat live holding. Poses are sampled at 500 Hz; physics runs at 4 kHz. Quiet-window measurements exclude the first second of ordinary contact settling. Full acquisition/release tip excursion is also checked separately (<1 mm).

| Mechanism | Loaded-hold tip span | Acquisition tip span | Quiet orientation excursion |
| --- | ---: | ---: | ---: |
| quincy | 0.169 mm | 0.209 mm | 0.0272 deg |
| s60 | 0.125 mm | 0.190 mm | 0.0173 deg |
| st_load | 0.062 mm | 0.160 mm | 0.0078 deg |
| st_unload | 0.062 mm | 0.160 mm | 0.0085 deg |
| cv_load | 0.062 mm | 0.160 mm | 0.0086 deg |
| ts_drawer | 0.062 mm | 0.160 mm | 0.0086 deg |

All six stationary motor targets remain unchanged. Untouched equipment travel is zero. The 31 free passive bodies pass a 30 s gravity/contact soak; worst position span after 2 s settling is 0.00107 mm.

25 regression tests pass, including missed grasp, jam stop, latched stance, exact bath transform and a negative control where removing the table makes the rack fall.

## Layout and limitations

- Active exhist_operational.xml uses the currently selected eleven-bath layout and matching insert orientations supplied by LeHisto Physical Training. The newer moving-rail integration has separate evidence in rail_loop_lab_validation.json; previous parked-carriage routes remain preserved. The original tilted five-bath film is historical, separate choreography—not new physics proof.
- The passive soak covers 11 containers, 11 one-inch inserts, one loaded-rack support envelope and eight QC blocks. QC mass (20 g) and support-only envelopes are provisional.
- Doors/drawers use fixed-dock Nori tests, provisional dynamics and proposed adapters where documented. Whole-cabinet, mobile approach and neighbor-machine collisions are not exhaustively certified.
- Loose slides, loaded slide folders, scanner/coverslipper carrier exchanges and coordinated Nori/LeHisto handoffs remain unvalidated. The live rail-loop station uses a separate composed lab scene with other equipment held as visual context. Existing full-workflow guards stay enabled.
- The earlier equipment-access video/reports predate this controller update and are historical. Current stability evidence is in stability_<kind>_current.json/.npz and stability_validation.json.

## Reproduce

`python stability_audit.py --kind quincy` (repeat for s60, st_load, st_unload, cv_load, ts_drawer); `python passive_stability.py`; `python -m unittest test_stability test_equipment_access test_lehisto_loaded_reach test_first_pass test_jar_riser test_review_corrections -v`; `python verify_stability.py`.
