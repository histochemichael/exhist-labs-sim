# LeHisto moving-rail loop in ExHist Labs

PASS: a fresh 313.249-second contact-driven run completed all eleven loaded
transfers, 1 → 2 → … → 11 → 1, with the original gripper and 25.4 mm inserts.
The rack was released and supported at every destination. The full-room view
preserves the current Leica glazing, canopy, furniture and raised Nori posture.

## Measured results

- Carriage: −60.0002 to +210.0011 mm; returns to its starting end.
- Peak rail speed: 19.63 mm/s, below the original 100 mm/s limit.
- Minimum rack clearance over rail: 50.97 mm; over container rims: 41.07 mm.
- Maximum relative grasp slip: 1.473 mm.
- Maximum seated lateral error: 0.526 mm.
- Rack/rail contact: zero. Joint-limit violations: none.
- Original 145 g loaded-rack model, ±1.5 Nm arm caps, joint limits and contact
  guards retained. No payload weld, hop reset or runtime rack-pose controller.
- The actual bounded tabletop replaces the isolated test's infinite support
  plane. The negative control confirms an off-table rack falls to the room floor.

The route completed and its trace was saved before a NumPy-boolean JSON export
error. That exporter is fixed. The report was recovered against the unchanged
model hash. The additional two-second released-rack hold was rerun from the
measured final state; it was stable. This recovery is recorded explicitly in
`rail_loop_lab_validation.json`, not described as a second full route run.

## Run and inspect

- `launch.ps1` or `launch.ps1 -LeHistoLoop`: complete lab, paused at startup.
- SPACE: start/pause. L: LeHisto detail. O: whole room. 1–8: stations.
- `launch.ps1 -Hold`: previous readiness-hold viewer. Access/mobility tools remain.
- `ExHist-Live-LeHisto-Rail-Loop.mp4`: 80.33-second review at 4× speed, including
  the actual gripper-camera view and whole-room overview.
- `lab_rail_loop.py --validate`, using the existing lerobot Python environment:
  rerun the live controller; `test_lab_rail_loop.py`: integration/evidence guards.

## Scope

This is the live LeHisto contact station within the full lab scene, **not a fully
coupled operational lab**. Other robots/equipment are held visual context.
Nori handoffs, neighbor-machine/person collisions, loose-slide loading, fluids
and hardware operation are not validated by this loop. The loaded-rack model
uses fixed slide visuals; it is not proof of individually retained loose slides.

The rail uses the existing 270 mm modeled stroke, not the full 600 mm extrusion.
The far-side crossover goes above the rail; this is not a 360-degree orbit around
both physical ends. The source experiment and older first-pass video remain
unchanged. Both current CAD/layout inspectors use the new eleven-bath placement.
