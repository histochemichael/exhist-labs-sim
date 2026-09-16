# Equipment access — nominal contact validation

Historical recording: the controller was subsequently updated for latched grip stance and live terminal holding. See STABILITY_VALIDATION.md and the source-matched stability reports for the current controller. The video and measurements below retain their original provenance.

Six isolated, fixed-dock MuJoCo cycles. Nori uses its actual seven-joint left arm, stock left claw and force-limited lift. Each equipment joint is passive. These are simulation results, not hardware certification or full-lab transfer readiness.

| Mechanism | Maximum opening | Closed residual | Peak arm torque | Relative grip slip | Bilateral contact |
| --- | ---: | ---: | ---: | ---: | ---: |
| quincy | 70.197 deg | 0.0000 deg | 3.055 Nm | 1.532 mm | 99.986% |
| s60 | 199.833 mm | 0.4451 mm | 2.667 Nm | 0.141 mm | 99.999% |
| st_load | 179.007 mm | 0.0000 mm | 2.477 Nm | 1.286 mm | 99.992% |
| st_unload | 179.099 mm | 0.0000 mm | 2.361 Nm | 1.334 mm | 99.992% |
| cv_load | 179.116 mm | 0.0000 mm | 2.377 Nm | 1.345 mm | 99.992% |
| ts_drawer | 179.107 mm | 0.0000 mm | 2.238 Nm | 1.244 mm | 99.997% |

## Safeguards and reproduction

- 0.25 ms physics step. Original 4 Nm arm caps; 1.5 Nm claw cap; provisional 150 N lift cap and 3 N commanded assist.
- Actual measured handle pull above 12 N for 30 ms stops motion. This is a sustained-force stop, not a guarantee that brief peaks remain below 12 N.
- Deliberately jammed ST load drawer stops with 1.534 mm maximum travel; missed-grasp regression never enters OPEN.
- Joint limits, finger mimic, bilateral contact, slip, collision, timeout and force checks stay active. No live equipment pose assignment, actuator, weld or mocap attachment.
- Only nested telescopic proxy overlaps and proximal claw-gear/socket overlaps are internally exempted. Distal fingers and external equipment contacts remain active.
- Quincy uses the separate proposed 35 mm stand-off / 10 mm round pull; Leica drawers use the v02 U-pulls. Scanner uses its existing CAD handle. Original machine and printed slide/rack-gripper designs are unchanged.
- CAD concepts: ExHist-Quincy-Pull-Concept-v01.step/.f3d and ExHist-Robot-Drawer-Pulls-v02.step/.f3d. Mounting, materials, latch forces, thermal/load suitability and manufacturing approval remain open.
- `python access_suite.py` reproduces the positive cycles and physics-state recordings. `python -m unittest test_equipment_access -v` checks passivity and failure handling. `python verify_access_artifacts.py` rejects stale results.
- `launch.ps1 -Access quincy` opens the real-time fixture. Other kinds: s60, st_load, st_unload, cv_load, ts_drawer.
- Video: ExHist-Equipment-Access-Contact-Review.mp4 is a 3x replay of recorded dynamic states. The fixed head-side inset is a proposed camera bracket, not a calibrated hardware mount; external views complement any arm occlusion.

## Remaining scope

The chassis is fixed at each dock. The fixture checks the arm, selected door/drawer panels, handle and bench; it is not exhaustive cabinet/interior collision validation. No mobile approach, rack insertion/extraction or full-lab production readiness is claimed. The main workflow remains guarded. The special-stain canopy and 800 mm tables / higher Nori work posture are retained.

## Provenance

Controller, collision geometry, hinge interface and all six reach plans share the source hashes stored in access_final_validation.json. The source-hashed recordings are access_<kind>_states.npz; each report retains events and force traces.
