# Access controller development — nominal fixtures completed

Final current evidence is in `ACCESS_VALIDATION.md` and `access_final_validation.json`.
All six nominal contact cycles pass on one common source version. The missed
Quincy handle reports NO_GRASP with zero door travel; the jammed stainer drawer
stops after 1.534 mm. No passive equipment has an actuator, weld or live pose
setter. This completes isolated fixed-dock access, not full-lab transfers.

Final corrections after the historical notes below: 0.25 ms physics step,
smooth effort-ramped claw closure/release, a software empty-close stop, a narrow
proximal gear/socket proxy exemption, planned hinge approach with reduced wrist
rotation, actual contact-wrench overload monitoring and measured closed-stop
recognition. Drawer motors retain 4 Nm arm caps; no force limits were increased.
Quincy uses the separately exported proposed pull concept. Its original handle
and all source machine designs are unchanged.

The proposed fixed head-side camera in the review is editorial/simulation
mount geometry, not calibrated hardware. Lab route guards remain enabled.

## Historical debugging notes (superseded by the final reports)

The active request is to finish physical door/drawer cycles. Do not infer full-lab transfer readiness from these fixtures.

`equipment_access.py` uses the actual seven-joint left arm and force-limited lift. Equipment is passive: no live door pose assignments, door actuators, welds or mocap attachments. All imported arm effort caps remain 4 Nm. Current pull ceiling is 3 N and left-claw torque ceiling 1.5 Nm. Dynamics remain explicitly provisional.

Important corrections made during development:

- Isolated only the spurious contacts between nested telescopic lift tube bounding boxes. External tube collisions remain enabled.
- Planned whole-stroke arm/lift branches, corrected phase-transition timeout accounting and seeded redundant IK from its planned branch.
- Preserved the actual left-claw CAD recesses across both axial length and width. Earlier whole-width convex slices filled recesses.
- Selected the flat inner finger region: nominal grasp depth 114 mm for pulls, 116 mm for scanner handle. Original visual CAD and tested printed slide/rack gripper remain unchanged.
- Made acquisition hold its pose instead of chasing a handle displaced during closing; require bilateral contact, alignment and low equipment velocity.
- Found excessive soft joint-stop penetration under constant claw effort. Joint limits and the left finger mimic now use stiff constraints, with explicit violation monitoring. Old successes require revalidation after this correction.

Four drawer types completed cycles with the refined contact geometry before the latest stop/mimic and planned-branch corrections: ST5020 load/unload, CV5030 load and TS5025 transfer. Scanner and incubator remain under active revalidation. Read the latest reports; do not claim all six pass until the final common-source regression run does so.

The OEM Quincy handle mounting feet physically interfere with the original left claw. `quincy_offset_pull.py` is a separate **proposed** 35 mm stand-off, 10 mm diameter, 100 mm tall round pull with 80 mm clear space between supports. Its 80 g mass is assumed; mounting/fastening, temperature compatibility and load capacity are not validated. No original Fusion design has been altered and this candidate has not yet been exported to CAD or integrated into the lab XML.

`quincy_branch_search.py` creates the current fixed-dock 70-degree reach plan. Mobile-arc and short-segment scripts are experiments, not validated workflows. `quincy_oem_probe.py` tests the unmodified stock handle separately. `access_contact_probe.py` reports actual contact forces, alignment and finger linkage state.

Still to do: finish the nominal six-type positive regression, rerun missed-grasp/jam/constraint regressions, inspect rendered motion, provide an access viewer/video, document exact scope and any required adapter, and leave unrelated loaded-rack transfers guarded. Canopy and Nori height work from the earlier change are retained.
