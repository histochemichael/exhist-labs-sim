# ExHist Labs - CAD layout and physics-readiness workbench

The lab is assembled from the user's Fusion designs. **Full-lab physical handling is not yet validated.**

## Moving-rail LeHisto loop in the complete lab — current launch

`launch.ps1` (or `launch.ps1 -LeHistoLoop`) opens the complete lab with the live
special-staining contact loop. It starts paused: SPACE runs/pauses, L focuses
LeHisto, O shows the whole room, and 1–8 select stations. `-Hold` retains the
previous read-only readiness view; `-Access`, `-Mobility`, and `-Preview` remain.

The imported eleven-bath route is 1→2→…→11→1. The carriage uses its original
modeled −60 to +210 mm stroke; the far-side transfer crosses above the extrusion.
This is not a 360-degree path around both ends of the physical rail. The rack
is a 145 g free body; the unchanged gripper physically grips it above the
25.4 mm inserts. Original limits and failure guards are retained.

`exhist_rail_loop.xml` includes the entire current lab, transparent machine
lids, canopy and raised Nori inspection posture. The LeHisto station runs live
MuJoCo contacts; other robots and equipment remain held visual context, not
coupled dynamics. Its support uses the bounded actual tabletop and room floor,
not an infinite bench-height plane. This does not validate Nori handoffs,
other-machine contacts, fluids, loose slides or physical hardware.

The pinned source lives in `rail_loop_reference`; the previous eleven-bath
parked-carriage evidence and historical first-pass film are preserved. The
canonical operational layout also uses the new bath transforms on rebuild.
Run `integrate_rail_loop.py` with the Nori environment to refresh the import.
Run `lab_rail_loop.py --validate` with the existing lerobot environment to
repeat all eleven transfers. Current new evidence: `rail_loop_lab_validation.json`.
The completed integration and limitations are summarized in `LAB_RAIL_LOOP_RESULTS.md`;
the new 80-second review is `ExHist-Live-LeHisto-Rail-Loop.mp4`.

The older sections below describe historical development stages; they do not
override current contact/stability reports or unlock full-lab transfers.

## Equipment access / canopy / chest height — 2026-09-15

The special-staining station now has its own open-bottom canopy (1080 x 760 mm,
underside 1700 mm). It is concept geometry, not a fume-containment or ventilation
design. Bench height stays 800 mm. Nori's review IK now prefers a higher lift
posture (example: 430 mm lift / 946 mm shoulder height) and solves **each actual
port height** instead of adding a height offset after IK. This prevents the old
offset method from exceeding the CAD 700 mm lift limit. Low folder poses remain
task-dependent. Full review regression passes with the existing guard retained.

`equipment_access.py` is a separate **actual-arm, fixed-dock development test**.
It supports the Quincy hinge, S60 slide and four Leica drawer interfaces using
their imported CAD joint limits. Only Nori's arm, lift and claw have controls;
equipment motion comes from contact. No door/drawer motor, weld, mocap or runtime
pose assignment is used. Approach, bilateral-force acquisition, paced opening,
hold, closing, release and retreat states stop on slip, contact loss, torque
saturation, collision, jam, unreachable pose or timeout. Failed states continue
physics for at least 0.5 s. Negative tests cannot count as successful access.

The completed nominal contact cycles and current source hashes are recorded in
`ACCESS_VALIDATION.md` and `access_*_validation.json`. Reproduce with
`python access_suite.py`; inspect with `launch.ps1 -Access quincy` (or `s60`,
`st_load`, `st_unload`, `cv_load`, `ts_drawer`). The separate fixed-dock test does
not unlock the full-lab transfer workflow: loaded extraction, insertion,
navigation and exhaustive cabinet/interior collisions are still separate work.

The unchanged left claw now uses 4 mm axial/distal-width CAD contact cells rather
than one hull that fills its hooked profile. Within-cell concavities remain
conservative. Only proximal gear/socket and nested lift-proxy overlaps are
exempted from internal contact; external finger contacts remain enabled.
Contact inspection showed the old 44 mm-clear drawer pull touching
a finger at its mounting leg. The v02 pull concept is 82 x 33 x 10 mm with 66 mm
clear width and 3.3 mm mounting bores. All eight simulated pulls use it; four
separate native Fusion solids and STEP export are in `ExHist-Robot-Drawer-Pulls-v02.*`.
Original machine designs/v01 exports were not overwritten. Fasteners, mounting
strength, grip robustness and manufacturer approval remain unvalidated.

The access fixture fixes the chassis (not wheel/docking validation), uses a
provisional 150 N lift limit, imported 4 Nm arm limits, 0.5 Nm claw limit, 2 N
drawer friction, 1.5 kg drawer mass and estimated door dynamics. Detailed fixed
machine cavities, all internal collisions, latches and seals are incomplete.
Nothing here authorizes hardware operation or certifies a physical grip.

Run `python equipment_access.py --kind quincy --view` for the first isolated
attempt. Other kinds: `s60`, `st_load`, `st_unload`, `cv_load`, `ts_drawer`.
`python -m unittest test_equipment_access -v` checks passivity, missed-grip/jam
failure behavior, unchanged effort caps, the canopy and pull geometry. These
safety-test passes must not be reported as successful opening/closing.

`special_canopy_nori_height.png` and `nori_height_review.json` show the layout and
posture measurements. The default lab viewer still stops at unvalidated access.

## One-inch 24-slide container risers

All 11 review containers now contain a separate passive **25.4 mm** insert.
The canonical jar builder and existing operational jars include the same insert.
Rack seating, Nori transfer targets and the five active LeHisto bath paths are
raised by exactly 25.4 mm. The grip center moves from 0.8 mm below the rim to
24.6 mm above it. Lateral rail travel/reach is unchanged.

`jar_riser.py` defines a 94 x 40 x 25.4 mm block with 6.2 mm clipped corners,
fitting the conservative jar cavity while supporting the rack-base footprint.
Each insert is a free body with gravity/contact support, no actuator or weld.
Mass is provisionally 120 g. Original Fusion jar/rack designs are not overwritten.
STL (millimetres), editable OpenSCAD and mesh JSON are in `assets/rack24_riser*`.
Material, chemical compatibility and fabrication tolerances remain unvalidated.
The raised slides reduce available immersion depth by 25.4 mm; reagent levels
and block displacement need checking before physical use. No fluid behavior or
reagent fill setting has been silently changed.

`python -m unittest test_jar_riser test_review_corrections -v` verifies insert
height/count, fit, passive gravity support (including a no-insert control),
raised target frames and the updated arm reach paths.

## Screenshot correction pass — 2026-09-15

The old three-minute film is superseded, not evidence of successful transfers.
`launch-first-pass.ps1` now runs a **guarded review**: it stops before unvalidated
equipment access. SPACE cannot bypass that hold; R resets the review. Neither
incubator nor scanner doors move from workflow timers anymore. Their existing
CAD handle locations are explicit sites, but a contact-driven opening/closing
controller is still required. Do not interpret the hold as completed integration.

Corrected in the review model (source Fusion files unchanged):

- 24-rack T-bar frame and 90-degree carrier orientation, based on the actual CAD
  and local real-world approach/YOLO pre-close recordings; original fingers retained.
- Rack bases and QC blocks placed inside tabletop support footprints; no elevated
  placeholder slide on an empty LeHisto gripper. QC block base gap is effectively zero.
- Unused Nori arms relax downward. Scanner carriers, CV output magazines, and
  closed folders have left-arm candidate poses; those grips are not contact-certified.
- Coverslipper enclosure visible by default; X deliberately selects cutaway.
- Eleven actual perpendicular jars fit the special-stain table. Only the central
  five (4–8) have verified seated/rim-clearing IK paths within the exported 270 mm
  rail stroke. The six darker outer jars are spares, not claimed reachable baths.
  The five-bath candidate choreography keeps the rack on its upper-handle frame.

`rack24_contact_check.py` separately tests the real 20 x 2 x 3 mm T-bar extents
with the original grooves, gravity and a free rack body. It sweeps provisional
0.8/2/4 N jaw-force limits at an assumed 145 g loaded mass, and checks deliberately
weak and 90-degree-misaligned grasps. It is a Cartesian fixture, not Nori; its
conservative lower-rack envelope and lumped mass do not validate loose slides.
The release/drop criterion is required as well as the lift/hold criterion.
At the current fixture settings, the 2 N case passes lift/hold/release; the 0.8 N
and 4 N cases lift but fail release. Therefore this sweep does **not** establish
a robust handling range or a hardware servo setting. Both negative controls fail
to acquire a grasp, as intended. These limitations are kept in the saved report.

Remaining: contact-driven incubator/scanner access; actual shelf/cassette insertion;
CV output extraction by Nori; complete CV magazine slot geometry; populated-CV-to-S60
slide exchange; loaded left-hand carrier/folder grasp and retention. No second Nori
or speculative replacement gripper has been installed. Scanner depth remains provisional.

Checks: `python -m unittest test_review_corrections test_first_pass test_workflow -v`.
`first_pass.py --check` checks the guarded stop. An explicit `--storyboard` bypasses
access holds **only for candidate-motion inspection**, never physical operation.
The film renderer requires `--storyboard` and writes a differently named,
explicitly unvalidated movie, preserving the historical film.

## First-pass workflow review (new)

Historical milestone below; superseded by the guarded correction pass above.
`ExHist-First-Pass-Three-Minute.mp4` is the 180-second narrated-by-caption review,
based on the previous `Leica-Workstation-Three-Minute-v3.mp4` CAD choreography.
The original `launch.ps1` still defaults to the physics-readiness hold.

The review completes four batches / 36 slides / 25 station transfers. Both Leica
workstations now display the actual bath route, supported TS transfer, individual
slide extraction/rotation/coverslip/output, and empty-rack return. The later snap
handles are registered onto the older film rack. Nori turns in the aisle, faces
the equipment at handoff, and carries review payloads at its original groove
frame. LeHisto robots perform representative slide-handling motions.

Folder finishing now uses the actual 20-place cardboard tray and both CAD flaps.
Nori places the assigned slides into individual slots, closes the left and right
flaps, then transfers each closed folder to a distinct position on the existing
General Work bench, relabeled FINISHED SLIDE FOLDERS. Completion is counted only
after this delivery. A partly populated case folder may close once every slide
assigned to that case is seated; unused slots do not need to be filled.
Nori owns the loading/closing task exclusively, so it cannot simultaneously run
another transfer. The current LeHisto placement could not meet all tested flat
slot poses, so this operation is assigned to Nori without changing the grippers.
Slot geometry comes from the CAD (77 x 27 mm recesses; 75 x 24.95 mm slide visuals).
Cardboard crease axes, flat staging-point supply, edge pinch and contact/retention
forces remain first-pass assumptions. `folder_finish.py` contains the geometry
and sequencing; `review_folders.py` renders the filled/closing/closed/finished views.
The folder work positions leave clearance to the rear rails; block-QC samples
are moved forward within the same bench in this review copy. Run
`python first_pass.py --folder-review --speed 2` to start at the finishing segment.

Controls: SPACE pause, R restart, X machine cutaway, 1-8 station views, O overview,
9 finished-folder bench, T follow Nori, V/B/N head left/right/wide, M oblique head interaction camera,
+/- playback speed, F inject scanner fault, C recover.

This is deliberately **not** a physical-transfer success claim: machine visuals
replay rigid CAD transforms; the old machine joint graph is retained at its
source pose, not dynamically driven. Nori/LeHisto and access-joint motion is
kinematic; wheel rotation is visual odometry. No contact/grip-force validation is
inferred from an attached visual payload. Incubator/scanner insertion, passive
drawer forces, special-rack re-racking and scanner/folder carrier exchanges are
still abstractions. Front staging ports are provisional. The cosmetic flexible
mountant hose is omitted from the rigid-only playback. Original CAD and the
contact-only development benches remain unchanged.

Rebuild: `python build_first_pass.py`; validate: `python test_first_pass.py`;
render the film: `python film_first_pass.py`. The builder/playback reference the
earlier local Leica film project through `first_pass_assets.json`; this is not yet
a portable standalone package. The first-pass output is `exhist_first_pass.xml`;
`exhist_operational.xml` and its physics-readiness behavior are preserved.
`first_pass_validation.json` records the regression run; interactive session
progress is kept separately in `first_pass_live.json`.

Regression coverage: the full review, 24 baths per routine batch, 8/6/12/10
covered-slide inventories, 25 paired visual handoffs, equipment-facing heading,
active joint limits, near-pick pose alignment, pause, reset and camera selection.
The original workflow fault/recovery and 14 docking/readiness tests also pass.

Newest: `arm_transfer.py --front` performs the slide transfer with Nori facing
the table, an approved provisional force-limited lift motor, and three fixed
head-mounted camera views. `head_camera_interactions.png` shows sampled object
interactions. `equipment_dock_bench.py` tests wheel-driven facing/stopped docking.
`simulation_parameters.json` clearly separates assumed values from hardware
specifications. These primitives are not yet one connected full-lab workflow.

Latest: `arm_transfer.py` completes an actual Nori-arm, contact-only slide
pick/lift/carry/place/release test. It uses development guide slots and an explicit
lift brake, not a completed machine-to-machine workflow. Run
`python -m unittest test_arm_transfer -v` for eight additional groups (25 total
with previous readiness tests). Matching CAD staining jars are now restored from
the `Staining Jar` body in `Slide Rack v13`. See the newest readiness-report section.

New: `launch.ps1 -Mobility` runs a separate wheel-driven Nori development test
(forward, body turn, forward, stop). It uses an explicit lift test brake and
assumed chassis/caster/wheel parameters. The passive-drawer contact test detects
slip and force faults but does not yet complete opening/closing. Run
`python -m unittest test_mobile_drawer -v` with the project runtime for six
regressions. See the new integration section in PHYSICS_READINESS.md for scope.

`launch.ps1` now opens a read-only **PHYSICS READINESS HOLD**. No passive objects or robots animate and no workflow completes. `launch.ps1 -Preview` explicitly enables the old **ANIMATION ONLY** workflow.

Start with [PHYSICS_READINESS.md](PHYSICS_READINESS.md) for current findings, CAD changes needed, seven regression tests, and the separate contact-only slide-grip bench. That report supersedes the historical first-layout notes below. Contact tests do not establish Nori arm, wheel, rack retention or bucket-insertion validity.

Latest correction: keep the physically tested original upper handle/lower slide grooves. `groove_bench.py` tests those grooves without added pads; `gripper_mass_model.json` replaces the erroneous all-steel mass with an explicit ~257 g construction estimate. Eight passive drawer-pull concepts and a separate Fusion/STEP CAD export are added. See the correction at the top of the readiness report.

## Historical first-layout milestone (not current physics status)

## Included
- Two LeHisto sorters, three Quincy 10GC ovens, two complete Leica ST5020/CV5030 workstations.
- One special-stain LeHisto with a provisional open reagent-bucket lane.
- Two send-out LeHistos, two actual 20-place slide folders, and provisional block QC samples.
- Two NanoZoomer S60 assemblies with their CAD monitors.
- Actual Leica racks and 24-slide racks.
- Nori A3 with the original 22 joints and three mimic constraints, plus three planar preview joints.
- Eight tables, named docking sites and handoff sites.

## Viewer
Run launch.ps1 in PowerShell. SPACE toggles the scripted station tour.
Keys 1-8 focus a station and place Nori in the aisle opposite it. O returns to the overview.
Use the mouse to orbit and zoom. The tour uses prescribed base positions and mj_forward.
It is a kinematic inspection preview, not autonomous navigation or physical wheel simulation.

## Files
- exhist.xml: self-contained MuJoCo model with relative asset paths.
- build_scene.py: reproducible CAD mesh conversion and table placement.
- run_lab.py: inspection viewer, preview rendering and validation.
- assets/*.json: body geometry, CAD colors and source names exported from Fusion.
- assets/*.obj: generated visual meshes; asset instances reuse these meshes.
- layout.json: dimensions, placements, station sites and assumptions.
- validation.json: compile and sampled straight-aisle geometry check.
- overview.png, sorting.png, sendout.png, imaging.png: actual MuJoCo renders.

## Rebuild and verify
Use the existing Python runtime:
C:/Users/Owner/Documents/Nori/.venv/Scripts/python.exe
Run build_scene.py, then run_lab.py --check --render.

## Current limits
Equipment and LeHisto geometry is static in this milestone. The CAD joints have not been exported.
The original Nori installation is untouched; its meshes are copied into this project.
Machine envelope colliders are conservative solid boxes, not suitable for rack insertion.
Table height (0.80 m), table placement and dock sites are provisional. The row is 14.85 m long.
The bucket geometry and block QC samples are placeholders; no reagent chemistry is simulated.
The S60 source is NanoZoomer-S60-branding-v05.f3d, based on door-fit-v04.
Front branding is CAD geometry approximated from the user's photo and shared by both simulated scanners.
The scanner's internal cassette depth remains provisional.
The Leica source is the Mechanical QC v2 workstation. The folder is Cardboard Slide Folder - 20 Place v1.
The Leica rack asset has 28 slides. The 24-slide rack is the actual Slide Rack v13 CAD body.
Validation checks compilation, preserved mimic constraints, site presence and 80 aisle positions.
It does not establish docking feasibility, grasp success, device behavior, throughput, or sim-to-real performance.
Next: export articulation and precise contact surfaces, measure handoff poses, and implement one rack transfer.
