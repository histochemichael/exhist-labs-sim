# ExHist: physical handling readiness

Status, 2026-09-14: **full-lab physical operation is blocked, not validated**.

## Equipment access update — 2026-09-15

See `ACCESS_VALIDATION.md` for source-matched nominal opening/closing tests of
the Quincy hinge, S60 slide door, ST5020 load/unload drawers, CV5030 load drawer
and TS5025 transfer drawer. These are actual-arm contact fixtures, with a fixed
chassis dock and provisional dynamics. Equipment has no actuator, grasp weld or
live pose setter. Missed grasps and excessive measured pull force stop progress.
The viewer is `launch.ps1 -Access <kind>`; the recorded-physics review is
`ExHist-Equipment-Access-Contact-Review.mp4`.

Quincy needs the separate proposed stand-off pull concept in this test; Leica
drawers use the v02 proposed U-pulls. Original machine CAD is untouched. The
printed green parallel slide/rack gripper is unchanged. This is not approval
to fabricate adapters or run hardware. Full-lab insertion and transfer guards
remain enabled. Historical findings below are superseded only for these
isolated access cycles.

`lehisto_loaded_reach.json` imports the independently tested nominal 200–330 mm
loaded radius, with a separate 10 mm design margin. Actual placed shoulder and
carriage transforms are used. `lehisto_lab_reach_audit.json` flags the different
lab height/tilt/rail trajectory; radial inclusion alone cannot admit a transfer.

## Equipment-facing operation and head cameras

The user approved explicitly provisional simulation parameters while keeping
hardware validation separate. `simulation_parameters.json` records that scope.
The new front-facing transfer mode replaces the test lift brake with a 150 N
force-limited lift servo (5,000 N/m stiffness, 200 Ns/m damping), retains CAD
travel limits and the middle-stage coupling, and leaves original CAD/source XML
unchanged. One collision pair between the nested lift's solid bounding proxies
is excluded; external lift collisions remain active. This is not validation of
the actual hollow telescoping geometry or its mechanism.

Run `arm_transfer.py --front` for the front-facing physical transfer and camera
report. Nori's chassis faces the table along its +X forward axis; the right arm
works in the front-right reachable area. The robot must be stopped and aligned
for 0.3 seconds before approaching. Heading departure over 3 degrees or tilt over
12 degrees faults the test. At the nominal 1 ms timestep, the sequence completes
with approximately 0.420 mm final placement error, 0.443 mm maximum relative slip,
over 99.9% bilateral contact and 2.915 Nm peak arm effort. Maximum heading error is
0.049 degrees; provisional lift effort peaks around 99.62 N, below its 150 N cap.

The initial half-timestep check exposed a 3.42 mm release shift, correctly failing
the placement tolerance. The front-mode provisional guide now has physical end
stops (25.6 x 1.4 mm clear slot, 20 mm tall), and release opens the jaws by 5 mm
per jaw over 1.5 seconds before retreating. No weld or relaxed tolerance was added.
Final error is 0.420 mm at 1 ms and 0.560 mm at 0.5 ms. These locating slots are
development fixtures, not a claim that existing equipment carriers have this design.

Three **fixed head-mounted** cameras are added to the new test and lab XML:
`nori_head_left`, `nori_head_right`, `nori_head_wide`. Their positions are anchored
to the imported head/visor geometry under `lift_top_link`, so base yaw and lift
travel carry the cameras physically. Their FOV and downward angles are provisional
simulation extrinsics, not calibrated real-camera parameters. They do not track
the slide or move independently to improve a result.

At sampled approach/grasp/lift/carry/release/retreat transitions, renderer
segmentation checks that at least one view contains the slide and both finger
meshes. The slide is visible in all three views at the tested transitions.
`head_camera_interactions.png` is the real rendered montage, and
`front_transfer_validation.json` contains pixel counts, contact and motion data.
These are sampled visibility checks, **not continuous occlusion guarantees or a
recorded training video**. Complete rack and drawer scenes still need their own
visibility checks.

`equipment_facing_docks.json` defines front-facing headings for all eight lab
stations. The current row requires +Y-facing chassis headings (90 degrees).
These are heading contracts; legacy dock stand-off and reach still need physical
validation. `equipment_dock_bench.py` separately drives and turns Nori through
wheel forces with the provisional lift servo, then admits interaction only after
the facing/stopped dwell check. It stops with about 1.39 degrees heading error.
No base pose commands occur after initialization.

`test_front_docking.py` adds seven groups covering facing/motion rejection, invalid
inputs, station headings, fixed head-frame motion, actual wheel docking, front-arm
transfer with camera evidence, and timestep comparison. All seven pass. The previous
25 baseline transfer/readiness groups were also rerun successfully. **The full-lab controller is still
in readiness hold**: physical loaded-rack transfers, LeHisto integration, drawer
handling and machine/container insertion have not been connected end-to-end.

## Actual-arm slide transfer milestone

`arm_transfer.py` now completes a development transfer using Nori's actual seven
arm joints: approach, two-jaw grasp, 40 mm lift, 100 mm horizontal carry, lower,
support confirmation, release and retreat. The slide is a free body. There are no
payload welds, mocap updates or payload pose assignments after initialization.
`pose_control.py` solves position AND orientation on scratch state, respects joint
limits, and leaves the live state untouched. Only actuator commands drive the arm.
Planning rejects sampled tabletop collisions; this is not a complete collision
audit of every imported robot surface.

At a 1 ms timestep the test achieves 39.73 mm peak lift, 0.767 mm maximum relative
slip, 99.853% bilateral-contact coverage and 0.461 mm final placement error.
The arm retains 4 Nm imported effort caps; the gripper cap is unchanged at 0.8 N.
Numerical friction refinement (`noslip_iterations=20`) prevents the creep seen
with the original solver settings. Gear equalities use a 2 ms solver time constant;
these are numerical contact/coupling settings, not changed physical fingers or
claims of real servo calibration. The original proven grooves remain unchanged.

`test_arm_transfer.py` passes eight groups, including successful 1 ms and 0.5 ms
rollouts, agreement of final placement, disabled-grip rejection, misaligned-slide
rejection, scratch-state isolation, passive payload structure and an open-jar probe.
All 17 prior readiness/mobile/gripper test groups also pass (25 groups total).
The full detailed rollout is in `arm_transfer_validation.json`; the four rollout
summaries are in `arm_transfer_regression.json`.

Important limits: this uses **development guide slots, not real rack slots**, an
explicit lift test brake, assumed base/caster parameters, and incomplete imported
robot contact proxies. It starts in a planned pre-grasp posture. It does not include
wheel travel while carrying, LeHisto sorting, loaded-rack retention, doors/drawers,
machine insertion or a glass-fracture model. A successful primitive is not successful
end-to-end lab processing. Main lab launch therefore remains readiness hold.

The matching 24-slide jar is now located and exported from **Slide Rack v13 /
Staining Jar**, not fabricated or substituted. Six genuine visual CAD jars are
restored to the special-stain table as passive free bodies in the operational XML.
Fusion face inspection establishes 98 x 44 mm inner flat dimensions, a -1.5 mm
inside floor and 91.8 mm rim in source coordinates, with 5 mm inner corner radii.
`cad_jar.py` preserves the open cavity using documented approximate contacts;
corner blocks and an un-beveled rim reduce clearance, and bottom blends are
simplified. Empty mass/inertia are provisional. Actual robot/rack insertion is not
yet tested. `export_rack24_jar.py` reproduces the export without editing source CAD.

The user has now approved explicitly provisional lift force, drawer resistance
and scanner-cassette dimensions for simulation-only integration. See the newer
equipment-facing section above. No unmeasured parameter is verified hardware;
scanner insertion and full-lab integration remain unfinished.

## Continued integration: physical mobility and guarded drawer tests

`mobile_base_bench.py` now runs the complete imported Nori on a free chassis with
wheel velocity actuators and `mj_step`. It drives 450 mm, turns approximately
90 degrees, drives another 350 mm and stops. No base pose is assigned after
initialization. Arm servos retain imported effort limits. The chassis is assumed
10 kg, wheel effort is provisionally capped at 1.5 Nm, and rear ball joints are
rolling-caster proxies. An explicitly named TEST FIXTURE brake holds the lift;
this is not a verified brake or a solution to the imported 1 N lift limit.
This standalone test does not validate navigation around lab obstacles or carrying.

Run `launch.ps1 -Mobility` for that live, wheel-driven development test. Default
launch still holds the full lab; `-Preview` remains animation-only. These are
separate modes, not a claim that the lab's old pose-driven controller is fixed.
`mobile_base_validation.json` records actual pose, stopped speeds, tilt, lateral
velocity and per-actuator peak efforts with units. `mobile_base_trace.json` records
wheel rotation and chassis motion. Disabled wheel motors do not navigate, and a
tilt fault remains latched after the pose recovers.

`drawer_contact_bench.py` uses the original left claw on an explicitly actuated
Cartesian test fixture, with a passive drawer slider. There is no drawer actuator,
weld, mocap or post-initialization pose assignment. It tests the original concept's
44 mm pull opening and a 64 mm test variant. **Neither completes the transfer**:
both trigger the 4 mm slip guard. A 30 N friction / 1 N actuator-cap negative case
triggers a force stop. Physics continues for 0.5 seconds after each stop to measure
residual drawer motion. Successful detection of these failures is not successful
drawer operation. Nominal 1.5 kg drawer mass, 2 N friction, 0.5 Nm grip cap, contact
friction and thresholds are assumptions, not machine measurements.

The left fingertips' approximately 55 mm distal bounding span flags the 44 mm
opening for an exact swept-geometry review, **not a proven clearance rejection**.
The bench uses conservative convex claw hulls and excludes coupled-finger self
contact; it cannot certify the real claw's contact surfaces. The wider test variant
also slips, so widening alone is not an established fix. Source CAD and the v01
pull exports are unchanged. The physically tested right/LeHisto grooves remain intact.

`test_mobile_drawer.py` contains six regressions: drive/turn/stop, disabled motors,
latched tilt fault, passive drawer structure, slip rejection and blocked-drawer
force rejection. Carrier retention, full six-dimensional arm alignment, exact
container interiors, supported insertion/release, machine latch/door forces and
full-lab controller integration **remain unfinished**. The matching 24-slide jar
has since been identified and restored as described in the newer milestone above.

## Correction after physical-build evidence

The prior flat-gap rejection and 1.03 kg hardware-weight inference were wrong for
this printed gripper. The user has physically printed and tested slide/rack pickup.
The image identifies the upper handle groove and lower slide groove. The earlier
"LeHisto Physical Training" task also records a successful lift-and-return
demonstration; its later 25-episode dataset concerns centered approaches, not a
general reliability certification. I reviewed that task's records, not the raw
training videos in this pass.

**Keep the original tested fingers.** No extra pads or replacement slide fingers
are required on the basis of the old flat-gap calculation. The original concave
grooves now use CAD-section convex partitions on Nori and all five LeHistos, with
separate oriented handle/slide grip frames. The legacy single convex jaw hull
filled these recesses. A regression probe verifies that the new groove stays open.

`groove_bench.py` is the current contact bench. The original lower groove lifts a
25 x 75 x 1 mm slide and the upper groove lifts a 20 x 2 x 7.5 mm handle-top-bar
coupon with a 9 mm neck, with no added pads, welds or payload pose commands. Both
hold for two seconds, with about **0.21 mm / 0.44 mm** relative motion. Misalignment
and near-zero closing force fail. This is a fixture/coupon test, not a loaded-rack
transfer or proof of hardware reliability. `contact_bench.py` and its pad results
are retained as **superseded historical experiments**, not the current design.

All 45 gripper bodies in Fusion had **Steel** assigned, including the printed
jaws, frame, camera bracket and electronic assemblies. The replacement
construction-based model is **0.2566 kg**, or **0.3332 kg** in a more conservative
material/tube/camera/cable scenario. It uses the manufacturer STS3215 mass of
55 +/- 1 g ([Feetech](https://www.feetechrc.com/products.html?keyword=STS3215)),
107.06 cm3 of printed parts at an assumed solid-PLA density of 1.24 g/cm3
([Polymaker](https://polymaker.com/wp-content/tech-docs/PolyLite_PLA_PIS_EN_V1.1.pdf)),
and two 125 mm aluminum tubes with 6 mm OD / assumed 4 mm ID. The
[Arducam B0332 datasheet](https://www.welectron.com/mediafiles/productimg/arducam/Amazon/B0332_OV9281_Global_Shutter_UVC_Camera_Datasheet.pdf)
does not give a net camera mass; 35 g nominal / 50 g conservative are allowances,
not sourced specifications. Filament, infill, tube ID and final wiring remain
unmeasured. This supports the user's sub-kilogram expectation, not a measured weight.

Mass and approximate inertia distribution are updated in the lab's Nori gripper.
The same exported-pose shoulder gravity term is now **2.81 Nm**, below the imported
4 Nm cap. That retires the previous 5.90 Nm warning for this pose; it does not
validate every loaded trajectory or the still-missing arm controller.

Eight proposed U-pulls (four per workstation) now follow the existing load,
unload, coverslipper-input and transfer-station drawer joints. They are passive,
with grasp/approach markers and no new drawer actuators. Four source-CAD-positioned
pull concepts, including 3.3 mm mounting bores, are exported separately as
`ExHist-Robot-Drawer-Pulls-v01.f3d` and `.step`; original machine CAD is untouched.
Fastening, real panel strength, latch forces, left-hand access and swept clearance
still need validation. `test_groove_updates.py` passes four additional test groups:
partition area, groove void, mass/frames/eight joint-following pulls, and bench results.

The old animation completed workflows by assigning joint coordinates and carrier
poses. It did not prove wheel motion, grasps, insertion or real equipment cycles.
The default launcher now opens a read-only readiness hold: robots, carriers and
passive doors do not animate and no batches are marked physically completed.
`launch.ps1 -Preview` explicitly enables the old **ANIMATION ONLY** view.

## Earlier readiness pass (pad experiment superseded above)

- A separate MuJoCo contact bench with the imported Nori/LeHisto jaw geometry,
  coupled jaw/pinion joints, force-limited fixture servos and a free glass slide.
  It uses `mj_step`, not payload position commands, mocap, attachment welds or
  adhesion. Its upstream lift is a test fixture, **not Nori's arm**.
- Pickup admission checks position and orientation. Lifting waits for 120 ms of
  two-jaw contact. Loss of contact or more than 2 mm relative slip stops further
  commanded lifting. A two-second hold must pass before a grasp is accepted.
- Both stock jaws and a **3 x 18 x 5 mm flat-pad concept** pass the nominal slide
  lift/hold test. Relative movement during the hold is approximately 1.10 mm and
  0.26 mm respectively. A misaligned slide is rejected; zero-friction contact slips
  and is rejected. Opening the jaws in the intentional drop test lets gravity move
  the glass. This is NOT a supported-placement demonstration.
- Reusable fail-closed checks cover six-dimensional alignment, two-jaw force,
  dwell, slip, joint/effort limits, container clearance, verified insertion paths,
  support contact and settling before release. Pose/grasp checks run in the bench;
  insertion/release checks are unit-tested but not integrated into a lab transfer.
- The three Quincy doors and two S60 doors now have passive CAD joints, exported
  pivots, axes and limits. The existing Leica joint imports are preserved. These
  new door joints have **no actuators or automatic motion commands**. Door masses,
  inertia and friction remain estimates and their contact geometry is not ready.
- Seven regression tests pass, including negative subcases, default no-motion
  hold, five passive door mappings, and the contact-only bench.

## Problems found in the actual assets

| Finding | Consequence / required change |
| --- | --- |
| Full lab: 257 joints, only one actuator, 20 mocap carrier variants | Joint-count coverage is not dynamics. Add physically driven robots, passive payloads and measured mechanical properties before enabling operation. |
| Distal flat gap is about 13 mm; handle neck is 9 mm | This comparison does NOT reject the actual upper-groove/top-bar grasp. Preserve the physically tested fingers and validate the correct grip surfaces. |
| All-steel CAD mass was 1.0305 kg | Superseded by the construction-based 0.2566 kg estimate; hardware weighing remains desirable. |
| Imported right shoulder-roll cap is 4 Nm; revised exported-pose gravity is about 2.81 Nm | This pose is below that cap with the corrected mass estimate. Dynamic loaded reach and actuator implementation still need validation. |
| Lift cap is 1 N; revised generalized gravity term is about 72.3 N | Importer/controller data remain incomplete. This is an unconstrained gravity term, not measured motor force or a complete transmission/mimic load calculation. |
| Glass is 25 x 75 x 1 mm; typical slide pitch is 3.535 mm, leaving 2.535 mm | Retain this geometry in the sim and test the demonstrated lower-groove extraction path. Do not infer a finger redesign from pitch alone. |
| 28 glass slides alone weigh about **131 g** at assumed density 2500 kg/m³ | Rack, handle, liquid and acceleration loads are additional. The low-force slide bench does not establish loaded-rack capacity. |
| Detachable handle is visually rigidly merged with the rack | The sim hides handle detachment. Model and test its retention separately; both jaws touching the handle does not prove the rack is secured. |
| Tool points used by the legacy IK are provisional; it solves position only | Calibrate the actual contact frame and approach axes. Solve full pose with limits, self-collision, scene collision and effort checks; fail if unreachable. |
| Base is moved using independent X/Y/yaw preview coordinates | It can slide sideways and turn independently of wheel travel. Replace with wheel-driven dynamics, odometry and body-heading control. CAD nominal wheel radius is 76.2 mm, track 300 mm. |
| Rear caster contacts are fixed spheres | They are not articulated rolling/swiveling casters; turning resistance and support geometry need an appropriate model. |
| Solid machine-envelope collision boxes and visual-only internals | They block real insertion and/or omit collisions. Build cavity, mouth, rail, stop and slot collision geometry from the correct CAD; preserve clearance under every joint pose. |
| Slides are merged into rack meshes; rack types change visually | Make slides independent passive bodies supported by actual rack slots. Coverslipping/scanning conversions must physically unload and reload them, never swap geometry to declare success. |
| Matching 24-slide buckets are absent; S60 cassette depth is provisional | Bucket and scanner insertion cannot pass yet. Use the actual matching CAD and measured clearances. |
| Leica source includes provisional joints and filling/reference slides | Those are not evidence of real powered mechanisms. Classify each joint as motorized, mechanically coupled, passive, fixed or reference-only before driving it. |

Measurements and source-derived model checks are reproducible in
`physics_readiness.json` and `audit_physics.py`.

## CAD changes to design next (not silently installed)

1. **Stainer drawer pulls for Nori's left hand.** Initial CAD/sim concepts are now added. Validate the pull and approach pose
   together, with finger clearance, mechanical attachment, full drawer travel and
   closed-door clearance. Model any latch separately. Validate pulling/pushing
   force, robot base stability and left-arm reach while the right hand carries a
   rack. A handle alone does not prove that a drawer can be opened.
2. **Secure rack transport interface.** Test the detachable handle under load.
   Consider a positively retained handle or a grasp around a structural rack rim.
   Preserve loading and coverslipper clearance. Do not obscure slides or sensors.
3. **Scanner/coverslipper carrier pickup features.** Their carriers need their own
   verified grip/approach frames or compatible removable handling adapters. Prove
   docking, insertion stops, capacity, clearance and adapter removal where needed.
4. **Preserve the existing lower slide grooves.** Reproduce the successful physical
   contact region and approach path, with force, clearance and contamination checks.
   The flat-pad idea is retired; do not redesign proven fingers to compensate for
   a collision-model error.
5. **Second Nori: defer.** Start with one Nori for rack/door work and LeHisto for
   individual slides. A second robot may help reach or throughput, but does not
   cure bad grips, incompatible carriers or missing mechanisms. Reassess after a
   single physical transfer and scheduling/collision analysis.

## Rule for passive objects and damage

Loose slides, racks, folders and containers get passive dynamics and support
contacts; they do not receive autonomous pose commands. Hinged/sliding doors and
drawers retain their real mechanical constraints and move only from applied
contact forces or a modeled real motor/linkage. A non-electronic item may of
course move when a powered conveyor or robot physically pushes it. A genuine
fixed/bolted fixture may remain fixed; a loose container must not be welded down
merely to make insertion pass.

The goal is **detect success or failure honestly**, not guarantee that a grasp
always succeeds. Missing evidence blocks the next action. A drop/slip/impact should
halt handling and flag the item for quarantine. No calibrated glass-fracture model
or experimentally supported break threshold is available; do not call an item
unbroken, or predict shattering, based on this bench.

MuJoCo contacts are soft constraints and ordinary mesh collision uses convex
representations. Accurate concave rack/bucket interiors need suitable decomposition
or primitive contact geometry. See the official
[computation documentation](https://mujoco.readthedocs.io/en/latest/computation/) and
[modeling guide](https://mujoco.readthedocs.io/en/latest/modeling.html).

## Verification and next milestone

Runtime: `C:/Users/Owner/Documents/Nori/.venv/Scripts/python.exe`.

Run `build_operational.py`, `groove_bench.py`, `test_groove_updates.py`,
`test_physics_readiness.py`, then `audit_physics.py`.
`contact_bench_validation.json` and `contact_*_trace.json` contain the measured
bench results; `contact_padded.png` shows the concept in contact with the slide.
Legacy workflow, robot-motion and machine-motion tests remain animation tests.

Before claiming an operational lab, implement and validate **one complete real
transfer**: choose the exact rack and matching container; verify handle retention;
calibrate both contact frames; solve a feasible joint trajectory; approach on
physical wheels; grasp under contact/effort limits; lift-test; carry; operate a
passive door if required; align and insert; verify support; release; withdraw.
Run missed-grasp, shifted-container, slip, obstruction, overload and loss-of-contact
cases. Only then extend that validated interaction to other stations.

Needed inputs: the matching 24-slide bucket CAD; verified Nori lift/arm/gripper
ratings and measured masses; and actual latch/drawer/carrier interface details.
