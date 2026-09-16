# ExHist operational workflow demo

**Current default: PHYSICS READINESS HOLD.** Run launch.ps1 for stationary inspection. Full physical operation is blocked pending contact, actuator, carrier and clearance work. See PHYSICS_READINESS.md for the current audit and separate contact bench.

The remainder of this document describes the **legacy kinematic animation only**. Run `launch.ps1 -Preview` (or `operate_lab.py --preview`) to opt in. It starts at 2x demonstration speed; its completion counts are not physical successes.

Controls: Space pauses; R resets the batch run; F fails the next completed scan; C recovers faulted stations and retries the scan; 1–8 focuses a table; O shows the entire lab; T follows Nori; + / - changes speed.

Four batches (36 uniquely tracked slides) follow sorting, baking, routine or special staining, coverslipping, curing, QC, scanning and packaging. Equipment slots are reserved before transport; a single Nori transport queue retains batch custody during transfers. Coverslipping and curing stay on the same routine workstation. Carrier changes preserve slide IDs. The QC step here is a slide-processing workflow placeholder, not automated block-image assessment.

The normal and injected-scanner-fault tests both complete 36 slides with 21 paired pickup/dropoff events. The MuJoCo model also compiles and completes a 3,390-frame kinematic check. See workflow_validation.json and operational_validation.json. Live runs save their events and slide records to workflow_live.json on completion or viewer close.

## Stainer and coverslipper articulation

Both Leica workstation copies include all 84 source CAD joints: 10 rigid relationships and 74 moving joint assemblies per copy. Seven planar joints each expand into three coordinates, including a locked rotation, giving 88 imported coordinates per workstation. CAD poses, axes and limits are retained in assets/leica_kinematics.json; the complete source-to-simulation map is in machine_articulation.json. Hidden optional/reference joints remain present without reintroducing their hidden visual proxies.

Staining drives ST5020 motion trials; coverslipping drives CV5030 motion trials. Independent machine labels and activity lights indicate RUNNING, IDLE, CURING, PAUSED or FAULT based on the simulation scheduler—not equipment telemetry. Doors, hoods, drawers, filling references and transfer-reference joints are available for inspection but do not all cycle automatically. Small periodic strokes are illustrative, not manufacturer operating sequences.

Press X for transparent-enclosure inspection. Press J to pause and enter the joint inspector; [ and ] select a coordinate, G makes a small jog, and H restores its exported pose. Space leaves inspection and resumes the workflow. Locked coordinates are identified and do not move.

test_machine_motion.py checks all 162 unlocked imported coordinates across both copies, 14 locked coordinates, source coverage, joint limits, pause/fault freeze and completion of the 36-slide workflow. It does not validate internal collisions, rack insertion, coverglass transfer, processing quality or real hardware. Placeholder inertias are unsuitable for dynamic verification.

## Boundaries

The lab now includes a provisional 18.4 x 8.9 m furnished room, shown as a three-sided cutaway: four observation windows, two closed doors, four tall storage cabinets plus under-bench drawers, five shelving bays, four additional work tables and task chairs, and two additional PCs. Processing-zone signs, lights and floor markings identify the bays. New furnishings remain outside the Nori aisle; test_room.py checks the complete workflow against their collision geometry. O opens the full-room overview; 1–8 still focus the equipment stations. Furniture is static, and this is not an ergonomic or building-code validation.

Two conceptual suspended canopy hoods sit over the ST5020 stainers, with tapered housings, 200 mm duct stubs and hanger rods to a provisional 3 m ceiling datum. Each is approximately 1.36 x 1.05 m with its lowest lip at 1.88 m, 0.49 m above the static CAD machine envelope. These are geometric placeholders, not engineered ventilation or structural designs; capture performance and all-pose clearance remain unvalidated. canopy_hoods.py preserves them during scene rebuilds.

This is a workflow simulation with robot-motion trials, NOT an operational physical lab or a validated robotics controller. Equipment processing is logical; physical rack insertion, individual-slide loading and grasp/contact physics are not implemented. Carriers still move independently through visual handoff poses, not solved robot grasps. Cycle times, travel speed and table dimensions are demonstration assumptions, not throughput or safety validation. Scanner cassette and output-magazine conversions are logical, and scanner carrier capacity compatibility still needs validation.

## Articulated robots

Leica racks use the corrected detachable handle exported from the live Leica Staining Rack v6 CAD: 100.51 mm handle top, copied 24-slide-rack grip, 28 slide positions loaded with 15–16 reserved for the handle. The shared carrier asset and eight rack locations inside each Leica workstation include it; empty embedded racks remain empty. Original source JSON exports are retained. Handle retention, detachment forces and coverslipper compatibility remain unvalidated. See handled_rack_validation.json for geometric alignment checks.

All five LeHistos use source Fusion v38 as-built joints and rigid groups: 532 CAD parts each, with no fallback part assignments. Ten joints per mechanism cover the carriage, lead screw, five arm joints, two jaws and pinion. They execute local position-IK motion trials during assigned station work and hold when idle or paused. These are not yet rack pick/place trajectories. Slide coordinates are converted from CAD centimeters to meters; revolute limits remain radians.

Nori is sourced from the saved "Nori with parallel histo gripper.xml" model. The right wrist has the parallel histology gripper; the original left gripper is retained. Nori executes a right-arm carry/reach trajectory and an approach/retreat at handoff phases. The provisional docking offset is 0.95 m from the aisle pose. Both parallel jaws translate together with the source rack/pinion coupling and actuator target preserved. The left arm remains at its imported pose. Position-only IK reaches the trial target within 1 mm, but tool orientation and grasp contact have not been solved. The tool point is provisional in the parallel-gripper mounting frame.

test_robot_motion.py verifies all five LeHistos and Nori move, joint limits remain respected, all 36 slides finish logically, and no greater-than-1-mm table/machine penetration is detected through the sampled workflow using Nori's existing coarse colliders. It does NOT validate self-collision, continuous-time clearance, rack contacts or LeHisto collision geometry. Results are in robot_motion_validation.json; source mappings are in articulation.json.

Special-stain buckets are not fabricated stand-ins in this viewer: six purple targets mark the pending matching 24-slide-rack bucket CAD. The previous static layout remains separately accessible through run_lab.py and still contains its older bucket placeholders. The generic embedded cassette CAD is nominal and needs comparison to the actual cassette used.

## Rebuild and tests

Use C:/Users/Owner/Documents/Nori/.venv/Scripts/python.exe:

1. build_scene.py
2. build_operational.py
3. test_workflow.py
4. operate_lab.py --check-render
5. operate_lab.py

Next implementation layer: align actual rack handles with calibrated tool frames, add LeHisto and rack contact geometry, enforce orientation and self-collision constraints, couple payload custody to successful physical grasps, and validate instrument loading. Hardware adapters require explicit interlocks. Do not connect this demo directly to real equipment.
