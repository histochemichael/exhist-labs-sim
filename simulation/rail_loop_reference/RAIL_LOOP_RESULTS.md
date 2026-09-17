# Moving-Rail Closed Loop

PASS: eleven loaded transfers, visiting all eleven containers and returning the rack to container 1. The carriage traveled from -60.00 to 210.00 mm, ending within 0.015 mm of its start.

## Scope of the Fix

Unlike the previous parked-carriage demonstration, the linear actuator is now commanded throughout transfers along both sides. It uses the full EXISTING MODELED stroke of 270 mm (-60 to +210 mm). The physical rail mesh is about 600 mm long; this does not prove that the carriage can use that whole extrusion. Joint/control/effort limits are unchanged. The source URDF limits rail speed to 0.1 m/s; this planner requests at most 0.02 m/s.

The far-side transition passes above the rail at a verified height; the near-end transition goes around the end. This is a closed operational route, NOT a continuous 360-degree perimeter orbit around both physical rail ends. A literal full-extrusion traversal requires verified hardware stroke and a new reach/clearance check. No physical robot or actuator was operated.

## Results

- Measured peak carriage speed: 19.63 mm/s.
- Minimum rack-bottom clearance above rail during overflight: 50.97 mm.
- Minimum rack-bottom clearance above jar rims during transit: 41.07 mm.
- Maximum seated lateral error: 0.522 mm.
- Maximum relative grasp slip: 1.771 mm.
- Rack/rail contact: 0.000 N.
- Joint-limit violations: [].
- Simulated duration: 313.25 s.

Each transfer verifies grasp settling, a short lift probe, clearance, destination support, release and retreat. The rack remains a free body with no attachment or per-hop reset. The added rail collision box conservatively covers the full rail-mesh bounds and interacts with rack geometry. A separate negative-control test puts the rack into that box to confirm collision detection.

The eleven containers remain on the same bench with unchanged dimensions, 145 g rack payload, 25.4 mm risers, and +/-1.5 Nm arm torque caps. Original parked-carriage tests remain intact. The room and adjacent machines in the video are visual context, not collision-certified. This is a nominal dry reference-control simulation, not a trained policy, hardware reliability study or loose-slide/fluid validation.

## Reproduce

Run with the lerobot Python environment from this directory, in order:

    python rail_loop.py --screen
    python rail_loop.py
    python -m unittest test_rail_loop -v
    python render_lab_row.py rail_loop

Files: rail_loop_layout.json has all lab-world locations and local frame orientations. rail_loop_controller.py is the generated, auditable isolated controller. rail_loop_trace.json contains the measured state history. rail_loop_results.png shows the measured path and carriage travel. rail_loop_review.mp4 shows the full route at 4x speed, with both cameras and measured carriage position.
