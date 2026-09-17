# ExHist promo v4 — upright loaded-rack handling

The 60-second cut replaces v3's tilted tool pose and unconstrained loaded-arm transitions.

- Nori now uses the straight LeHisto-style grasp frame: zero intentional tool tilt.
- Loaded-arm moves follow Cartesian paths with an upright rack orientation constraint. Chassis yaw may rotate the rack horizontally, but may not pitch or roll it.
- Each simulated oven has 120 mm additional upper clearance. The shell, liner, upper door and hinge geometry are extended; shelf, controls and pull heights are unchanged. This is a modified concept envelope, not a claim about stock Quincy dimensions or a validated oven redesign.
- The previous single black oven pulls, oven closure, drawer handling, restricted jaw release, stainer pickup/rear-vessel placement and logo ending are retained.

Run from the parent directory:

```powershell
python promo_solid_audit.py 1
python promo_postflight.py
python promo_demo.py --render --sixty
python promo_video_verify.py
```

For this public package, follow [the portable demo guide](../docs/PROMO.md). Run tools/unpack_assets.py first. Rendering uses exhist-sim; geometric checks use the separate requirements-promo-checks.txt environment. Replanning is experimental, not the supported playback entry point.

Validation reports check all 1,800 video frames for rack tilt, groove alignment, container clearance, closure/placement and scoped CAD-solid intersections. The solid check covers the oven/stainer interaction corridor, including cross-arm and rack/head/left-arm checks. Intentional claw/handle and fork/rack contacts are excluded; non-manifold robot meshes use conservative convex hulls. Reported intersections exceed 0.02 cubic millimetres. Video verification binds the scene/state hashes to the decoded 60-second MP4.

This is staged kinematic playback. Upright orientation is not proof of slide retention under real acceleration, slip, vibration or gripping forces. Dynamic/hardware testing and continuous-time certification remain separate. Base lab assets and previous evidence are preserved.
