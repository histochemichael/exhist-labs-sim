# QC folder cradle: layout integration

This is a **fixed concept fixture**, not a physically validated folder transfer. It is added only to the selectable `exhist_rail_loop_qc45.xml` composition. The existing three main XMLs and eleven-bath validation evidence are preserved.

## Geometry and coordinates

The snapshot comes from the LeHisto 45-degree experiment. Its pinned JSON, geometry XML, CAD tessellation and builder are in `source/qc-folder45/`; the builder is provenance, not a standalone lab entry point. Read [the layout manifest](../qc_folder45_layout.json) for current coordinates and SHA-256 dependencies.

- The source tabletop is z = -0.0697 m; the lab QC tabletop is z = 0.800 m.
- The source-to-lab translation is [2.364, 0.700, 0.8697] m, without rotation.
- The current pinned candidate has source folder center [0.026, -0.300, 0.090] m; the lab center is [2.390, 0.400, 0.9597] m. Later source experiments may differ: this is a coherent pinned candidate, not a live reference to changing files.
- The folder tray is 45 degrees above horizontal, facing the QC LeHisto. Both flaps are held 90 degrees behind it. All 20 CAD pockets are retained.
- The stand has ten solid primitives: backing, lower stop, two stop links, two stop arms, two posts and two feet. Fasteners, strength, materials and manufacturing tolerances are unspecified.
- STL exports are millimetres with the tabletop at z=0; the standalone asset XML uses the same origin in metres. The placed lab scene uses the lab transform above.

## Handling provision

Two reserved side volumes are 80 x 340 x 150 mm. They are bookkeeping/visual sites, hidden in normal rendering, not obstacles or gripper models. The existing Nori left arm is the proposed folder handler; no extra Nori or replacement gripper was added. The cradle is intended to support the folder while LeHisto places slides. Loading/unloading into it, keeping it seated, and closing/removing a full folder require separate contact-driven tests.

The source LeHisto task is still tuning its 45-degree pose/controller. No source control script is executed by this integration. The lab LeHisto geometry remains at its original held pose; matching reach and robot-frame calibration are not established by a collision-free layout.

## Checks completed

See [qc_folder45_validation.json](../qc_folder45_validation.json). The current snapshot compiles; the feet meet the tabletop, the fixture footprint stays inside the bench, and the folded-back covers clear the table by approximately 17.1 mm. Conservative bounding-box separation from the nearest held LeHisto is approximately 157.4 mm. The blocks and scanner remain untouched. Both reserved side volumes are empty at the held pose.

The integrity check removes only the named QC additions and requires the remaining XML to equal the original room model. Degrees of freedom, actuator settings, joint parameters and equality constraints are unchanged. New QC geometry is fixed and non-contact in this visual-context composition; it is not silently welded to a hand. A 0.080 s live special-stain-controller initialization passed; the full eleven transfers were not rerun for this optional composition. Thirteen regression tests (five QC plus eight existing loop tests) pass.

These are **static layout/startup checks**, not swept robot clearance, Nori IK, passive docking, support/contact physics, glass breakage, slide placement or hardware validation. The new 45-degree placement controller must not be labelled passed based on them.

## Refresh and files

`python qc_folder_cradle.py --build --render` regenerates the optional composition, standalone XML, eleven stand STL files, scoped report and images **from the pinned local snapshot**, without consulting the other task or changing the validated base XML. A deliberate source refresh must first copy a coherent XML/JSON/CAD set and then rerun all checks.

The native cradle STEP in the other task was still an earlier candidate at this integration point. It is deliberately not bundled as matching the newer STL/XML. The original folder STEP is already supplied as [slide-folder-20.stp](../cad/step/slide-folder-20.stp). A matching cradle STEP can be added after the source candidate is finalized.

- [Whole QC station image](images/qc-folder45.png)
- [Robot-side folder detail](images/qc-folder45-detail.png)
- [Stand STL](../cad/stl/Folder-Stand-45-Assembly.stl)
- [Folder/cradle inspection XML](../models/assets/qc_folder45.xml)

No physical hardware was operated. No Nori handling result or full-lab completion is claimed.
