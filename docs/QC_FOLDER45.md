# QC folder cradle: final 45-degree layout

This is a **fixed concept fixture**, added only to selectable `exhist_rail_loop_qc45.xml`. Existing base XMLs and the eleven-bath validation evidence are preserved. No hardware is operated.

## Geometry and Coordinates

The pinned source is [LeHisto commit d6da8e0](https://github.com/histochemichael/LeHisto/tree/d6da8e09789b35e5a1595b4da6a9ddd3871c7cfe), not a live reference to changing experiments. See [the layout manifest](../qc_folder45_layout.json) for dependency hashes.

- Source tabletop z=-0.0697 m maps to lab tabletop z=0.800 m by translation [2.364, 0.700, 0.8697] m.
- Folder center [0.026, -0.240, 0.075] m maps to [2.390, 0.460, 0.9447] m.
- Tray is 45 degrees above horizontal; covers are held 100 degrees behind it. All 20 CAD pockets remain present.
- Ten stand solids: backing, lower stop, two stop links, two stop arms, two posts and two feet. Fasteners, materials, strength and flap clips still need engineering.
- Three source-holder boxes are included, centered at lab [2.604, 0.600, 0.8824] m, yaw 67.38 degrees. This is a fixture concept, not a manufactured holder design.
- CAD STL uses millimeters with the tabletop at z=0. Standalone XML uses meters at that same tabletop datum.

## Handling Provision

Two 80 x 340 x 150 mm side-access volumes are reserved. They are hidden bookkeeping sites, not robot grippers or obstacles. The existing Nori left arm is the proposed handler; a future two-arm Nori remains an option, not an added or tested robot. The cradle supports the folder during LeHisto placement in the concept, so Nori need not hold it continuously.

The folder is fixed in this layout. Nori reach, grasping, loaded exchange, passive retention and flap closing have **not** been validated. Lab LeHisto geometry remains at its original held pose; alignment of its robot frame and swept motion still need work.

## Checks

[Ten static layout checks](../qc_folder45_validation.json) pass: model compilation, unchanged robot degrees of freedom and dynamics, bench support, fixture footprint, cover clearance and conservative neighbor/access checks. The folder clears the tabletop by 15.7 mm. The nearest held LeHisto has a conservative bounding-box gap of 15.3 mm including the new source holder. This small static gap is not a certified moving-robot clearance.

Thirteen regression tests (five QC, eight existing loop tests) cover the layout and preserved model. The integrity check strips only named QC additions and requires the remainder to equal the base room model. New QC elements are fixed, non-contact context; they do not establish in-lab contact dynamics.

Separately, the source LeHisto simulation passed one upper pocket (zero-based row 4, column 1) at nominal and half timesteps, with open-gripper and 3 mm misalignment controls rejected. [Source video, CAD and evidence](https://github.com/histochemichael/LeHisto/blob/d6da8e09789b35e5a1595b4da6a9ddd3871c7cfe/docs/FOLDER45.md). Those results do not prove Nori handling, all 20 pockets, or full-lab slide-placement success. Rigid contacts omit glass fracture and cardboard compliance.

## View and Refresh

```powershell
python qc_folder_cradle.py
python lab_rail_loop.py --qc45
```

In the live lab viewer, select station 6 for QC. The optional composition retains the existing special-stain controller; this update does not rerun or replace its full eleven-transfer evidence.

`python qc_folder_cradle.py --build --render` regenerates the composition, standalone XML, stand STL, report and images from the pinned snapshot. The source builder in `source/qc-folder45` is provenance only, not a runnable lab experiment.

- [Whole QC station](images/qc-folder45.png)
- [Folder detail](images/qc-folder45-detail.png)
- [Matching stand STEP](../cad/step/Folder-Stand-45-Concept.step)
- [Stand Fusion archive](../cad/Folder-Stand-45-Concept.f3d)
- [Stand assembly STL](../cad/stl/Folder-Stand-45-Assembly.stl)
- [Standalone inspection XML](../models/assets/qc_folder45.xml)

This fixed QC fixture is included in the public ExHist simulation snapshot; it remains distinct from the staged Nori promo and the separately tested LeHisto pocket experiment.
