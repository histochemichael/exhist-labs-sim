# Pinned Final QC Inputs

Copies from LeHisto commit d6da8e09789b35e5a1595b4da6a9ddd3871c7cfe (2026-09-16). Final folder center [0.026, -0.240, 0.075] m, 45-degree tray, covers held 100 degrees behind. Source holder [0.240, -0.100, 0.0127] m at 67.38-degree yaw.

`folder45.py` is builder provenance only; imports are not packaged to run here. `folder_slide_45.xml` contributes only the fixed folder, stand and source-holder geometry. `folder_stand_45.json` defines the ten-body stand. `slide_folder.json` is the original CAD tessellation.

Matching stand STEP, Fusion archive and millimeter STL are in `../../cad`. Hashes are recorded in `../../qc_folder45_layout.json`. LeHisto's standalone one-pocket dynamics passed; this lab integration is static context only, not Nori or lab slide-placement validation. Do not run the source builder to mutate another task.
