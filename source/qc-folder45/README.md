# Pinned QC folder concept inputs

Read-only copies from the LeHisto task, `LeHisto/simulation`, captured 2026-09-16 from the coherent candidate with folder center `[0.026, -0.300, 0.090]` m and tabletop z `-0.0697` m. This is not necessarily the source task's latest candidate.

- `folder45.py`: source builder for provenance only; its imports are not bundled as a runnable experiment here.
- `folder_stand_45.json`: ten-part cradle specification in metres.
- `folder_slide_45.xml`: source experiment snapshot. Only the fixed folder CAD and stand geometry are imported by `qc_folder_cradle.py`; source robot paths/controller assumptions are not used.
- `slide_folder.json`: original 20-place folder CAD tessellation from `simulation/source/cad/slide_folder.json`.

SHA-256 dependencies are recorded in `../../qc_folder45_layout.json`. The exported stand STL files are generated directly from this pinned JSON, not copied from a changing source candidate. No matching native cradle STEP was available at snapshot time. Placement/contact success is not asserted. Do not run the source builder to mutate another task.
