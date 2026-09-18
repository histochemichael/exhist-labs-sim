# Repository layout

The top level is a short entry point rather than hundreds of development files.

```text
README.md              Start here: downloads, gallery, installation
launch.ps1             Launch from the repository root
docs/                  Repository navigation and organization checks
simulation/            Self-contained simulation and historical evidence
  README.md            Full illustrated station guide and commands
  cad/                 STEP/STP, STL and native design assets
  models/assets/       Standalone static inspection XML models
  assets/              Runtime meshes and textures
  source/              Pinned source models and component provenance
  licenses/            Upstream license notices
  docs/                Station images, asset catalog, validation guides
  promo_v5/            Latest MP4, looping GIF, portable scene/playback and evidence
    background-assets/ Recorded folder-rig meshes
    evidence/          Author provenance and packaged replay verification
    images/            A few selected review frames
  promo_v4/            Historical promo, playback and scoped evidence
  rail_loop_reference/ Pinned LeHisto loop implementation and data
  tools/               Installation, unpacking and publication utilities
```

The loose files formerly at the root are now inside `simulation/`. Its Python modules, scene XMLs and historical output files intentionally retain their relative positions in this first organization pass: many experiments load neighboring traces and models by name. This avoids rewriting tested simulation behavior merely to change GitHub's landing page.

## Existing command migration

Run existing guide commands **from `simulation/`**, or use the new root launcher. All links within the full illustrated guide remain relative to that bundle. No CAD geometry, trace, model or video was regenerated for this move.

For example:

```powershell
cd simulation
python tools/unpack_assets.py
python -m unittest test_lab_rail_loop test_qc_folder_cradle -v
```

The root `.gitignore` excludes restored large assets at their new locations. Tracked gzip archives and their SHA-256 manifest remain inside the simulation bundle. Use `simulation/tools/unpack_assets.py` to restore them, not Git LFS.

## Organization checks (2026-09-17)

- Before documentation updates, all 1,977 relocated tracked files matched the Git blob hashes from `df662f0` exactly. No simulation code, CAD, XML, image, video or trace changed.
- All four compressed assets restored/verified against their existing SHA-256 manifest.
- Three main MuJoCo scenes compiled, and the live LeHisto controller initialized successfully from the new location.
- All 13 existing loop/QC regression tests passed.
- New navigation links and existing publication-guide links resolved; common credential-pattern and oversized-file checks passed. This is not a comprehensive security audit.
- The existing neutral-pose adjacent-link warnings remain; this organization pass does not claim new physical validation or rerun the full lab workflow.
