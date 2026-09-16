# CAD / asset provenance and units

## What is included

- Native STEP/STP assembly exports from the open Fusion designs: **LeHisto v38**, **Quincy 10GC - Bottom Shelf - Swing Door v1**, **Cardboard Slide Folder - 20 Place v1**, **Slide Rack v13**, **Leica Staining Rack v6**, and **Leica_ST5020_CV5030_Workstation_Mechanical_QC_v2 v2**.
- Existing STEP exports of the branded NanoZoomer S60, embedded cassette block, CV5030 30-position output magazine, drawer-pull v02 concept and Quincy pull concept.
- A component STEP export of **cassette_0 | Bottom cassette | DEPTH PROVISIONAL** from the open **NanoZoomer S60 v0** design.
- Exact simulation tessellations in JSON/OBJ, original robot STL meshes and their XML/URDF definitions.
- Consolidated STL exports of the twelve catalogued CAD-derived families, plus representative procedural room assets.

The handled Leica rack and scanner cassette exports include the current **unsaved in-memory Fusion state**. The export did not save, modify, replace or close the user's Fusion documents. Other exported named designs were unmodified at export time. A STEP file represents the assembly geometry at export, not a Fusion history/joint archive. The simulation's JSON tessellations are separate pinned inputs; do not assume a later CAD export proves every mesh version is identical.

## Units

| Files | Units / interpretation |
| --- | --- |
| `cad/step/*.stp`, `*.step` | Unit-aware native CAD exports; read the STEP unit definition. |
| `cad/stl/*.stl` | **Millimetres**, static triangulated visualization exports. |
| `models/assets/*.xml` | **Metres**, static inspection assets; furniture exported about a local center. |
| `assets/*.obj`, source tessellation JSON | Metres as used by the simulation. |
| Original robot STL meshes | Preserve original units; XML/URDF includes the corresponding scale. |

The standalone CAD gallery keeps native assembly coordinates except the S60/cassette axis conversion used for upright inspection. Thus it may not share the translated/rotated origin of a placed lab instance. Jointed, colored, collision-aware scene definitions remain in the main XMLs, not the flattened STL exports.

## Large assets

`leica-workstation.stp` is larger than GitHub's normal 100 MiB single-file limit. It and the large recorded traces are losslessly gzip-compressed in Git. Run:

```sh
python tools/unpack_assets.py
```

The decompressed hashes and sizes are in [compressed-assets.json](../compressed-assets.json). The unpacker refuses to overwrite modified files. You can also use a gzip-compatible archive utility. Git LFS is not required. See [GitHub's file-size documentation](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github).

## Assets with no native STEP source

Nori's supplied package explicitly contains **no manufacturing CAD**: its STL meshes are simplified visualization models. We have not invented Nori STEP geometry or presented a mesh conversion as original CAD.

Benches, shelves, cabinets, chairs, PCs, architectural doors/windows, canopies, rails/markings, signs, glass-slide/coverslip primitives and support/collision fixtures are also represented procedurally or as simulation geometry. Their authoritative dimensions/transforms are in the Python builders and XMLs. Representative room assets have STL/XML exports and images; their lack of native STEP is explicit. The one-inch riser additionally includes an OpenSCAD source. Glass inside the validated rack loop is a fixed visual representation, not loose glass retention proof.

No new gripper, scanner adapter or manufacturing-ready design is implied by exporting existing assets.

## Validation boundary

The portable package rewrites only XML asset locations and author-specific source-model constants needed by supported launch paths. Source meshes, contact parameters, force limits, joint limits and controller logic are retained. The pinned reference manifest distinguishes author hashes from relocated XML hashes. See [distribution-provenance.json](distribution-provenance.json) and [portability-validation.json](portability-validation.json).

Older authoring/export/rebuild scripts and historical reports are retained for provenance. Some retain author-local paths, and are not supported clone-and-run entry points. They do not run during the documented launch sequence. An original report is not silently relabeled as a new test; the distribution's fresh loop run has its own current model hash.

Read [LICENSE-NOTICE.md](../LICENSE-NOTICE.md) before redistribution or reuse.

## Optional 45-degree QC folder cradle

The new pinned fixture includes ten component STLs plus an assembly STL in `cad/stl/Folder-Stand-45-*.stl`, a metre-scale standalone XML, and a separate whole-lab composition. It does not modify the previously validated base model. Native cradle STEP availability and the explicit handling/validation limits are documented in [QC_FOLDER45.md](QC_FOLDER45.md). The source experiment is evolving; only the local pinned geometry is used at runtime.
