# GitHub distribution validation — 2026-09-15

The distribution is a separate copy; it does not replace the author's working lab.

## Public snapshot update — 2026-09-17 UTC

The current package adds the upright-rack 60-second promo and final pinned QC fixture assets. Fresh checks on this copy passed: all three base models compiled and the live controller initialized; all 13 existing loop/QC regressions passed; the promo's 1,800-frame CAD-solid audit reported zero unintended intersections in scope; postflight confirmed maximum rack tilt 0.0174 degrees, minimum container clearance 0.612 mm, closed oven and correct rear-vessel seating. The full 60-second MP4 was re-rendered here, decoded to 1,800 frames and checked against its source hashes and ending logo. This does not add a full-lab physical success claim.

All 430 promo resources match the author's bytes, and only XML resource paths were relocated. Original reports are retained separately under promo_v4/author-evidence. [Current promo guide](PROMO.md) and [portability record](promo-portability.json).

The final 45-degree QC concept now includes matching native Fusion/STEP alongside STL/XML; earlier snapshot notes below are historical. The taller promo oven has STL/XML only, explicitly distinct from the original Quincy STEP. Common token/private-key patterns, oversized Git files and documentation links were checked without printing candidate secrets; see [publication check](publication-check.json).

## Optional QC layout addition — 2026-09-16

The selectable `exhist_rail_loop_qc45.xml` adds a pinned, fixed 45-degree folder cradle. The base model and evidence below remain byte-for-byte unchanged. Five additional QC tests and all eight existing loop tests pass. The optional composition compiled and initialized the live special-stain controller for 0.080 s; its eleven-transfer route was not rerun. See [QC_FOLDER45.md](QC_FOLDER45.md) for static clearances, source-candidate status, and explicitly unvalidated Nori/slide-placement handling. The new cradle's STL/XML do not imply a matching finalized native STEP yet.

## Fresh packaged contact route

A fresh live run using Python 3.10 / MuJoCo 3.4.0 / PlaCo 0.9.20 completed **11/11** transfers in 313.249 simulated seconds, then kept physics running for a continuous two-second released-rack hold.

- Minimum clearance above the rail: **50.974 mm**.
- Minimum transfer/rim clearance: **41.068 mm**.
- Maximum relative grasp slip: **1.485 mm**.
- Rail contact: **0 N**; joint-limit violations: **none**.
- Released-rack hold position span: **9.146 × 10⁻⁹ m**.
- Portable model SHA-256: `86539e0e41ca4219cbf6ef47d76f7f8d4e57048b6b66da9d31b32428eba2994d`.
- Trace SHA-256: `8a9c1a2ae9214b6d1f8b0cd1ea0221f3de308f91c2fd288cfc24bf8f1881d08d`.

This is a new run, not the author's earlier restored-final-state report. Machine/room context stays static; it is not a whole-lab workflow or loose-slide retention test.

## Regression and portability checks

- **8/8** `test_lab_rail_loop` integration guards passed against the fresh report.
- **12/12** equipment-access failure guards, Nori posture and machine-glazing tests passed in the separate MuJoCo 3.12.0 environment. The nominal six full access cycles were not rerun as part of this packaging pass.
- Main XMLs and controller startup compile/run in the portable copy.
- A clean local Git clone of commit `091cfd7` restored all three compressed files, compiled all three main models and initialized the live controller successfully. No author-specific mesh path was needed. This checks clone portability, not a fresh operating-system dependency installation.
- All non-path XML parameters, referenced resource bytes and 420 numerical model arrays per scene are compared with the author copy. Resource path buffers and derived convex-hull representations are excluded from bitwise array comparisons and recorded explicitly.
- CAD/trace gzip archives are decompressed and SHA-256 checked.
- README image/file links are checked locally; actual rendered station/asset images are visually reviewed.
- Common token/private-key patterns were scanned without printing candidate secrets. No such matches were found; this is not a comprehensive security audit.

See [portability-validation.json](portability-validation.json), [asset-file-manifest.json](asset-file-manifest.json), [asset-catalog.json](asset-catalog.json), [room-asset-catalog.json](room-asset-catalog.json), and the root `rail_loop_lab_validation.json`.

The original historical video/review reports in the snapshot may refer to earlier hashes. They are provenance, not evidence of the new packaged run or a complete lab. Use the current report and documented test entry points.
