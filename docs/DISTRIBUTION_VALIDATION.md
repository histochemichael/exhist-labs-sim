# GitHub distribution validation — 2026-09-15

The distribution is a separate copy; it does not replace the author's working lab.

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
