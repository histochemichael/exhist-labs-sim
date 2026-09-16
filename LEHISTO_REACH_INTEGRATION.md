# Loaded-reach profile integration

`lehisto_loaded_reach.json` is consumed by special-stain placement and bath
targets. `lehisto_loaded_reach.py` provides per-rig FK shoulder/carriage frames,
radial and yaw screening, and fail-closed loaded-transfer admission. The tested
200–330 mm nominal bounds remain separate from the configurable 10 mm design
margin. Nori cannot use this profile. Collision, rack-bottom clearance,
payload, height, rail and grasp-frame evidence remain mandatory.

The actual standalone shoulder is z=0.027983 m, not the model origin z=0.
Thus its original jar-origin height relative to the shoulder is -0.092183 m.
The current lab row is -0.117143457 m: 24.960457 mm lower. This corrects the
initial audit's mistaken source-origin height. The source-to-lab rig rotation
uses the shoulder-based -90 degree mapping from `lab_row_test.py`; the actual
placed rig and rail transform is then applied, not a global hardcoded origin.

Five profile tests pass, including moved rig and carriage frames, margin versus
tested bounds, different payload/height/rail rejection, Nori rejection, and
required collision/clearance/grasp-frame evidence. `lehisto_lab_reach_audit.json`
and `lehisto_loaded_reach_audit.png` show the current placements. Active jars
4 and 5 are only 204.80 and 201.33 mm from the parked shoulder; spare jars 9–11
exceed 330 mm. The current row and its choreography were not relocated or
promoted to validated loaded motion by this integration.

The separate Physical Training task reported a successful proposed 30 mm
forward row shift for 4→5→6→7→8 using upright-rack dynamics, and is separately
working on all eleven baths. Its source and experiment files are untouched here.
Those experiments do not validate this lab's old tilted-tool choreography,
surrounding machinery, Nori interactions or all eleven original row positions.

The attempted cross-task report was blocked by the app's sharing approval
check. The findings remain available locally; no blocked message was bypassed.
