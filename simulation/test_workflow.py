import json
from pathlib import Path
from workflow import Lab
ROOT=Path(__file__).resolve().parent
stations=json.loads((ROOT/"layout.json").read_text())["stations"]
results={}
for label,fault in [("normal",False),("scanner_fault_recovery",True)]:
    lab=Lab(stations);lab.inject_scan_fault=fault;fault_seen=False;fault_start=None;parallel=False
    for _ in range(20000):
        if lab.faults:
            fault_seen=True
            if fault_start is None:
                fault_start=lab.time
                frozen=lab.time;lab.paused=True;lab.tick(10);assert lab.time==frozen;lab.paused=False
            if lab.time-fault_start>=15:lab.recover()
        lab.tick(.1)
        parallel|=sum(j.state=="PROCESSING" for j in lab.jobs)>1
        if lab.completed==36:break
    assert lab.completed==36
    assert all(j.state=="DONE" and j.carrier=="slide_folder" for j in lab.jobs)
    assert all(j.trace==[s.name for s in j.route] for j in lab.jobs)
    assert parallel
    assert fault_seen==fault
    picks=[e for e in lab.events if e["action"]=="custody_pickup"]
    drops=[e for e in lab.events if e["action"]=="custody_dropoff"]
    assert len(picks)==len(drops)==21,(len(picks),len(drops))
    assert len({sid for e in lab.events if e["action"]=="batch_completed" for sid in e["slide_ids"]})==36
    (ROOT/f"workflow_{label}.json").write_text(json.dumps(lab.report(),indent=2))
    results[label]={"passed":True,"slides":36,"transfers":len(picks),"parallel_processing":parallel,"seconds":round(lab.time,1)}
(ROOT/"workflow_validation.json").write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))

