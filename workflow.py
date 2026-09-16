"""Deterministic lab workflow, resource reservations and custody tracking.
Times are demonstration values, not calibrated instrument cycle times.
"""
from dataclasses import dataclass, field, asdict
import json

CAPACITY={"sorting":2,"baking":3,"routine_a":1,"routine_b":1,"special":1,
          "sendout":2,"imaging_a":1,"imaging_b":1}
TIMES={"sort":6,"bake":18,"stain":24,"special_stain":24,"coverslip":12,
       "cure":12,"qc":8,"scan":20,"package":8}
@dataclass
class Step:
    name:str
    stations:tuple
    seconds:float
@dataclass
class Job:
    id:str
    protocol:str
    slides:list
    route:list
    index:int=0
    state:str="WAITING"
    location:str="sorting"
    slot:int=0
    holds:str|None=None
    reserved:str|None=None
    carrier:str="leica_rack"
    remaining:float=0
    trace:list=field(default_factory=list)
    scan_attempts:int=0
    folder_loaded:int=0
    folder_closed:bool=False

class Lab:
    def __init__(self,stations):
        self.capacity=CAPACITY.copy()
        self.x={s["name"]:s["x"] for s in stations}
        self.time=0.;self.paused=False;self.faults=set();self.events=[]
        self.occupancy={s:{} for s in CAPACITY};self.transport=None
        self.robot_x=self.x["sorting"];self.completed=0;self.jobs=[]
        for i,(protocol,count) in enumerate([("HE",8),("SPECIAL",6),("HE",12),("HE",10)]):
            routine=("routine_a","routine_b")
            route=[Step("sort",("sorting",),TIMES["sort"]),Step("bake",("baking",),TIMES["bake"]),
              Step("special_stain" if protocol=="SPECIAL" else "stain",
                   ("special",) if protocol=="SPECIAL" else routine,
                   TIMES["special_stain" if protocol=="SPECIAL" else "stain"]),
              Step("coverslip",routine,TIMES["coverslip"]),Step("cure",routine,TIMES["cure"]),
              Step("qc",("sendout",),TIMES["qc"]),Step("scan",("imaging_a","imaging_b"),TIMES["scan"]),
              Step("package",("sendout",),TIMES["package"])]
            self.jobs.append(Job(f"B{i+1:02d}",protocol,[f"B{i+1:02d}-S{k+1:02d}" for k in range(count)],route,
                                 slot=i,carrier="rack24" if protocol=="SPECIAL" else "leica_rack"))
        self.slide_records={s:{"batch":j.id,"protocol":j.protocol,"stage":"queued","location":"sorting"} for j in self.jobs for s in j.slides}
        self.inject_scan_fault=False
    def event(self,j,action,**kw):
        self.events.append(dict(t=round(self.time,3),batch=j.id if j else None,action=action,**kw))
    def fault(self,station):
        self.faults.add(station);self.event(None,"station_fault",station=station)
    def recover(self):
        for station in sorted(self.faults):self.event(None,"station_recovered",station=station)
        self.faults.clear()
        for j in self.jobs:
            if j.state=="FAULT":
                j.state="PROCESSING";j.remaining=TIMES["scan"];j.scan_attempts+=1
                self.event(j,"scan_retry")
    def reserve(self,j,station):
        used=set(self.occupancy[station])
        slot=next((k for k in range(self.capacity[station]) if k not in used),None)
        if slot is None:return None
        self.occupancy[station][slot]=j.id
        return slot
    def release(self,j,station):
        if station:
            for slot,owner in list(self.occupancy[station].items()):
                if owner==j.id:del self.occupancy[station][slot]
    def start(self,j,station,slot):
        j.state="PROCESSING";j.location=station;j.holds=station;j.reserved=None;j.slot=slot
        j.remaining=j.route[j.index].seconds
        if j.route[j.index].name=="scan":j.scan_attempts+=1
        self.event(j,"step_started",step=j.route[j.index].name,station=station,carrier=j.carrier)
    def finish(self,j):
        step=j.route[j.index].name
        if step=="scan" and self.inject_scan_fault:
            self.inject_scan_fault=False;j.state="FAULT";self.fault(j.holds)
            self.event(j,"scan_failed",reason="injected demonstration fault");return
        j.trace.append(step)
        old=j.carrier
        if step=="coverslip":j.carrier="output_magazine"
        if step=="qc":j.carrier="scanner_cassette"
        if step=="package":j.carrier="slide_folder"
        if old!=j.carrier:self.event(j,"carrier_conversion",source=old,target=j.carrier,slide_ids=j.slides.copy())
        for sid in j.slides:self.slide_records[sid].update(stage=step,location=j.location)
        self.event(j,"step_completed",step=step)
        j.index+=1
        if j.index==len(j.route):
            j.state="DONE";self.completed+=len(j.slides);self.release(j,j.holds);j.holds=None
            self.event(j,"batch_completed",slide_ids=j.slides.copy())
        else:j.state="READY"
    def tick(self,dt):
        if self.paused:return
        self.time+=dt
        if self.transport:
            tr=self.transport;j=next(j for j in self.jobs if j.id==tr["job"])
            # A destination fault pauses the transfer with reservation and custody retained.
            if tr["target"] not in self.faults:
                tr["elapsed"]+=dt
                if tr["elapsed"]>=tr.get('pickup_time',tr["approach"]+2) and not tr["picked"]:
                    tr["picked"]=True;self.release(j,j.holds);j.holds=None;j.location="nori"
                    for sid in j.slides:self.slide_records[sid]["location"]="nori"
                    self.event(j,"custody_pickup",carrier=j.carrier)
                if tr["elapsed"]>=tr["total"]:
                    self.robot_x=self.x[tr["target"]]
                    self.start(j,tr["target"],tr["slot"])
                    for sid in j.slides:self.slide_records[sid]["location"]=j.location
                    self.event(j,"custody_dropoff",station=j.location)
                    self.transport=None
        for j in self.jobs:
            if j.state=="PROCESSING" and j.holds not in self.faults:
                j.remaining-=dt
                if j.remaining<=0:self.finish(j)
        for j in self.jobs:
            if j.state not in ["WAITING","READY"]:continue
            step=j.route[j.index]
            candidates=step.stations
            # Keep coverslipping and curing on the same reserved workstation.
            if step.name in ["coverslip","cure"] and j.holds in ["routine_a","routine_b"]:
                candidates=(j.holds,)
            if j.holds in candidates and j.holds not in self.faults:
                self.start(j,j.holds,j.slot);continue
            dest=next((s for s in candidates if s not in self.faults and len(self.occupancy[s])<self.capacity[s]),None)
            if not dest:continue
            if j.index==0:
                slot=self.reserve(j,dest);self.start(j,dest,slot);continue
            if self.transport or self.robot_busy:continue
            slot=self.reserve(j,dest);j.reserved=dest;j.state="TRANSPORT"
            approach=abs(self.robot_x-self.x[j.location])/0.65
            travel=abs(self.x[j.location]-self.x[dest])/0.65
            self.transport=dict(job=j.id,source=j.location,source_slot=j.slot,target=dest,slot=slot,
                                robot_start=self.robot_x,approach=approach,travel=travel,
                                total=approach+travel+4,elapsed=0.,picked=False)
            self.event(j,"transport_reserved",source=j.location,target=dest)
        self.assert_invariants()
    def assert_invariants(self):
        ids=[s for j in self.jobs for s in j.slides]
        assert len(ids)==len(set(ids))==len(self.slide_records)==36
        for station,slots in self.occupancy.items():
            assert len(slots)<=self.capacity[station]
            assert len(set(slots.values()))==len(slots)
        assert self.completed==sum(len(j.slides) for j in self.jobs if j.state=="DONE")
        for j in self.jobs:
            owners=[s for s,slots in self.occupancy.items() if j.id in slots.values()]
            expected={s for s in [j.holds,j.reserved] if s}
            assert set(owners)==expected,(j.id,owners,expected)
            assert j.trace==[s.name for s in j.route[:j.index]]
            if j.state=="DONE":assert not owners
    @property
    def robot_busy(self):return False

    def report(self):
        return dict(simulated_seconds=round(self.time,2),completed_slides=self.completed,
                    total_slides=len(self.slide_records),jobs=[
                    dict(batch=j.id,protocol=j.protocol,slides=len(j.slides),state=j.state,
                         trace=j.trace,carrier=j.carrier,scan_attempts=j.scan_attempts) for j in self.jobs],
                    slides=self.slide_records,events=self.events,
                    limitations=["Demo cycle times; not throughput estimates.",
                    "Kinematic carrier handoffs; no grasp or insertion physics.",
                    "Slide-to-carrier conversion is logical, not individual-slide manipulation."])
