"""Phase-driven machine activity preview and individual CAD joint inspection."""
import json,math
import numpy as np
from run_lab import ROOT

class Machines:
    def __init__(self,m,d):
        self.m=m;self.d=d;self.manifest=json.loads((ROOT/"machine_articulation.json").read_text())
        self.rows=self.manifest["joint_map"];self.elapsed={};self.states={};self.inspect=False;self.selected=0;self.jog_phase=0
        self.dofs=[(row,dof) for row in self.rows for dof in row["dofs"]]
        self.shells=[i for i in range(m.ngeom) if (m.geom(i).name or "").startswith("routine_") and (m.geom(i).name or "").endswith("_shell")]
        self.shell_alpha=m.geom_rgba[self.shells,3].copy();self.xray=False;self.last_time=0.
        for row in self.rows:self.elapsed[row["body"]]=0.
    def toggle_xray(self):
        self.xray=not self.xray
        self.m.geom_rgba[self.shells,3]=.12 if self.xray else self.shell_alpha
    def select(self,delta):
        self.selected=(self.selected+delta)%len(self.dofs);self.jog_phase=0.
    def jog(self,reset=False):
        if not self.inspect:return
        row,dof=self.dofs[self.selected]
        self.jog_phase=0 if reset else (self.jog_phase+.25)%1.25
        value=self.target(dof,self.jog_phase*.15)
        self.d.qpos[self.m.joint(dof["name"]).qposadr[0]]=value
    def target(self,dof,fraction):
        if dof["locked"]:return dof["limits_delta"][0] or 0.
        lo,hi=dof["limits_delta"];cap=.035 if dof["kind"]=="slide" else .3
        low=max(-cap,lo if lo is not None else -cap)
        high=min(cap,hi if hi is not None else cap)
        end=high if abs(high)>=abs(low) else low
        return end*fraction
    def update(self,lab):
        dt=max(0,lab.time-self.last_time);self.last_time=lab.time
        for station in ["routine_a","routine_b"]:
            jobs=[j for j in lab.jobs if j.holds==station and j.state in ["PROCESSING","FAULT"]]
            job=jobs[0] if jobs else None
            phase=job.route[job.index].name if job else "idle"
            fault=station in lab.faults
            for device in ["ST5020","CV5030"]:
                active=not fault and not lab.paused and ((device=="ST5020" and phase=="stain") or (device=="CV5030" and phase=="coverslip"))
                self.states[(station,device)]="FAULT" if fault else "PAUSED" if lab.paused else "RUNNING" if active else "CURING" if phase=="cure" and device=="CV5030" else "IDLE"
                rgba=[1,.1,.1,1] if fault else [1,.65,.05,1] if active else [.15,.5,.3,1]
                self.m.site_rgba[self.m.site(station+"_"+device+"_activity").id]=rgba
            for row in self.rows:
                if row["station"]!=station:continue
                name=row["cad_name"]
                inspection_only=name.startswith(("SIM_","CYCLE_FILL")) or any(k in name for k in ["DOOR","HOOD","DRAWER","COVER_OPEN"])
                active=self.states[(station,row["device"])]=="RUNNING" and not inspection_only and not row["hidden_reference"]
                if self.inspect or not active:continue
                self.elapsed[row["body"]]+=dt
                # Small, smooth strokes make activity visible without asserting a device protocol.
                u=.5-.5*math.cos(2*math.pi*self.elapsed[row["body"]]/4)
                for dof in row["dofs"]:
                    self.d.qpos[self.m.joint(dof["name"]).qposadr[0]]=self.target(dof,u)
        # Two CAD telescoping sliders encode a half-stroke relationship by design.
        if not self.inspect:
            for station in ["routine_a","routine_b"]:
                half=next(r for r in self.rows if r["station"]==station and r["cad_name"]=="QC_TS_Z_HALF_mm")
                full=next(r for r in self.rows if r["station"]==station and r["cad_name"]=="TS5025_TRANSFER_Z_mm")
                self.d.qpos[self.m.joint(half["dofs"][0]["name"]).qposadr[0]]=.5*self.d.qpos[self.m.joint(full["dofs"][0]["name"]).qposadr[0]]
    def status(self):
        rows=[f"{s}: ST5020 {self.states.get((s,'ST5020'),'IDLE')} | CV5030 {self.states.get((s,'CV5030'),'IDLE')}" for s in ["routine_a","routine_b"]]
        if self.inspect:
            row,dof=self.dofs[self.selected];q=self.d.qpos[self.m.joint(dof["name"]).qposadr[0]]
            rows.append(f"INSPECT {self.selected+1}/{len(self.dofs)}: {row['station']} {row['cad_name']} | {dof['kind']} {q:.4f}")
            rows.append("LOCKED" if dof["locked"] else "Hidden/reference geometry" if row["hidden_reference"] else "G: small jog | H: source pose")
        return "\n".join(rows)
