"""Apply the corrected handle to imported workstation racks without changing originals."""
import json,copy,re
import numpy as np
from build_scene import ROOT,ASSETS

def build():
    source=json.loads((ASSETS/"leica_rack_handled.json").read_text())
    rack=next(p for p in source["parts"] if p["name"]=="root/Slide rack")
    handle=next(p for p in source["parts"] if "Removable snap handle" in p["name"])
    sv=np.asarray(rack["vertices"]).reshape(-1,3)
    hv=np.asarray(handle["vertices"]).reshape(-1,3)
    assert abs(hv[:,2].max()-.10051)<1e-5,"Wrong tall-handle revision"
    slots=[int(m.group(1)) for p in source["parts"] if (m:=re.search(r"LEICA_SLIDE_SLOT_(\d+)",p["name"]))]
    assert len(slots)==28 and 15 not in slots and 16 not in slots
    workstation=json.loads((ASSETS/"leica_workstation.json").read_text())
    additions=[];report=[]
    for p in workstation["parts"]:
        if not p["name"].rsplit("/",1)[-1].startswith("Slide rack "):continue
        v=np.asarray(p["vertices"]).reshape(-1,3)
        assert v.shape==sv.shape
        shift=(v.min(0)+v.max(0)-sv.min(0)-sv.max(0))/2
        # Mesh vertex order differs between Fusion exports; compare point clouds.
        source_points=np.unique(np.round(sv+shift,8),axis=0)
        target_points=np.unique(np.round(v,8),axis=0)
        error=0.
        for start in range(0,len(source_points),64):
            distances=np.sum((source_points[start:start+64,None,:]-target_points[None,:,:])**2,axis=2)
            error=max(error,float(np.sqrt(distances.min(axis=1).max())))
        assert error<1e-5,(p["name"],error)
        new=copy.deepcopy(handle)
        new["name"]=p["name"].rsplit("/",1)[0]+"/Removable snap handle - slots 15-16"
        new["vertices"]=(hv+shift).reshape(-1).tolist()
        additions.append(new);report.append(dict(rack=p["name"],translation_m=shift.tolist(),registration_error_m=error))
    assert len(additions)==8
    # Preserve empty versus loaded rack states; exclude any reserved center slides.
    workstation["parts"]=[p for p in workstation["parts"] if not re.search(r"CYCLE_LEICA_SLIDE_(15|16)(?=:)",p["name"])]
    workstation["parts"].extend(additions)
    (ASSETS/"leica_workstation_handled.json").write_text(json.dumps(workstation,separators=(",",":")))
    result=dict(revision="Corrected 100.51-mm top, copied 24-rack grip",source=source["document"],
        source_slides=slots,embedded_racks=report,source_files_unchanged=True,
        limitations=["Seated visual assembly only; snap retention and coverslipper compatibility unvalidated."])
    (ROOT/"handled_rack_validation.json").write_text(json.dumps(result,indent=2))
    print("Handled rack: 28 slides; slots 15/16 empty; 8 embedded rack handles registered.")
if __name__=="__main__":build()
