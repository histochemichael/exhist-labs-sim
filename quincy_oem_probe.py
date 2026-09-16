"""Check the untouched OEM handle with corrected original-claw CAD contacts."""
import json
import equipment_access as e
import quincy_offset_pull
quincy_offset_pull.add=lambda moving,center,R:center
if __name__=='__main__':
    a=e.Access('quincy')
    for _ in range(100000):
        if a.step():break
    r=a.result();(e.ROOT/'quincy_oem_profile_validation.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k!='trace'},indent=2))
