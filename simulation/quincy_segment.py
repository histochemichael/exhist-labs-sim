"""Test a handle-aligned hinge segment before integrating physical re-docking."""
import math,json
import equipment_access as e
from access_planning import search

if __name__=='__main__':
    import sys
    if '--plan' in sys.argv:
        p=search('quincy',math.radians(20),1.)
        assert 'path' in p
        (e.ROOT/'access_quincy_plan.json').write_text(json.dumps(p,indent=2))
    a=e.Access('quincy');a.meta['open_target']=math.radians(20)
    for _ in range(100000):
        if a.step():break
    r=a.result();(e.ROOT/'quincy_segment_validation.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k!='trace'},indent=2))
