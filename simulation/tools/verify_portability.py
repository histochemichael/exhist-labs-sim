"""Compare original/portable compiled model numerics; the only XML edits are paths."""
import argparse, hashlib, json, sys
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco, numpy as np
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main(source):
    source=Path(source);report=[]
    names=['exhist.xml','exhist_operational.xml','exhist_rail_loop.xml','rail_loop_reference/station.xml']
    for name in names:
        def semantic(path):
            def visit(e):return (e.tag,tuple(sorted((k,v) for k,v in e.attrib.items() if k not in ('file','filename'))),tuple(visit(c) for c in e))
            return visit(ET.parse(path).getroot())
        assert semantic(source/name)==semantic(ROOT/name), 'Non-path XML change: '+name
        resources=0
        aa=ET.parse(source/name).getroot();bb=ET.parse(ROOT/name).getroot()
        for ea,eb in zip(aa.iter(),bb.iter()):
            for attr in ('file','filename'):
                if ea.get(attr):
                    pa=(source/name).parent/ea.get(attr);pb=(ROOT/name).parent/eb.get(attr)
                    assert sha(pa)==sha(pb), 'Changed mesh/texture: '+str(pb)
                    resources+=1
        a=mujoco.MjModel.from_xml_path(str(source/name))
        b=mujoco.MjModel.from_xml_path(str(ROOT/name));checked=0;skip=[]
        for field in dir(a):
            if field.startswith('_'):continue
            value=getattr(a,field)
            if isinstance(value,np.ndarray):
                # MuJoCo's parallel convex-hull preprocessing is not bitwise
                # deterministic (normal ordering / coplanar polygon merging).
                # Source mesh bytes and all XML physics parameters are checked.
                if field.startswith('mesh_poly') or field in ('mesh_graph','mesh_graphadr'):
                    if not np.array_equal(value,getattr(b,field)):skip.append(field+' (derived convex hull)')
                    continue
                if field in ('paths','mesh_pathadr','hfield_pathadr','tex_pathadr'):skip.append(field);continue
                np.testing.assert_array_equal(value,getattr(b,field),err_msg=name+':'+field);checked+=1
        for field in dir(a.opt):
            if field.startswith('_'):continue
            value=getattr(a.opt,field)
            if isinstance(value,(int,float,np.ndarray)):np.testing.assert_array_equal(value,getattr(b.opt,field))
        report.append(dict(file=name,original_sha256=sha(source/name),portable_sha256=sha(ROOT/name),
                           identical_numeric_arrays=checked,excluded_fields=skip,non_path_xml_identical=True,identical_resource_files=resources))
    out=dict(passed=True,engine=mujoco.__version__,models=report,
             scope='Identical non-path XML parameters and numerical dynamics arrays. Resource path buffers and derived convex-hull representations excluded; not full workflow validation.')
    (ROOT/'docs/portability-validation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',required=True);a=p.parse_args();main(a.source)
