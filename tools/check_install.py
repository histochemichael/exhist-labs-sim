"""Fast self-contained installation check; does not claim physical validation."""
import importlib.metadata as metadata
import json, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import mujoco, numpy as np
from lab_rail_loop import verify_snapshot, LiveLoop
def main():
    assert mujoco.__version__=='3.4.0','Activate exhist-sim (MuJoCo 3.4.0).'
    verify_snapshot()
    for name in ('exhist.xml','exhist_operational.xml','exhist_rail_loop.xml'):
        model=mujoco.MjModel.from_xml_path(str(ROOT/name))
        data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        assert np.isfinite(data.qpos).all()
        print(name,model.nbody,'bodies',model.njnt,'joints',model.nmesh,'meshes')
    loop=LiveLoop();loop.advance(.08);assert np.isfinite(loop.d.qpos).all();loop.close()
    print('PASS: models compile and live controller initializes; this is not a full route test.')
if __name__=='__main__':main()
