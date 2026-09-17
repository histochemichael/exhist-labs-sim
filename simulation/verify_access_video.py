"""Decode the complete review and compare its frames to the dynamics recordings."""
import json,sys
import numpy as np
from build_scene import ROOT
from equipment_access import KINDS
from access_suite import source_hashes
sys.path.insert(0,r'C:/Users/Owner/.codex/visualizations/2026/09/07/01a079ae-3b7f-7562-a6b4-d9df80acd01c/st5020-demo/tools')
import imageio_ffmpeg
r=json.loads((ROOT/'access_video_validation.json').read_text());assert r['source_sha256']==source_hashes()
expected=sum(len(np.load(ROOT/f'access_{k}_states.npz')['time']) for k in KINDS)
reader=imageio_ffmpeg.read_frames(str(ROOT/'ExHist-Equipment-Access-Contact-Review.mp4'));meta=next(reader);count=sum(1 for _ in reader)
assert count==expected==r['frames'];assert abs(meta['fps']-15)<.001
r.update(decoded_frames=count,decoded_size=meta['size'],decode_passed=True,head_view='Proposed fixed head-side camera; arm occlusion at some poses; hardware calibration unvalidated')
(ROOT/'access_video_validation.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
