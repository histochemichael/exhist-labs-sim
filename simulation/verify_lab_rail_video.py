"""Decode the delivered movie and inspect non-overlay motion/content."""
import json
import imageio.v2 as imageio
import numpy as np
from PIL import Image
from build_scene import ROOT


def verify():
    path=ROOT/'ExHist-Live-LeHisto-Rail-Loop.mp4'
    expected=json.loads((ROOT/'rail_loop_lab_video.json').read_text())
    reader=imageio.get_reader(str(path));meta=reader.get_meta_data();count=reader.count_frames()
    frames=[reader.get_data(i) for i in (0,400,850,1174)];reader.close()
    assert count==expected['frames']==1205
    assert meta['size']==(1280,720) and abs(meta['fps']-15)<1e-6
    assert all(np.std(f[85:400,:850])>20 for f in frames)
    delta=float(np.mean(np.abs(frames[1][85:400,:850].astype(float)-frames[2][85:400,:850])))
    assert delta>1.,'Station view is unexpectedly frozen'
    sheet=Image.new('RGB',(1280,720))
    for i,f in enumerate(frames):sheet.paste(Image.fromarray(f).resize((640,360)),((i%2)*640,(i//2)*360))
    sheet.save(ROOT/'rail_loop_lab_video_qa.jpg',quality=85)
    result=dict(passed=True,decoded_frames=count,fps=meta['fps'],duration_s=count/meta['fps'],
                content_motion_delta=delta,sampled_frames=[0,400,850,1174],size=meta['size'])
    (ROOT/'rail_loop_lab_video_qa.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':verify()
