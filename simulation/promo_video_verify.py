"""Decode the delivered MP4, check its identity/timing, and save review frames."""
import json,hashlib
import numpy as np
import imageio_ffmpeg
from PIL import Image
from promo_demo import OUT,MODEL

video=OUT/'ExHist-Labs-Promo-60s.mp4'
manifest=json.loads((OUT/'promo_60_manifest.json').read_text())
assert manifest['source_states_sha256']==hashlib.sha256((OUT/'promo_states.npz').read_bytes()).hexdigest()
assert manifest['scene_sha256']==hashlib.sha256(MODEL.read_bytes()).hexdigest()
reader=imageio_ffmpeg.read_frames(str(video),pix_fmt='rgb24');meta=next(reader)
assert meta['size']==(1280,720) and abs(meta['fps']-30)<1e-6 and abs(meta['duration']-60)<.02,meta
selected={570:'upright-oven-pickup',750:'oven-closed',1030:'claw-withdrawal',1110:'upright-prealignment',1175:'container-release',1618:'rear-vessel',1750:'logo'}
count=0;last=None
for frame,data in enumerate(reader):
    count+=1
    if frame in selected:
        pixels=np.frombuffer(data,dtype=np.uint8).reshape(720,1280,3)
        Image.fromarray(pixels).save(OUT/('video-check-'+selected[frame]+'.jpg'),quality=92)
    last=data
assert count==1800,count
logo=Image.open(OUT/'exhist-logo.png').convert('RGB');logo.thumbnail((720,720),Image.Resampling.LANCZOS)
expected=Image.new('RGB',(1280,720),'white');expected.paste(logo,((1280-logo.width)//2,(720-logo.height)//2))
error=float(np.abs(np.frombuffer(last,dtype=np.uint8).reshape(720,1280,3).astype(float)-np.asarray(expected)).mean())
assert error<3,error
report=dict(decoded_frames=count,duration_s=meta['duration'],fps=meta['fps'],size=meta['size'],logo_mean_absolute_pixel_error=error,video_sha256=hashlib.sha256(video.read_bytes()).hexdigest(),source_states_sha256=manifest['source_states_sha256'],scene_sha256=manifest['scene_sha256'])
(OUT/'promo_video_verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
