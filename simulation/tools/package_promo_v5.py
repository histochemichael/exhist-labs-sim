"""Export the accepted authoring playback into a portable, read-only v5 bundle."""
import argparse,hashlib,json,os,shutil,sys,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import imageio_ffmpeg

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'promo_v5'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);args=ap.parse_args()
    source=args.source.resolve();sys.path.insert(0,str(source))
    from render_promo_revision import Playback,cinematic_camera
    from promo_gripper_inset import active,camera as detail_camera
    OUT.mkdir(exist_ok=True);(OUT/'evidence').mkdir(exist_ok=True);(OUT/'images').mkdir(exist_ok=True)
    author=source/'promo_v5'
    delivery=json.loads((author/'delivery-gripper-closeups-rollaway-verification.json').read_text())
    movie=author/'ExHist-Labs-Promo-60s-v5-gripper-closeups-rollaway.mp4'
    assert delivery['passed'] and delivery['video_sha256']==sha(movie)
    shutil.copy2(movie,OUT/'ExHist-Labs-Promo-60s.mp4')
    shutil.copy2(author/'exhist-logo.png',OUT/'exhist-logo.png')
    for name in ['sequence-solid-audit.json','swap-mechanics-audit.json','background-playback-audit.json','oven-headroom-concept.json','rollaway-motion-report.json','delivery-gripper-closeups-rollaway-verification.json','promo-final-gripper-closeups-rollaway-manifest.json']:
        shutil.copy2(author/name,OUT/'evidence'/('author-'+name))
    for name in ['v5-inset-rollaway-review-0675.jpg','v5-inset-rollaway-review-1245.jpg','v5-inset-rollaway-review-1650.jpg']:
        shutil.copy2(author/name,OUT/'images'/name)
    root=ET.parse(author/'promo_final_scene.xml').getroot();resources=[]
    for node in root.iter():
        if not node.get('file'):continue
        src=Path(node.get('file'))
        if not src.is_absolute():src=(author/src).resolve()
        relative=src.relative_to(source);dest=ROOT/relative
        if dest.exists():
            assert sha(src)==sha(dest),f'Existing asset differs: {relative}'
        else:
            dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dest)
        node.set('file',os.path.relpath(dest,OUT).replace('\\','/'))
        resources.append(dict(file=node.get('file'),sha256=sha(dest)))
    ET.indent(root);ET.ElementTree(root).write(OUT/'scene.xml',encoding='utf-8',xml_declaration=True)
    p=Playback();m=mujoco.MjModel.from_xml_path(str(OUT/'scene.xml'))
    arrays=['body_pos','body_quat','body_mass','body_inertia','body_parentid','jnt_type','jnt_pos','jnt_axis','jnt_range','geom_type','geom_pos','geom_quat','geom_size','geom_rgba','geom_dataid','mesh_vert','mesh_face','site_pos','site_quat','eq_data']
    for name in arrays:assert np.array_equal(getattr(m,name),getattr(p.m,name)),f'Packaging changed {name}'
    assert (m.nq,m.nmocap,m.ngeom,m.nbody)==(p.m.nq,p.m.nmocap,p.m.ngeom,p.m.nbody)
    q=[];pos=[];rot=[];cams=[];details=[];opacity=[];kind=[]
    def values(c):return [*c.lookat,c.distance,c.azimuth,c.elevation]
    for i in range(1800):
        t=i/30;st,_,_=p.update(t)
        q.append(p.d.qpos.copy());pos.append(p.d.mocap_pos.copy());rot.append(p.d.mocap_quat.copy())
        cams.append(values(cinematic_camera(st,p.report,p.d.qpos,p.m)))
        label,alpha=active(t,p.report);opacity.append(alpha);kind.append(1 if label=='RACK PICKUP' else 2 if label else 0)
        details.append(values(detail_camera(p.d,label)) if label else [0]*6)
    np.savez_compressed(OUT/'playback.npz',qpos=q,mocap_pos=pos,mocap_quat=rot,camera=cams,detail_camera=details,inset_alpha=opacity,inset_kind=kind)
    evidence=dict(passed=True,frames=1800,fps=30,duration_s=60,resources=resources,
        compiled_model_arrays_equal=arrays,author_scene_sha256=sha(author/'promo_final_scene.xml'),
        packaged_scene_sha256=sha(OUT/'scene.xml'),playback_sha256=sha(OUT/'playback.npz'),video_sha256=sha(OUT/'ExHist-Labs-Promo-60s.mp4'),
        scope='Only resource paths rewritten. Compiled geometry, joints and inertia arrays match author model exactly. All backgrounds and cameras sampled from accepted playback at original output times. No new physical-validation claim.')
    (OUT/'evidence/portability.json').write_text(json.dumps(evidence,indent=2))
    # Full-length, indefinitely looping README preview; original MP4 is retained.
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    filters='fps=10,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=5'
    subprocess.run([ffmpeg,'-y','-v','error','-i',str(OUT/'ExHist-Labs-Promo-60s.mp4'),'-filter_complex',filters,'-loop','0',str(OUT/'preview.gif')],check=True)
    print(json.dumps(dict(frames=1800,resources=len(resources),gif_bytes=(OUT/'preview.gif').stat().st_size,mp4_bytes=movie.stat().st_size),indent=2))

if __name__=='__main__':main()
