"""Render the corrected CAD lids, not an equipment-operation proof video."""
import json
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from equipment_access import Access
from render_access_review import lighting
from build_scene import ROOT
from access_suite import source_hashes


def main():
    a = Access('st_unload')
    lighting(a)
    opt = mujoco.MjvOption()
    opt.sitegroup[:] = 0
    c = mujoco.MjvCamera()
    c.lookat[:] = [0, .55, 1.16]
    c.distance = 2.25
    c.elevation = -23
    font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 25)
    sheet = Image.new('RGB', (1600, 1050), '#10283c')
    with mujoco.Renderer(a.m, height=480, width=800) as renderer:
        for k, angle in enumerate((40, 125, 220, 310)):
            c.azimuth = angle
            renderer.update_scene(a.d, c, scene_option=opt)
            im = Image.fromarray(renderer.render())
            sheet.paste(im, ((k % 2)*800, 60+(k//2)*480))
    ImageDraw.Draw(sheet).text((20, 17), 'Tinted covers retained | visual inspection only | joints and contacts unchanged', font=font, fill='white')
    sheet.save(ROOT/'machine_glazing_angles.png')
    c.azimuth = 40
    with mujoco.Renderer(a.m, height=720, width=1280) as renderer:
        renderer.update_scene(a.d, c, scene_option=opt)
        preview = Image.new('RGB', (1280, 794), '#10283c')
        preview.paste(Image.fromarray(renderer.render()), (0, 48))
        draw = ImageDraw.Draw(preview)
        draw.text((18, 11), 'ExHist Labs | transparent inspection lids', font=font, fill='white')
        small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 17)
        draw.text((18, 771), 'Closed enclosures retained. Static material preview; contacts, joints and controllers unchanged.', font=small, fill='#d5edf5')
        preview.save(ROOT/'machine_glazing_corrected.png')
    report = dict(controller_unchanged=source_hashes() == json.loads((ROOT/'stability_validation.json').read_text())['source_sha256'],
                  scope='Static visual QA. Not a new process or access-cycle recording.')
    (ROOT/'machine_glazing_render.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report))


if __name__ == '__main__':
    main()
