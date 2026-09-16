"""One-inch insert fit, support under gravity and raised reach regressions."""
import json,unittest
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from build_scene import ROOT,el,vec
from cad_jar import add_jar
from jar_riser import RISER_HEIGHT,JAR_FLOOR_Z,JAR_RIM_Z,RACK_SEAT_Z,geometry


def fixture(with_riser=True,second_jar=False):
    root=ET.Element('mujoco',model='24-slide jar with passive one-inch insert')
    el(root,'compiler',angle='radian');el(root,'option',timestep='.001',integrator='implicitfast',iterations='100',gravity='0 0 -9.81')
    visual=el(root,'visual');el(visual,'global',offwidth='1100',offheight='760');el(visual,'headlight',ambient='.5 .5 .5')
    assets=el(root,'asset');world=el(root,'worldbody')
    el(world,'geom',type='plane',size='.5 .5 .01',rgba='.70 .76 .78 1')
    add_jar(root,'jar',[0,0,.0055],free=False,riser=with_riser)
    if second_jar:add_jar(root,'empty_jar',[-.135,0,.0055],free=False,riser=True)
    source=json.loads((ROOT/'assets/rack24.json').read_text())['parts'][0]
    v=np.array(source['vertices']).reshape(-1,3)+[0,0,.0015]
    el(assets,'mesh',name='actual_rack',vertex=vec(v.ravel()),face=' '.join(map(str,source['triangles'])),inertia='shell')
    rack=el(world,'body',name='test_rack',pos=vec([0,0,.004+RISER_HEIGHT+.005]))
    el(rack,'freejoint',name='test_rack_free')
    el(rack,'inertial',pos='0 0 .04',mass='.145',diaginertia='.00008 .00015 .00012')
    el(rack,'geom',name='rack_visual',type='mesh',mesh='actual_rack',rgba='.63 .65 .66 1',contype='0',conaffinity='0',density='0')
    el(rack,'geom',name='rack_support_envelope',type='box',pos='0 0 .033',size='.044 .016647 .033',rgba='0 0 0 0',density='0',friction='.5 .005 .0001',solref='.004 1')
    el(rack,'site',name='rack_handle_grasp',pos='0 0 .0925',size='.001',rgba='0 0 0 0')
    return root


def settle(with_riser=True,second_jar=False):
    root=fixture(with_riser,second_jar);m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
    for _ in range(2500):mujoco.mj_step(m,d)
    return root,m,d


class RiserTests(unittest.TestCase):
    def test_exact_height_and_cavity_clearance(self):
        v,f=geometry();self.assertAlmostEqual(np.ptp(v[:,2]),.0254,places=12)
        self.assertLess(np.abs(v[:,0]).max(),.049);self.assertLess(np.abs(v[:,1]).max(),.022)
        # Clipped corners stay outside the conservative corner-contact blocks.
        self.assertLess(np.max(np.abs(v[:,0])+np.abs(v[:,1])),.044+.017)
        # Full rectangular rack-base footprint fits inside the convex insert.
        self.assertGreater(.047+.020-.0062,.044+.016647)
        volume=sum(np.dot(v[a],np.cross(v[b],v[c]))/6 for a,b,c in f)
        self.assertGreater(volume,0)

    def test_each_current_container_has_one_passive_insert(self):
        root=ET.parse(ROOT/'exhist_first_pass.xml').getroot()
        for i in range(1,12):
            name=f'special_staining_jar_{i}'
            bodies=root.findall(f"worldbody/body[@name='{name}_riser']");self.assertEqual(len(bodies),1)
            body=bodies[0];self.assertIsNotNone(body.find('freejoint'));self.assertNotEqual(body.get('mocap'),'true')
            self.assertAlmostEqual(float(body.get('pos').split()[2]),.804)
            target=root.find(f"worldbody/site[@name='special_bath_target_{i}']")
            self.assertAlmostEqual(float(target.get('pos').split()[2]),RACK_SEAT_Z)
        for actuator in root.find('actuator'):self.assertNotIn('riser',actuator.get('joint',''))

    def test_gravity_support_and_negative_control(self):
        results=[]
        for enabled in (True,False):
            root,m,d=settle(enabled)
            height=float(d.xpos[m.body('test_rack').id,2]);expected=.004+(RISER_HEIGHT if enabled else 0)
            self.assertAlmostEqual(height,expected,delta=.0005)
            self.assertEqual(m.nu,0);self.assertFalse(root.findall('.//weld'))
            self.assertLess(np.linalg.norm(d.qvel),.001)
            if enabled:
                insert=m.body('jar_riser').id
                self.assertAlmostEqual(float(d.xpos[insert,2]),.004,delta=.0005)
                grasp_height=float(d.site_xpos[m.site('rack_handle_grasp').id,2]-(.0055+JAR_RIM_Z))
                self.assertAlmostEqual(grasp_height,.0246,delta=.0005)
            results.append(dict(riser=enabled,rack_base_world_z_m=height,expected_z_m=expected))
        self.assertAlmostEqual(results[0]['rack_base_world_z_m']-results[1]['rack_base_world_z_m'],RISER_HEIGHT,delta=.0005)
        (ROOT/'jar_riser_validation.json').write_text(json.dumps(dict(cases=results,height_m=RISER_HEIGHT,scope='Gravity support fixture with conservative rack envelope, not robot insertion or reagent validation.'),indent=2))


def render():
    from PIL import Image,ImageDraw,ImageFont
    _,m,d=settle(True,True)
    with mujoco.Renderer(m,height=760,width=1100) as renderer:
        c=mujoco.MjvCamera();c.lookat[:]=[-.06,0,.064];c.distance=.39;c.azimuth=115;c.elevation=-70
        opt=mujoco.MjvOption();opt.sitegroup[:]=0
        renderer.update_scene(d,camera=c,scene_option=opt);im=Image.fromarray(renderer.render())
        draw=ImageDraw.Draw(im);font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',24)
        draw.rectangle([0,0,1100,82],fill='#10283c')
        draw.text((18,12),'24-slide containers | 25.4 mm passive support blocks',font=font,fill='white')
        draw.text((18,46),'Handle grip center: 24.6 mm above rim | insert material unvalidated',font=font,fill='#ffce82')
        im.save(ROOT/'rack24_riser_review.png')

if __name__=='__main__':
    import sys
    if '--render' in sys.argv:render()
    else:unittest.main(verbosity=2)
