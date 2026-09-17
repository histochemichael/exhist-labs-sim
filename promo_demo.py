"""ExHist promo: explicit cinematic state playback, NOT autonomous validation.

Door/drawer reference motions come from recorded contact trials. Translated
and retargeted arms, the new rack transfer and mobile travel are staged IK.
Only this separate scene and promo outputs are written; base evidence is kept.
"""
import argparse,copy,hashlib,json,math,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from first_pass import ik,kinematics,GRASP_ROTATION,yaw_matrix,smooth

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'promo_v4';MODEL=OUT/'promo_scene.xml';FPS=20
LEFT=['left_'+s+'_joint' for s in ('shoulder_pitch','shoulder_roll','bicep_yaw','elbow_pitch','forearm_yaw','wrist_pitch','wrist_roll')]
RIGHT=[n.replace('left_','right_') for n in LEFT]
RACK_GRIP=np.array([-.00001505,-.000045,.098996])
GRIP_TILT=0.
GRIP_R=np.array([[1,0,0],[0,np.cos(GRIP_TILT),-np.sin(GRIP_TILT)],[0,np.sin(GRIP_TILT),np.cos(GRIP_TILT)]])@GRASP_ROTATION
LOOP_PREFIX='promo_loop_'

def vec(x):return ' '.join(f'{float(v):.10g}' for v in x)
def quat(R):
    q=np.zeros(4);mujoco.mju_mat2Quat(q,np.ascontiguousarray(R).ravel());return q
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def build():
    OUT.mkdir(exist_ok=True)
    if not (OUT/'exhist-logo.png').exists():
        import shutil
        shutil.copy2(ROOT/'promo_v2/exhist-logo.png',OUT/'exhist-logo.png')
    root=ET.parse(ROOT/'exhist_operational.xml').getroot();root.set('model','ExHist Labs PROMO - staged simulation concept')
    world=root.find('worldbody');assets=root.find('asset')
    for e in assets:
        if e.get('file'):e.set('file',str((ROOT/e.get('file')).resolve()))
    # Concept-only taller oven: keep shelf, controls and pull datum unchanged;
    # extend upper shell, liner, door and hinges by 120 mm for a vertical grip.
    for k in (0,2,5,6,7,8,10):
        mesh=assets.find(f"mesh[@name='passive_quincy_{k}']")
        lines=Path(mesh.get('file')).read_text().splitlines()
        vertices=np.array([[float(x) for x in line.split()[1:4]] for line in lines if line.startswith('v ')])
        vertices[:,2]+=.12*np.clip((vertices[:,2]-.30)/.102,0,1)
        faces=[]
        for line in lines:
            if line.startswith('f '):
                ids=[int(x.split('/')[0])-1 for x in line.split()[1:]]
                faces.extend([ids[0],ids[j],ids[j+1]] for j in range(1,len(ids)-1))
        mesh.attrib.pop('file');mesh.set('vertex',vec(vertices.ravel()));mesh.set('face',' '.join(str(x) for tri in faces for x in tri))
    root.find('visual/global').attrib.update(offwidth='1280',offheight='720')
    base=world.find("body[@name='nori_mobile_base']");base.set('pos','0 -1 .002')
    wrist=base.find(".//body[@name='left_wrist_roll_link']")
    ET.SubElement(wrist,'site',name='promo_left_claw',pos='.0025 .00652 -.114',size='.002',rgba='0 0 0 0')
    # Diagnostic optical frame at the CAD camera's lens face. FOV is provisional.
    lens_file=assets.find("mesh[@name='lehisto_part_23']").get('file')
    import struct
    raw=Path(lens_file).read_bytes();count=struct.unpack_from('<I',raw,80)[0]
    assert len(raw)==84+50*count,'Expected binary STL camera lens asset'
    lv=np.array([struct.unpack_from('<9f',raw,84+50*i+12) for i in range(count)]).reshape(-1,3)
    eye=(lv.min(0)+lv.max(0))/2;eye[1]=lv[:,1].min()-.001
    z=eye-np.array([0,-.0961885,.0933]);z/=np.linalg.norm(z);x=np.cross([0,0,-1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    ET.SubElement(base.find(".//body[@name='lehisto_mount']"),'camera',name='promo_nori_gripper',pos=vec(eye),xyaxes=vec(np.r_[x,y]),fovy='70')
    # All three ovens share the reachable front-of-bench alignment.
    for k in (1,2,3):
        ob=world.find(f"body[@name='quincy_{k}']");pos=np.fromstring(ob.get('pos'),sep=' ');pos[1]=.10;ob.set('pos',vec(pos))
        door=ob.find('body');pull=ET.SubElement(door,'body',name=f'promo_oven_{k}_pull',pos='.084647787 -.31305711 .28349999',quat='.64278761 0 0 .76604444')
        for g in list(door):
            if g.tag=='geom' and g.get('mesh')=='passive_quincy_9':door.remove(g)
        ET.SubElement(pull,'geom',name=f'promo_oven_{k}_pull_bar',type='cylinder',pos='0 -.035 0',size='.005 .05',rgba='.055 .06 .065 1',contype='0',conaffinity='0',group='1')
        for n,z in [('top',.045),('bottom',-.045)]:ET.SubElement(pull,'geom',name=f'promo_oven_{k}_pull_{n}',type='box',pos=vec([0,-.007,z]),size='.005 .028 .005',rgba='.055 .06 .065 1',contype='0',conaffinity='0',group='1')
        ET.SubElement(pull,'site',name=f'promo_oven_{k}_handle',pos='0 -.035 0',size='.001',rgba='0 0 0 0')
    oven=world.find("body[@name='quincy_1']")
    # CAD's empty placement-reference rack must not remain after pickup.
    for g in list(oven):
        if g.tag=='geom' and g.get('mesh')=='passive_quincy_11':oven.remove(g)
    rack=world.find("body[@name='B01_leica_rack']")
    ET.SubElement(rack,'geom',name='promo_handle_beam_probe',type='box',pos=vec(RACK_GRIP),size='.010 .001 .0015',rgba='0 0 0 0',contype='0',conaffinity='0',group='5')
    # Suppress native L33 rack/slides so there is exactly one featured rack.
    art=json.loads((ROOT/'machine_articulation.json').read_text())
    cad=json.loads((ROOT/'assets/leica_workstation_handled.json').read_text())
    load=world.find(".//body[@name='routine_a_j66_LOAD_DRAWER_OPEN_mm']")
    for e in list(load):
        if e.tag=='geom' and e.get('name','').startswith('routine_a_art_leica_'):load.remove(e)
    origin=np.array([.495075,-.3648575,.0687])+[.6,.0728575,0]
    for i,p in enumerate(cad['parts']):
        owner=art['part_owners'].get(p['name'])
        if owner!='j66_LOAD_DRAWER_OPEN_mm' or '20 RACK L33' in p['name']:continue
        n='promo_drawer_part_'+str(i);v=np.array(p['vertices']).reshape(-1,3)-origin
        ET.SubElement(assets,'mesh',name=n,vertex=vec(v.ravel()),face=' '.join(map(str,p['triangles'])),inertia='shell')
        ET.SubElement(load,'geom',name=n,type='mesh',mesh=n,rgba=vec(p['rgba']),contype='0',conaffinity='0',density='0',group='1')
    for b in load.iter('body'):
        if 'CYCLE_FILL_' in b.get('name',''):
            for parent in b.iter():
                for g in list(parent):
                    if g.tag=='geom':parent.remove(g)
    stw=world.find(".//body[@name='routine_a_j73_ST_ARM_WRIST_deg']")
    # CAD st coordinate: 50 mm below wrist; support pins sit another 6.4 mm below.
    ET.SubElement(stw,'site',name='promo_stainer_tcp',pos='0 0 -.050',size='.002',rgba='0 0 0 0')
    # Exact prerecorded LeHisto station, placed in the lab's existing transform.
    for b in list(world):
        if b.get('name')=='special_lehisto' or b.get('name','').startswith('special_staining_jar_'):world.remove(b)
    source=ET.parse(ROOT/'rail_loop_reference/station.xml').getroot()
    layout=json.loads((ROOT/'rail_loop_reference/rail_loop_layout.json').read_text());R=np.array(layout['world_rotation']);t=np.array(layout['world_translation'])
    wrapper=ET.SubElement(world,'body',name='promo_special_station',pos=vec(t),quat=vec(quat(R)))
    for e in source.findall('asset/*'):
        e=copy.deepcopy(e)
        for k in ('name','texture','material'):
            if e.get(k):e.set(k,LOOP_PREFIX+e.get(k))
        if e.get('file'):e.set('file',str((ROOT/'rail_loop_reference'/e.get('file')).resolve()))
        assets.append(e)
    for e in source.find('worldbody'):
        if e.tag not in ('body','geom') or e.get('name')=='floor':continue
        e=copy.deepcopy(e);israck=e.get('name')=='rack'
        for n in e.iter():
            for key in ('name','mesh','material'):
                if n.get(key):n.set(key,LOOP_PREFIX+n.get(key))
            if n.tag=='geom':n.attrib.update(contype='0',conaffinity='0')
        if israck:
            for child in list(e):
                if child.tag in ('freejoint','joint','inertial'):e.remove(child)
            e.set('mocap','true');e.set('pos','0 0 -10');world.append(e)
        else:
            for child in list(e):
                if child.tag=='freejoint' or (child.tag=='joint' and child.get('type')=='free'):e.remove(child)
            wrapper.append(e)
    # Retain the optional QC fixture without changing its separate source model.
    qc=ROOT/'exhist_rail_loop_qc45.xml'
    if qc.exists():
        q=ET.parse(qc).getroot()
        f=q.find("worldbody/body[@name='full_lab_context']/body[@name='lab_context_qc45_fixture']")
        if f is not None:world.append(copy.deepcopy(f))
        for a in q.findall('asset/mesh'):
            if a.get('name','').startswith('lab_context_qc45_'):assets.append(copy.deepcopy(a))
    from promo_sorting import build_sorting
    build_sorting(root)
    from promo_geometry import build_container_checks
    counts=build_container_checks(root)
    (OUT/'container_geometry.json').write_text(json.dumps(counts,indent=2))
    ET.indent(root);ET.ElementTree(root).write(MODEL,encoding='utf-8',xml_declaration=True)

class Promo:
    def __init__(self):
        self.m=mujoco.MjModel.from_xml_path(str(MODEL));self.d=mujoco.MjData(self.m)
        self.q={self.m.joint(i).name:int(self.m.jnt_qposadr[i]) for i in range(self.m.njnt)}
        self.frames=[];self.events=[];self.errors=[];self.clock=0.;self.custody='oven';self.rack=np.array([-5.17476733,.26280824,.95042659]);self.rackR=np.eye(3)
        self.rackmid=self.m.body('B01_leica_rack').mocapid[0]
        self.d.mocap_pos[:]=[0,0,-10]
        for name in self.q:
            if name.endswith('_door_joint'):self.d.qpos[self.q[name]]=0
        self.rest('left');self.rest('right');self.setlift(.55)
        self.setbase([6.15,-1.05,np.pi]);self.d.qpos[self.q['left_gripper_joint']]=.65;self.d.qpos[self.q['left_gripper_idler_joint']]=-.65
        self.access={};self.prepare_access();self.lastbase=self.base().copy();self.wheels=np.zeros(2)
        from promo_clearance import Clearance
        self.clearance=Clearance(self.m,self.d);self.spatial_checks=[]
        from promo_geometry import VesselClearance
        self.vessels=VesselClearance(self.m,self.d)
        from promo_sorting import Sorting
        self.sorting=Sorting(self.m,self.d)

    def setlift(self,v):
        self.d.qpos[self.q['lift_extension_joint']]=v;self.d.qpos[self.q['lift_middle_joint']]=v/2
    def rest(self,side):
        for n,v in zip(LEFT if side=='left' else RIGHT,[0,1.25 if side=='left' else -1.25,0,-.45,0,0,0]):self.d.qpos[self.q[n]]=v
    def setbase(self,b):
        for n,v in zip(['base_x','base_y','base_yaw'],[b[0],b[1]+1,b[2]]):self.d.qpos[self.q[n]]=v
    def base(self):return np.array([self.d.qpos[self.q['base_x']],self.d.qpos[self.q['base_y']]-1,self.d.qpos[self.q['base_yaw']]])
    def forward(self):kinematics(self.m,self.d)
    def right(self,p,R=GRIP_R,lift=False,attempts=1):
        names=(['lift_extension_joint'] if lift else [])+RIGHT
        saved=self.d.qpos.copy();pe,re=self.clearance.tighten('nori_right_handle_groove',names,p,R)
        if pe>.001 or re>.01:
            self.d.qpos[:]=saved
            _,pe,re=ik(self.m,self.d,'nori_right_handle_groove',names,p,R,attempts=attempts,iterations=220)
        if pe<.002 and re<.02:pe,re=self.clearance.tighten('nori_right_handle_groove',names,p,R)
        gap=self.clearance.distances('right',self.base()[0]<-4).min()
        if gap<.006 or pe>.002 or re>.02:
            _,pe,re,gap=self.clearance.solve('nori_right_handle_groove',names,np.array(p),R,door=self.base()[0]<-4,attempts=max(3,attempts))
        self.forward();self.errors.append(dict(t=self.clock,side='right',position_m=pe,angle_rad=re))
        if pe>.004 or re>.045:raise RuntimeError(('right target unreachable',p,pe,re,self.base()))
    def left(self,p,R,attempts=1):
        _,pe,re=ik(self.m,self.d,'promo_left_claw',LEFT,p,R,attempts=attempts,iterations=220)
        if self.clearance.distances('left',self.base()[0]<-4).min()<.006:
            _,pe,re,gap=self.clearance.solve('promo_left_claw',LEFT,np.array(p),R,door=self.base()[0]<-4,attempts=max(attempts,4))
        self.forward();self.errors.append(dict(t=self.clock,side='left',position_m=pe,angle_rad=re))
        if pe>.004 or re>.045:raise RuntimeError(('left target unreachable',p,pe,re,self.base()))
    def oven_pose(self,angle,offset=0):
        self.d.qpos[self.q['quincy_1_door_joint']]=angle;self.forward()
        p=self.d.site('promo_oven_1_handle').xpos.copy();self.setbase([-4.92,-.26+p[1]-.057808238,np.pi/2])
        phi=-np.arctan2(p[0]-(-4.92-.125),p[1]-(self.base()[1]-.1135))
        R=yaw_matrix(phi)@np.array([[-1.,0,0],[0,0,-1],[0,-1,0]])
        p=p+R[:,2]*offset;names=['lift_extension_joint']+LEFT
        # Warm starts preserve the outside-elbow branch through the door swing.
        _,pe,re=ik(self.m,self.d,'promo_left_claw',names,p,R,attempts=20 if angle==0 else 1,iterations=250)
        _,pe,re,gap=self.clearance.solve('promo_left_claw',names,p,R,door=True,attempts=5 if angle==0 else 2)
        self.errors.append(dict(t=self.clock,side='left',position_m=pe,angle_rad=re))
        if pe>.002 or re>.025 or gap<.005:raise RuntimeError(('oven clearance/reach',angle,pe,re,gap))
    def left_jaw(self,v):
        self.d.qpos[self.q['left_gripper_joint']]=v;self.d.qpos[self.q['left_gripper_idler_joint']]=-v
    def jaws(self,value):
        for s in ('a','b'):self.d.qpos[self.q['lehisto_jaw_'+s+'_slide']]=value
        self.d.qpos[self.q['lehisto_pinion_joint']]=np.clip(value/.0072,*self.m.joint('lehisto_pinion_joint').range)
    def calibrated_closure(self):
        self.d.mocap_pos[self.rackmid]=self.rack;self.d.mocap_quat[self.rackmid]=quat(self.rackR)
        probe=self.m.geom('promo_handle_beam_probe').id;ft=np.zeros(6);faces=[g for g in range(self.m.ngeom) if self.m.geom(g).name.startswith(('nori_groove_a_handle_','nori_groove_b_handle_'))]
        lo=-.0355;hi=0.
        for _ in range(24):
            v=(lo+hi)/2;self.jaws(v);self.forward();gap=min(mujoco.mj_geomDistance(self.m,self.d,g,probe,.1,ft) for g in faces)
            if gap<.000015:lo=v
            else:hi=v
        self.jaws(0);self.closed_jaw=hi;return hi
    def limited_release(self):
        start=self.d.qpos[self.q['lehisto_jaw_a_slide']];safe=start
        for value in np.arange(start,0,.000025):
            self.jaws(value);self.forward();gap,pair=self.vessels.gaps()
            if gap<.0006:break
            safe=float(value)
        self.jaws(start)
        if safe-start<.00025:raise RuntimeError(('Insufficient release clearance',start,safe))
        self.release_jaw=safe;print('Container-limited jaw opening per side, mm:',(safe-start)*1000,flush=True);return safe
    def prepare_access(self):
        from equipment_access import build as access_build
        from access_suite import source_hashes
        for kind in ('quincy','st_load'):
            report=json.loads((ROOT/f'access_{kind}_validation.json').read_text());assert report['validation_ok']
            # Historical playback only: never relabel recorded validation as current.
            mismatches=[n for n,h in report['source_sha256'].items() if source_hashes().get(n)!=h]
            if mismatches:print('Historical recording; changed source:',kind,mismatches,flush=True)
            root,meta=access_build(kind);m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));d=mujoco.MjData(m)
            states=np.load(ROOT/f'access_{kind}_states.npz');shift=self.m.body(meta['machine']).pos-m.body(meta['machine']).pos
            assert states['qpos'].shape[1]==m.nq
            positions=[];rots=[]
            for q in states['qpos']:
                d.qpos[:]=q;mujoco.mj_kinematics(m,d)
                positions.append(d.site('access_claw_center').xpos.copy()+shift);rots.append(d.site('access_claw_center').xmat.reshape(3,3).copy())
            mapping={n:int(m.joint(n).qposadr[0]) for n in LEFT+['lift_extension_joint','lift_middle_joint','left_gripper_joint','left_gripper_idler_joint',meta['joint']]}
            dock=np.fromstring(root.find("worldbody/body[@name='mobile_chassis']").get('pos'),sep=' ')+shift
            self.access[kind]=dict(time=states['time'],qpos=states['qpos'],mapping=mapping,meta=meta,positions=np.array(positions),rots=np.array(rots),dock=np.r_[dock[:2],np.pi/2])
    def reference(self,kind,t,retarget=False):
        a=self.access[kind];hi=min(np.searchsorted(a['time'],t),len(a['time'])-1);lo=max(hi-1,0);u=0 if hi==lo else (t-a['time'][lo])/(a['time'][hi]-a['time'][lo]);u=np.clip(u,0,1)
        q=(1-u)*a['qpos'][lo]+u*a['qpos'][hi]
        for n,i in a['mapping'].items():
            if retarget and (n in LEFT or n.startswith('lift_')):continue
            self.d.qpos[self.q[n]]=q[i]
        if retarget:
            from scipy.spatial.transform import Rotation,Slerp
            R=Slerp([0,1],Rotation.from_matrix(np.array([a['rots'][lo],a['rots'][hi]])))([u]).as_matrix()[0]
            self.left((1-u)*a['positions'][lo]+u*a['positions'][hi],R,attempts=1)
        self.forward()
        if kind=='st_load':
            joint=a['meta']['joint'];old=self.d.qpos[self.q[joint]];maximum=abs(a['qpos'][:,a['mapping'][joint]]).max();new=old*.23495/maximum
            p=self.d.site('promo_left_claw').xpos.copy()+[0,new-old,0];R=self.d.site('promo_left_claw').xmat.reshape(3,3).copy();self.d.qpos[self.q[joint]]=new
            self.left(p,R,attempts=3)
    def record(self,label):
        sortpos,sortquat=self.sorting.update(self.clock)
        self.forward();b=self.base();delta=b-self.lastbase;heading=(b[2]+self.lastbase[2])/2
        forward=delta[0]*np.cos(heading)+delta[1]*np.sin(heading)
        self.wheels+=(forward+np.array([-1,1])*.145*delta[2])/.075
        for n,v in zip(['left_wheel_joint','right_wheel_joint'],self.wheels):self.d.qpos[self.q[n]]=v
        self.lastbase=b
        if self.custody=='right':
            p=self.d.site('nori_right_handle_groove').xpos;R=self.d.site('nori_right_handle_groove').xmat.reshape(3,3)
            self.rackR=R@GRIP_R.T;self.rack=p-self.rackR@RACK_GRIP
            tilt=np.arccos(np.clip(self.rackR[2,2],-1,1))
            if tilt>np.deg2rad(1):raise RuntimeError(('Loaded rack not upright',label,float(np.rad2deg(tilt))))
            if self.frames and self.frames[-1]['custody']=='right':
                ids=[self.q[n] for n in RIGHT];step=np.max(abs(self.d.qpos[ids]-self.frames[-1]['q'][ids]))
                if step>.30:raise RuntimeError(('Loaded arm branch discontinuity',label,float(step)))
        elif self.custody=='drawer':
            self.rack=np.array([-2.361045,.4171425+self.d.qpos[self.q['routine_a_j66_LOAD_DRAWER_OPEN_mm_0']],.934314]);self.rackR=np.eye(3)
        elif self.custody=='stainer':
            R=self.d.site('promo_stainer_tcp').xmat.reshape(3,3);p=self.d.site('promo_stainer_tcp').xpos
            self.rack=p+R@self.stainer_offset;self.rackR=R@self.stainer_rotation
        self.spatial_checks.append(dict(t=self.clock,phase=label,left=float(self.clearance.distances('left',b[0]<-4).min()),right=float(self.clearance.distances('right',b[0]<-4).min())))
        self.frames.append(dict(q=self.d.qpos.copy(),rack=self.rack.copy(),quat=quat(self.rackR),sortpos=sortpos,sortquat=sortquat,t=self.clock,label=label,custody=self.custody))
        self.clock+=1/FPS
    def segment(self,seconds,label,end=None,callback=None):
        start=self.d.qpos.copy();end=start.copy() if end is None else end.copy();begin=self.clock
        for i in range(round(seconds*FPS)):
            u=smooth((i+1)/(seconds*FPS))
            if not callback:self.d.qpos[:]=start+(end-start)*u
            if callback:callback(u)
            self.record(label)
        self.events.append(dict(start=begin,end=self.clock,label=label,custody=self.custody));print(label,round(self.clock,2),flush=True)
    def joint_move(self,seconds,label,fn):
        start=self.d.qpos.copy();fn();end=self.d.qpos.copy();self.d.qpos[:]=start
        if label in ('RIGHT GRIPPER APPROACH','PREALIGN RACK ABOVE VESSELS','LEFT ARM / OVEN REGRASP','LEFT ARM / DRAWER HANDLE','LEFT HAND REGRASP DRAWER','LEFT ARM DOWN','LEFT ARM REST','LEFT CLAW CLEAR'):
            side='right' if label in ('RIGHT GRIPPER APPROACH','PREALIGN RACK ABOVE VESSELS') else 'left';path=self.clearance.path(start,end,side,door=self.base()[0]<-4)
            def follow(u):
                f=u*(len(path)-1);i=min(int(f),len(path)-2);v=f-i;self.d.qpos[:]=path[i]*(1-v)+path[i+1]*v
            self.segment(seconds,label,callback=follow)
        else:self.segment(seconds,label,end)
    def cart_right(self,seconds,label,target,lift=False,attempts=1):
        self.forward();startp=self.d.site('nori_right_handle_groove').xpos.copy()
        # A continuous Cartesian path preserves the previous IK branch.
        self.segment(seconds,label,callback=lambda u:self.right(startp+(target-startp)*u,lift=lift,attempts=attempts))
    def movebase(self,seconds,label,pose):self.joint_move(seconds,label,lambda:self.setbase(pose))
    def plan(self):
        oven=np.array([-4.92,-.26,np.pi/2]);pickbase=np.array([-5.45,.025,np.pi/2]);stbase=self.access['st_load']['dock'];loadbase=np.array([-2.636045,-.031,np.pi/2])
        self.segment(2,'LAB FLY-THROUGH')
        self.movebase(8,'TRAVEL TO BAKING',[oven[0],-1.05,np.pi])
        self.movebase(1,'FACE THE OVEN',[oven[0],-1.05,np.pi/2]);self.movebase(2,'OVEN DOCK',oven)
        self.left_jaw(.30);self.joint_move(1,'RAISE CHEST FOR CLEARANCE',lambda:self.setlift(.696))
        self.joint_move(2,'LEFT ARM APPROACH',lambda:self.oven_pose(0,.055))
        self.segment(1,'SEAT LEFT CLAW ON PULL',callback=lambda u:self.oven_pose(0,.055*(1-u)))
        self.joint_move(.5,'GRASP OVEN PULL',lambda:self.left_jaw(.039))
        self.segment(7,'OPEN OVEN / LEFT ARM',callback=lambda u:self.oven_pose(np.pi/2*u))
        openbase=self.base().copy();opengrasp=self.d.qpos.copy()
        self.joint_move(.7,'RELEASE OVEN HANDLE',lambda:self.left_jaw(.15))
        self.forward();p=self.d.site('promo_left_claw').xpos.copy();R=self.d.site('promo_left_claw').xmat.reshape(3,3).copy()
        self.movebase(1,'LEFT ARM CLEAR',self.base()+[0,-.075,0])
        self.joint_move(1.3,'LEFT ARM DOWN',lambda:self.rest('left'))
        self.movebase(.6,'PICKUP REPOSITION',[oven[0],-.65,np.pi/2]);self.movebase(.5,'PICKUP TURN',[oven[0],-.65,np.pi])
        self.movebase(.6,'PICKUP ALIGN',[pickbase[0],-.65,np.pi]);self.movebase(.5,'FACE OVEN',[pickbase[0],-.65,np.pi/2]);self.movebase(1.5,'RIGHT GRIPPER DOCK',pickbase+[0,-.375,0])
        grip=self.rack+RACK_GRIP
        def empty_gripper_approach():
            # Select the insertion-compatible elbow/wrist branch while empty.
            ids=[self.q[n] for n in ['lift_extension_joint','lift_middle_joint']+RIGHT]
            self.d.qpos[ids]=[.5739448068,.2869724034,1.4328900047,-.4185391037,-1.7858304629,-1.2646296845,2.6449535289,1.447057387,.9903822274]
            self.right(grip+[0,-.375,.004],lift=True,attempts=12)
        self.joint_move(2,'RIGHT GRIPPER APPROACH',empty_gripper_approach)
        self.movebase(1.5,'ALIGN RACK HANDLE',pickbase)
        self.cart_right(1,'SEAT HANDLE GROOVES',grip,lift=True)
        closure=self.calibrated_closure();self.joint_move(.8,'GRASP LEICA RACK',lambda:self.jaws(closure));self.custody='right'
        self.cart_right(.8,'LIFT FROM OVEN SHELF',grip+[0,0,.004],lift=True)
        self.movebase(2,'EXTRACT LEICA RACK',[pickbase[0],-.35,np.pi/2])
        self.cart_right(1.5,'CARRY RACK',np.array([pickbase[0]+.275,-.11,1.12]),lift=True,attempts=8)
        self.movebase(1.5,'BACK INTO AISLE',[pickbase[0],-.85,np.pi/2]);self.movebase(.6,'TURN FOR OVEN CLOSING',[pickbase[0],-.85,0])
        self.movebase(1,'OVEN CLOSE ALIGN',[openbase[0],-.85,0]);self.movebase(.6,'FACE OVEN TO CLOSE',[openbase[0],-.85,np.pi/2]);self.movebase(1,'OVEN CLOSE DOCK',openbase+[0,-.075,0])
        self.left_jaw(.15)
        def open_pose_again():
            for n in LEFT+['lift_extension_joint','lift_middle_joint']:self.d.qpos[self.q[n]]=opengrasp[self.q[n]]
        self.joint_move(2,'LEFT ARM / OVEN REGRASP',open_pose_again)
        self.movebase(1,'SEAT LEFT CLAW TO CLOSE',openbase)
        self.joint_move(.5,'REGRASP OVEN HANDLE',lambda:self.left_jaw(.039))
        self.segment(5,'CLOSE OVEN / LEFT ARM',callback=lambda u:self.oven_pose(np.pi/2*(1-u)))
        self.joint_move(.5,'RELEASE CLOSED OVEN',lambda:self.left_jaw(.15))
        self.movebase(.8,'CLEAR CLOSED OVEN',self.base()+[0,-.075,0]);self.joint_move(1.3,'LEFT ARM DOWN',lambda:self.rest('left'))
        self.movebase(1,'OVEN DEPART',[oven[0],-.85,np.pi/2]);self.movebase(.6,'TURN TOWARD STAINER',[oven[0],-.85,0])
        self.movebase(3,'TRAVEL TO STAINER',[stbase[0],-.85,0]);self.movebase(.8,'FACE STAINER',[stbase[0],-.85,np.pi/2]);self.movebase(1.5,'STAINER DOCK',stbase)
        self.joint_move(2,'LEFT ARM / DRAWER HANDLE',lambda:self.reference('st_load',1.2))
        self.segment(6,'OPEN STAINER DRAWER',callback=lambda u:self.reference('st_load',1.2+24.1*u))
        self.joint_move(.7,'RELEASE DRAWER HANDLE',lambda:[self.d.qpos.__setitem__(self.q[n],v) for n,v in [('left_gripper_joint',.65),('left_gripper_idler_joint',-.65)]])
        self.forward();p=self.d.site('promo_left_claw').xpos.copy();R=self.d.site('promo_left_claw').xmat.reshape(3,3).copy()
        self.joint_move(1,'LEFT CLAW CLEAR',lambda:self.left(p+R[:,2]*.060,R,attempts=8))
        self.joint_move(1,'LEFT ARM REST',lambda:self.rest('left'))
        self.movebase(1,'LOAD REPOSITION',[stbase[0],-.5,np.pi/2]);self.movebase(.6,'LOAD TURN',[stbase[0],-.5,np.pi])
        self.movebase(1,'LOAD ALIGN',[loadbase[0],-.5,np.pi]);self.movebase(.6,'FACE CONTAINER',[loadbase[0],-.5,np.pi/2])
        target=np.array([-2.361045,.4171425+self.d.qpos[self.q['routine_a_j66_LOAD_DRAWER_OPEN_mm_0']],.934314])+RACK_GRIP
        # Loaded moves are Cartesian with a vertical tool/rack constraint,
        # never unconstrained interpolation between different wrist branches.
        self.cart_right(1.5,'PREALIGN RACK ABOVE VESSELS',target+[0,self.base()[1]-loadbase[1],.140],lift=True,attempts=12)
        self.movebase(1,'CONTAINER DOCK',loadbase);self.segment(.3,'ALIGN ABOVE STAINER CONTAINER')
        self.cart_right(2,'LOWER INTO STAINER CONTAINER',target,lift=True,attempts=12)
        # Stop opening before either finger reaches the actual tapered CAD wall.
        release=self.limited_release();self.custody='drawer';self.joint_move(.8,'RELEASE RACK IN CONTAINER',lambda:self.jaws(release))
        self.cart_right(1.5,'RIGHT GRIPPER CLEAR',target+[0,0,.100],lift=True,attempts=3)
        self.joint_move(.5,'OPEN JAWS ABOVE CONTAINER',lambda:self.jaws(0))
        self.movebase(1,'DRAWER CLOSE REPOSITION',[loadbase[0],-.5,np.pi/2]);self.joint_move(1.3,'RIGHT ARM DOWN',lambda:self.rest('right'));self.movebase(.6,'DRAWER CLOSE TURN',[loadbase[0],-.5,0])
        self.movebase(1,'DRAWER CLOSE ALIGN',[stbase[0],-.5,0]);self.movebase(.6,'FACE DRAWER',[stbase[0],-.5,np.pi/2]);self.movebase(1,'DRAWER CLOSE DOCK',stbase)
        self.joint_move(2,'LEFT HAND REGRASP DRAWER',lambda:self.reference('st_load',25.3))
        self.segment(6,'CLOSE STAINER / LEFT ARM',callback=lambda u:self.reference('st_load',25.3+24.3*u))
        self.joint_move(1.2,'LEFT ARM DOWN',lambda:self.rest('left'))
        machine=['routine_a_j70_ST_ARM_Z_mm_0','routine_a_j71_ST_ARM_SHOULDER_deg_0','routine_a_j72_ST_ARM_ELBOW_deg_0','routine_a_j73_ST_ARM_WRIST_deg_0']
        def machine_to(p):
            _,pe,re=ik(self.m,self.d,'promo_stainer_tcp',machine,p,np.eye(3),attempts=8,iterations=250)
            if pe<.001 and re<.01:pe,re=self.clearance.tighten('promo_stainer_tcp',machine,p,np.eye(3))
            if pe>.001 or re>.01:raise RuntimeError(('stainer target',p,pe,re))
        # Same fork-pin height relation as the original Leica CAD choreography:
        # st=252 mm, rack bottom=134.314 mm => 117.686 mm datum offset.
        pickup=self.rack+[0,0,.117686]
        self.forward();initial_tcp=self.d.site('promo_stainer_tcp').xpos.copy()
        approach=pickup+[0,.050,.133]
        self.segment(2,'STAINING RUN STARTS',callback=lambda u:machine_to(initial_tcp+(approach-initial_tcp)*u))
        self.segment(.6,'STAINER LOW APPROACH',callback=lambda u:machine_to(pickup+[0,.050,.133-.023*u]))
        self.segment(.6,'STAINER ALIGNS FOR PICKUP',callback=lambda u:machine_to(pickup+[0,.050*(1-u),.110]))
        self.forward();mp=self.d.site('promo_stainer_tcp').xpos.copy()
        self.segment(1.5,'STAINER ENGAGES RACK',callback=lambda u:machine_to(mp+(pickup-mp)*u))
        self.forward();R=self.d.site('promo_stainer_tcp').xmat.reshape(3,3);p=self.d.site('promo_stainer_tcp').xpos
        self.stainer_offset=R.T@(self.rack-p);self.stainer_rotation=R.T@self.rackR;self.custody='stainer'
        self.segment(2.2,'STAINER LIFTS RACK',callback=lambda u:machine_to(pickup+[0,0,.110*u]))
        self.segment(.6,'STAINER CLEARS FRONT LID',callback=lambda u:machine_to(pickup+[0,.050*u,.110]))
        self.segment(.6,'STAINER CLEARS VESSEL RIMS',callback=lambda u:machine_to(pickup+[0,.050,.110+.023*u]))
        from promo_geometry import BACK_RACK
        back_tcp=BACK_RACK+[0,0,.117686]
        def depart_and_place(u):
            if u<.65:machine_to((pickup+[0,.050,.133])*(1-u/.65)+(back_tcp+[0,0,.133])*(u/.65))
            else:machine_to(back_tcp+[0,0,.133*(1-(u-.65)/.35)])
            if u<.30:self.setbase(stbase+(np.array([stbase[0],-.85,np.pi/2])-stbase)*(u/.30))
            elif u<.45:self.setbase([stbase[0],-.85,np.pi/2*(1-(u-.30)/.15)])
            else:self.setbase([stbase[0]+(-.6-stbase[0])*(u-.45)/.55,-.85,0])
        self.segment(6,'NORI DEPARTS / STAINER PLACES RACK',callback=depart_and_place)
        self.custody='rear_bath';self.segment(1,'REAR CONTAINER / STAINING DWELL')
        self.segment(1,'FADE TO WHITE');self.segment(.5,'WHITE');self.segment(1,'LOGO FADE');self.segment(3,'LOGO')
        q=np.array([r['q'] for r in self.frames]);bad=[]
        for j in range(self.m.njnt):
            if self.m.jnt_limited[j] and self.m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE,mujoco.mjtJoint.mjJNT_SLIDE):
                values=q[:,self.m.jnt_qposadr[j]];lo,hi=self.m.jnt_range[j]
                if values.min()<lo-.006 or values.max()>hi+.006:bad.append(self.m.joint(j).name)
        report=dict(mode='STAGED SIMULATION CONCEPT',physical_success=False,duration_s=self.clock,fps=FPS,frames=len(q),events=self.events,joint_limit_violations=bad,max_ik_position_m=max(x['position_m'] for x in self.errors),max_ik_angle_rad=max(x['angle_rad'] for x in self.errors),oven_promo_position_m=[-5.13,.10,.8],base_scene_unchanged=True,source_access_recordings=['access_quincy_states.npz','access_st_load_states.npz'],source_special_stain='rail_loop_lab_trace.json',scope='Translated/retargeted recorded access paths; staged Cartesian IK rack transfer with rigid grip-frame attachment; wheel animation from chassis travel; recorded special-stain replay. No new contact, slip, collision, insertion, balance, force or hardware validation.')
        if bad:raise RuntimeError(('joint limits',bad))
        (OUT/'promo_clearances.json').write_text(json.dumps(self.spatial_checks,indent=2))
        report['scope']='Staged Cartesian IK and rigid grip-frame playback. Separate postflight reports check video-rate CAD geometry in the oven/stainer interaction corridor, container clearance, grasp alignment and placement. Individual slide sorting and native stainer-fork transfer are choreographed. Not contact-force, slip, balance, continuous-time or hardware validation.'
        report['sorter_max_ik_position_m']=float(np.max(self.sorting.errors,axis=0)[0])
        report['gripper_tilt_deg']=float(np.rad2deg(GRIP_TILT));report['closed_jaw_m']=self.closed_jaw;report['release_jaw_m']=self.release_jaw;report['drawer_extension_m']=.23495
        np.savez_compressed(OUT/'promo_states.npz',qpos=q,rack=np.array([r['rack'] for r in self.frames]),rack_quat=np.array([r['quat'] for r in self.frames]),sort_pos=np.array([r['sortpos'] for r in self.frames]),sort_quat=np.array([r['sortquat'] for r in self.frames]),label=np.array([r['label'] for r in self.frames]),custody=np.array([r['custody'] for r in self.frames]))
        (OUT/'promo_manifest.json').write_text(json.dumps(report,indent=2));return report

def camera(t,report,q,m):
    c=mujoco.MjvCamera();x=q[m.joint('base_x').qposadr[0]];y=q[m.joint('base_y').qposadr[0]]-1
    label=next((e['label'] for e in report['events'] if e['start']<=t<e['end']),report['events'][-1]['label'])
    c.azimuth=120;c.elevation=-25;c.distance=2.1;c.lookat[:]=[x,.2,1.04]
    if t<3:c.lookat[:]=[4,-.5,1.0];c.distance=7.2;c.elevation=-31;c.azimuth=112
    elif t<12:c.lookat[:]=[x+.1,-.35,1.03];c.distance=4.3;c.azimuth=110;c.elevation=-28
    elif ('STAINER' in label or 'CONTAINER' in label or 'DRAWER' in label) and x>-4:
        c.lookat[:]=[-2.39,.27,1.03];c.distance=1.9;c.azimuth=115;c.elevation=-31
    elif x<-4:
        c.lookat[:]=[-5.19,.12,1.05];c.distance=1.85;c.azimuth=115;c.elevation=-22
    if label in ('LEFT ARM APPROACH','SEAT LEFT CLAW ON PULL','GRASP OVEN PULL','OPEN OVEN / LEFT ARM','RELEASE OVEN HANDLE','LEFT ARM CLEAR','LEFT ARM DOWN','LEFT ARM / OVEN REGRASP','SEAT LEFT CLAW TO CLOSE','REGRASP OVEN HANDLE','CLOSE OVEN / LEFT ARM','RELEASE CLOSED OVEN','CLEAR CLOSED OVEN') and x<-4:
        c.lookat[:]=[-5.46,.07,1.04];c.distance=2.35;c.azimuth=88;c.elevation=-28
    if label in ('RIGHT GRIPPER APPROACH','ALIGN RACK HANDLE','SEAT HANDLE GROOVES','GRASP LEICA RACK','LIFT FROM OVEN SHELF','EXTRACT LEICA RACK'):
        c.lookat[:]=[-5.17,.16,1.04];c.distance=1.03;c.azimuth=98;c.elevation=-42
    if label in ('CARRY RACK','BACK INTO AISLE'):
        c.lookat[:]=[-5.17,-.14,1.05];c.distance=1.45;c.azimuth=28;c.elevation=-35
    if label in ('ALIGN ABOVE STAINER CONTAINER','LOWER INTO STAINER CONTAINER','RELEASE RACK IN CONTAINER','RIGHT GRIPPER CLEAR','OPEN JAWS ABOVE CONTAINER'):
        c.lookat[:]=[-2.39,.21,1.04];c.distance=1.4;c.azimuth=140;c.elevation=-36
    if 'TRAVEL TO STAINER'==label:c.lookat[:]=[x,-.20,1.0];c.distance=3.1;c.elevation=-27
    if 'RUN' in label or 'DEPARTS' in label or 'FADE' in label or label in ('WHITE','LOGO'):
        c.lookat[:]=[-2.46,.35,1.05];c.distance=2.85;c.elevation=-28;c.azimuth=115
    if label=='STAINING RUN STARTS' or label.startswith(('STAINER ENGAGES','STAINER LIFTS','STAINER TRANSFER')):
        c.lookat[:]=[-2.42,.55,1.11];c.distance=1.7;c.elevation=-35;c.azimuth=120
    if label=='REAR CONTAINER / STAINING DWELL':
        c.lookat[:]=[-2.63,.68,1.1];c.distance=1.75;c.elevation=-42;c.azimuth=112
    return c

def exact_grip_playback(m,d,states,f):
    """Preserve grasp frames after video-rate interpolation of joint angles."""
    kinematics(m,d);i=min(int(f),len(states['qpos'])-1)
    if str(states['custody'][i])=='right':
        s=d.site('nori_right_handle_groove');R=s.xmat.reshape(3,3)@GRIP_R.T;mid=m.body('B01_leica_rack').mocapid[0]
        d.mocap_pos[mid]=s.xpos-R@RACK_GRIP;d.mocap_quat[mid]=quat(R)
    from promo_sorting import ROT,GRIP
    elapsed=max(0,f/FPS-3);k=min(3,int(elapsed//16));u=min(16,elapsed-k*16)
    if 3<=u<10:
        s=d.site('sorter_2_slide_groove');R=s.xmat.reshape(3,3)@ROT.T;mid=m.body(f'promo_sort_slide_{k}').mocapid[0]
        d.mocap_pos[mid]=s.xpos-R@GRIP;d.mocap_quat[mid]=quat(R)
    kinematics(m,d)

def render(proof=False,short=False):
    import imageio_ffmpeg
    m=mujoco.MjModel.from_xml_path(str(MODEL));d=mujoco.MjData(m);states=np.load(OUT/'promo_states.npz');report=json.loads((OUT/'promo_manifest.json').read_text())
    # Keep the 5.5-second white/logo ending intact; tighten the action to 54.5s.
    states=dict(states)
    outfps=30 if short else FPS;duration=60. if short else len(states['qpos'])/FPS
    count=round(duration*outfps);action_end=next(e['start'] for e in report['events'] if e['label']=='FADE TO WHITE')
    def source_time(t):return (t*action_end/54.5 if t<54.5 else action_end+t-54.5) if short else t
    def output_time(t):return (t*54.5/action_end if t<action_end else 54.5+t-action_end) if short else t
    stem='ExHist-Labs-Promo-60s' if short else 'ExHist-Labs-Promo-Concept'
    def interp_array(a,f):
        lo=min(int(f),len(a)-1);hi=min(lo+1,len(a)-1);u=np.clip(f-lo,0,1);return (1-u)*a[lo]+u*a[hi]
    m.light_castshadow[:]=0;m.light_diffuse[:]=.15;m.light_ambient[:]=0;m.vis.headlight.ambient[:]=.43;m.vis.headlight.diffuse[:]=.55
    opt=mujoco.MjvOption();opt.sitegroup[:]=0;opt.geomgroup[3:]=0
    trace=json.loads((OUT/'special-stain-trace.json').read_text());times=np.array([r['t'] for r in trace])
    lm=mujoco.MjModel.from_xml_path(str(ROOT/'rail_loop_reference/station.xml'));layout=json.loads((ROOT/'rail_loop_reference/rail_loop_layout.json').read_text());R=np.array(layout['world_rotation']);tr=np.array(layout['world_translation'])
    mapping=[(m.joint(LOOP_PREFIX+lm.joint(j).name).qposadr[0],lm.jnt_qposadr[j]) for j in range(lm.njnt) if lm.jnt_type[j]!=mujoco.mjtJoint.mjJNT_FREE]
    rackadr=lm.joint('rack_free').qposadr[0];loopmid=m.body(LOOP_PREFIX+'rack').mocapid[0];mid=m.body('B01_leica_rack').mocapid[0]
    sortmid=[m.body(f'promo_sort_slide_{k}').mocapid[0] for k in range(4)]
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',19);bold=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',30)
    logo=Image.open(OUT/'exhist-logo.png').convert('RGB');logo.thumbnail((720,720),Image.Resampling.LANCZOS);endcard=Image.new('RGB',(1280,720),'white');endcard.paste(logo,((1280-logo.width)//2,(720-logo.height)//2))
    writer=None
    if not proof:
        writer=imageio_ffmpeg.write_frames(str(OUT/(stem+'.mp4')),(1280,720),fps=outfps,codec='libx264',quality=8,pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=2,output_params=['-preset','fast','-movflags','+faststart']);writer.send(None)
    selected={int(output_time(e['start']+.7*(e['end']-e['start']))*outfps) for e in report['events']};thumbs=[];started=time.monotonic()
    with mujoco.Renderer(m,height=720,width=1280) as renderer:
        for i in range(count):
            if proof and i not in selected:continue
            ot=i/outfps;t=source_time(ot);f=t*FPS;label=str(states['label'][min(int(f),len(states['label'])-1)])
            q=interp_array(states['qpos'],f);d.qpos[:]=q;d.mocap_pos[:]=[0,0,-10];d.mocap_pos[mid]=interp_array(states['rack'],f)
            rq=interp_array(states['rack_quat'],f);d.mocap_quat[mid]=rq/np.linalg.norm(rq)
            d.mocap_pos[sortmid]=interp_array(states['sort_pos'],f)
            sq=interp_array(states['sort_quat'],f);d.mocap_quat[sortmid]=sq/np.linalg.norm(sq,axis=1,keepdims=True)
            lt=12+t*1.3;hi=min(np.searchsorted(times,lt),len(times)-1);lo=max(0,hi-1);u=0 if hi==lo else np.clip((lt-times[lo])/(times[hi]-times[lo]),0,1)
            lq=(1-u)*np.array(trace[lo]['qpos'])+u*np.array(trace[hi]['qpos']);lq[rackadr+3:rackadr+7]/=np.linalg.norm(lq[rackadr+3:rackadr+7])
            for target,source in mapping:d.qpos[target]=lq[source]
            d.mocap_pos[loopmid]=R@lq[rackadr:rackadr+3]+tr;dq=np.zeros(4);mujoco.mju_mulQuat(dq,quat(R),lq[rackadr+3:rackadr+7]);d.mocap_quat[loopmid]=dq
            exact_grip_playback(m,d,states,f);renderer.update_scene(d,camera(t,report,q,m),scene_option=opt);im=Image.fromarray(renderer.render())
            draw=ImageDraw.Draw(im)
            if t<3:draw.text((38,37),'ExHist Labs',font=bold,fill='white',stroke_width=1,stroke_fill='#163444')
            draw.text((1015,685),'SIMULATION CONCEPT',font=font,fill='#294353',stroke_width=1,stroke_fill='white')
            if label=='STAINING RUN STARTS' or label.startswith(('STAINER ENGAGES','STAINER LIFTS','STAINER TRANSFER')):draw.rounded_rectangle((34,634,306,681),radius=10,fill='#0b5548');draw.text((49,646),'ST5020  |  RUNNING',font=font,fill='white')
            e=next(e for e in report['events'] if e['start']-.001<=t<e['end']+.001)
            u=np.clip((t-e['start'])/(e['end']-e['start']),0,1)
            if label=='FADE TO WHITE':im=Image.blend(im,Image.new('RGB',im.size,'white'),float(smooth(u)))
            elif label=='WHITE':im=Image.new('RGB',im.size,'white')
            elif label=='LOGO FADE':im=Image.blend(Image.new('RGB',im.size,'white'),endcard,float(smooth(u)))
            elif label=='LOGO':im=endcard.copy()
            if writer:writer.send(np.asarray(im))
            if i in selected:
                path=OUT/f'{"sixty-" if short else ""}frame-{i:04d}.jpg';im.save(path,quality=90);thumb=im.resize((384,216));ImageDraw.Draw(thumb).text((5,5),f'{ot:.1f}s {label}',fill='white',stroke_width=1,stroke_fill='black');thumbs.append(thumb)
            if i%(outfps*5)==0:print('Render',round(ot,1),'/',duration,'wall',round(time.monotonic()-started,1),flush=True)
    if writer:writer.close()
    sheet=Image.new('RGB',(1536,216*math.ceil(len(thumbs)/4)),'#122b3b')
    for i,im in enumerate(thumbs):sheet.paste(im,((i%4)*384,(i//4)*216))
    sheet.save(OUT/f'{"sixty-" if short else ""}promo-contact-sheet.jpg',quality=91)
    if short:
        from access_suite import source_hashes
        current=source_hashes()
        metadata=dict(duration_s=duration,fps=outfps,frames=count,source_states_sha256=sha(OUT/'promo_states.npz'),scene_sha256=sha(MODEL),logo_sha256=sha(OUT/'exhist-logo.png'),action_seconds=54.5,ending_seconds=5.5,physical_success=False,scope=report['scope'],historical_access_source_mismatches={k:[n for n,h in json.loads((ROOT/f'access_{k}_validation.json').read_text())['source_sha256'].items() if current.get(n)!=h] for k in ('quincy','st_load')},events=[dict(start=output_time(e['start']),end=output_time(e['end']),label=e['label']) for e in report['events']])
        (OUT/'promo_60_manifest.json').write_text(json.dumps(metadata,indent=2))
    print('Finished',count,'frames',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--plan',action='store_true');p.add_argument('--proof',action='store_true');p.add_argument('--render',action='store_true');p.add_argument('--sixty',action='store_true');a=p.parse_args()
    if a.plan:build();print(json.dumps(Promo().plan(),indent=2))
    if a.proof or a.render:render(proof=a.proof,short=a.sixty)
