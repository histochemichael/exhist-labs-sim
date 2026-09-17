"""Folder assembly and finishing state machine for the kinematic first pass.

Uses the user's actual two-flap cardboard CAD. Creases, robot edge grasps and
closing forces remain simulation assumptions, not hardware/contact validation.
"""
import json,math
import numpy as np
from workflow import Lab,Step
from build_scene import ROOT,el,vec

FOLDER_CENTERS=(2.39,3.31)
FOLDER_Y=.45
FOLDER_Z=.8
BENCH_X=(-3.52,-3.04,-2.56,-2.08)
BENCH_Y=-3.79
SLOT_POINTS=np.array([[x-.104,.0265+.032*r-.1705,.0041] for x in (.0535,.1545) for r in range(10)])
FLAT_ROTATION=np.array([[0,0,-1],[0,1,0],[1,0,0]],float)

class FolderLab(Lab):
    def __init__(self,stations):
        super().__init__(stations)
        self.x['finished_bench']=-2.8;self.capacity['finished_bench']=4;self.occupancy['finished_bench']={}
        for job in self.jobs:
            for step in job.route:
                if step.name=='package':step.seconds=7*len(job.slides)
            job.route.extend([Step('close_folder',('sendout',),18),Step('finished',('finished_bench',),.5)])

    @property
    def robot_busy(self):
        return any(j.index<len(j.route) and j.route[j.index].name in ('package','close_folder') and j.state in ('READY','PROCESSING') and j.holds=='sendout' for j in self.jobs)

    def start(self,job,station,slot):
        if job.route[job.index].name in ('package','close_folder'):
            if (self.transport and self.transport['job']!=job.id) or any(j is not job and j.state=='PROCESSING' and j.route[j.index].name in ('package','close_folder') for j in self.jobs):return
        if job.route[job.index].name=='close_folder':
            if self.transport or any(j is not job and j.state=='PROCESSING' and j.route[j.index].name=='close_folder' for j in self.jobs):return
            assert job.folder_loaded==len(job.slides),'Cannot close an incomplete folder'
            approach=max(6.,abs(self.robot_x-FOLDER_CENTERS[slot])/.65+2)
            job.route[job.index].seconds=approach+12
        super().start(job,station,slot)

    def reserve(self,job,station):
        if station=='finished_bench':
            assert job.folder_closed and job.folder_loaded==len(job.slides)
            slot=self.jobs.index(job);self.occupancy[station][slot]=job.id;return slot
        return super().reserve(job,station)

    def finish(self,job):
        step=job.route[job.index].name
        if step=='package':
            job.folder_loaded=len(job.slides)
            self.event(job,'folder_filled',slide_ids=job.slides.copy(),assigned_slots=len(job.slides),capacity=20)
        elif step=='close_folder':
            assert job.folder_loaded==len(job.slides)
            job.folder_closed=True;self.event(job,'folder_closed',robot='nori',flaps=['L','R'])
        elif step=='finished':
            assert job.folder_closed
            self.event(job,'closed_folder_placed',station='finished_bench',slot=job.slot,slide_ids=job.slides.copy())
        super().finish(job)

    def tick(self,dt):
        super().tick(dt)
        for job in self.jobs:
            if job.state=='PROCESSING' and job.route[job.index].name=='package':
                elapsed=job.route[job.index].seconds-job.remaining
                job.folder_loaded=min(len(job.slides),int(elapsed/7+.22))

def add_folders(root):
    source=json.loads((ROOT/'assets/slide_folder.json').read_text())
    assets=root.find('asset');world=root.find('worldbody')
    meshmap={}
    for index,part in enumerate(source['parts']):
        name=f'closing_folder_{index}';path=ROOT/'assets'/(name+'.obj')
        vertices=np.asarray(part['vertices']).reshape(-1,3)-[.104,.1705,0]
        with path.open('w') as f:
            f.writelines('v '+vec(v)+'\n' for v in vertices)
            f.writelines('f '+' '.join(str(int(i)+1) for i in face)+'\n' for face in np.asarray(part['triangles']).reshape(-1,3))
        el(assets,'mesh',name=name,file='assets/'+path.name,inertia='shell')
        key='L' if 'flap L' in part['name'] else 'R' if 'flap R' in part['name'] else 'tray'
        meshmap[key]=(name,part['rgba'])
    for batch in range(1,5):
        prefix=f'B{batch:02d}';body=world.find(f"body[@name='{prefix}_slide_folder']")
        for child in list(body):body.remove(child)
        name,color=meshmap['tray'];el(body,'geom',name=prefix+'_folder_tray',mesh=name,type='mesh',rgba=vec(color),contype='0',conaffinity='0')
        for side,x in [('L',-.104),('R',.104)]:
            hinge=el(body,'body',name=prefix+'_folder_flap_'+side,pos=vec([x,0,.0081]))
            el(hinge,'inertial',mass='.015',pos='0 0 0',diaginertia='.0001 .0001 .0001')
            el(hinge,'joint',name=prefix+'_folder_hinge_'+side,type='hinge',axis='0 1 0',range=vec([0,math.pi] if side=='L' else [-math.pi,0]),damping='.02')
            name,color=meshmap[side];el(hinge,'geom',name=prefix+'_folder_flap_geom_'+side,type='mesh',mesh=name,pos=vec([-x,0,-.0081]),rgba=vec(color),contype='0',conaffinity='0')
        for k,point in enumerate(SLOT_POINTS):
            slide=el(body,'body',name=f'{prefix}_folder_slot_{k+1}',pos=vec(point))
            el(slide,'geom',name=f'{prefix}_folder_slide_{k+1}',type='box',size='.0375 .012475 .0005',rgba='.55 .83 .88 .8',contype='0',conaffinity='0')
            el(slide,'geom',name=f'{prefix}_folder_label_{k+1}',type='box',pos='-.029 0 .00055',size='.0085 .0124 .0001',rgba='.96 .92 .75 1',contype='0',conaffinity='0')
        moving=el(world,'body',name=prefix+'_folder_loading_slide',mocap='true',pos='0 0 -10')
        el(moving,'geom',type='box',size='.0375 .012475 .0005',rgba='.55 .83 .88 .9',contype='0',conaffinity='0')
        el(moving,'geom',type='box',pos='-.029 0 .00055',size='.0085 .0124 .0001',rgba='.96 .92 .75 1',contype='0',conaffinity='0')
    # Repurpose the existing General Work bench in this review copy only.
    room=world.find("body[@name='lab_room']")
    # Keep the full open folder clear of the rear robot rail and block-QC samples.
    for body in world.findall('body'):
        name=body.get('name','')
        if name.startswith('qc_cassette_'):
            index=int(name.rsplit('_',1)[1])-1;position=list(map(float,body.get('pos').split()))
            position[1]=.02+(index//2)*.055;body.set('pos',vec(position))
    for g in list(room):
        if g.get('name','').startswith('lab_workbay_1_tray'):room.remove(g)
    from PIL import Image,ImageDraw,ImageFont
    image=Image.new('RGB',(1024,128),'#17404b');draw=ImageDraw.Draw(image)
    draw.text((512,64),'FINISHED SLIDE FOLDERS',anchor='mm',fill='white',font=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',48))
    image.save(ROOT/'assets/finished_folder_bench.png')
    el(assets,'texture',name='finished_folders_tex',type='2d',file='assets/finished_folder_bench.png')
    el(assets,'material',name='finished_folders_mat',texture='finished_folders_tex',emission='.3')
    el(room,'geom',name='finished_folders_sign',type='plane',pos='-2.8 -3.618 .68',size='.85 .08 .001',euler=vec([math.pi/2,0,math.pi]),material='finished_folders_mat',contype='0',conaffinity='0')
    (ROOT/'folder_finish_design.json').write_text(json.dumps(dict(source=source['document'],slots=SLOT_POINTS.tolist(),flaps={'L':[0,math.pi],'R':[-math.pi,0]},bench={'name':'finished_bench','existing_bench':'GENERAL WORK','x':BENCH_X,'y':BENCH_Y,'top_m':.83},scope='Actual CAD panels; provisional passive crease axes, closing contact and folder edge grip. Kinematic review only.'),indent=2))

class FolderMotion:
    def __init__(self,scene):
        import mujoco
        from first_pass import ik,JOINTS,FOLDER_ROTATION
        self.s=scene;self.approaches={};self.load_plans={};self.close_plans={};self.plan_errors=[]
        m=scene.m;d=mujoco.MjData(m);d.qpos[:]=scene.d.qpos
        # Nori indexes between staging and each folder slot. Each docking leg
        # turns/drives in the aisle; the base does not translate sideways.
        for label,z in [('source',.8026),('source_up',.8426),('place',.8046),('place_up',.8446)]:
            q,pe,re=ik(m,d,'nori_right_slide_groove',JOINTS,[.2,-.8,z],FOLDER_ROTATION)
            self.plan_errors.append(dict(robot='nori_slide_'+label,position_m=pe,rotation_rad=re))
            if pe>.004 or re>.04:raise RuntimeError('Nori folder slide pose unreachable: '+str((label,pe,re)))
            self.load_plans[label]=q
        print('Nori folder slot-placement poses ready',flush=True)
        # Nori keeps a fixed base while it follows each flap edge through the crease arc.
        d.qpos[:]=scene.d.qpos;d.qpos[scene.q['base_x']]=0;d.qpos[scene.q['base_y']]=0;d.qpos[scene.q['base_yaw']]=math.pi/2
        for side,sign in [('L',-1),('R',1)]:
            poses=[]
            for angle in np.linspace(0,math.pi,41):
                target=np.array([.2,-.70,FOLDER_Z])+[sign*(.104+.075*math.cos(angle)),-.13,.0081+.075*math.sin(angle)]
                q,pe,re=ik(m,d,'nori_right_handle_groove',JOINTS,target,attempts=1)
                if pe>.004:q,pe,re=ik(m,d,'nori_right_handle_groove',JOINTS,target,attempts=4)
                self.plan_errors.append(dict(robot='nori_flap_'+side,position_m=pe,rotation_rad=None))
                if pe>.004:raise RuntimeError('Folder closing arc unreachable: '+str((side,angle,pe)))
                poses.append(q)
            self.close_plans[side]=np.array(poses)
        (ROOT/'folder_motion_plan.json').write_text(json.dumps(dict(poses=self.plan_errors,scope='Kinematic IK poses, not contact/force validation'),indent=2))

    def update(self):
        import mujoco
        from first_pass import smooth,kinematics,FOLDER_ROTATION
        s=self.s;m=s.m;d=s.d;lab=s.lab
        for job in lab.jobs:
            phase=job.route[job.index].name if job.index<len(job.route) else 'finished'
            count=job.folder_loaded
            for i in range(20):
                for tag,alpha in [('slide',.85),('label',1.)]:
                    m.geom_rgba[m.geom(f'{job.id}_folder_{tag}_{i+1}').id,3]=alpha if i<count else 0.
            d.qpos[s.q[job.id+'_folder_hinge_L']]=math.pi if job.folder_closed else 0.
            d.qpos[s.q[job.id+'_folder_hinge_R']]=-math.pi if job.folder_closed else 0.
            if phase=='package' and job.state=='PROCESSING':
                center=np.array([FOLDER_CENTERS[job.slot],FOLDER_Y,FOLDER_Z]);s.put(job,center,kind='slide_folder')
                self.load(job,center)
            if phase=='close_folder' and job.state=='PROCESSING':self.close(job)

    def base(self,pose):
        s=self.s;d=s.d;s.robot_pose=pose
        d.qpos[s.q['base_x']]=pose[0];d.qpos[s.q['base_y']]=pose[1]+1;d.qpos[s.q['base_yaw']]=pose[2]
        delta=pose-s.previous_pose;forward=np.dot(delta[:2],[math.cos(pose[2]),math.sin(pose[2])])
        for name,sign in [('left_wheel_joint',-1),('right_wheel_joint',1)]:d.qpos[s.q[name]]+=(forward+sign*.145*delta[2])/.075
        s.previous_pose=pose.copy();s.rx=pose[0];s.lab.robot_x=s.rx

    def load(self,job,center):
        import mujoco
        from first_pass import smooth,kinematics,FOLDER_ROTATION
        s=self.s;d=s.d;m=s.m
        elapsed=job.route[job.index].seconds-job.remaining;index=min(len(job.slides)-1,int(elapsed/7));u=(elapsed/7)%1
        source=center+[.275,.07,.0026];target=center+SLOT_POINTS[index]+[-.029,0,.0005]
        a=s.dock(source);b=s.dock(target);key=job.id+'_load_'+str(index)
        if key not in self.approaches:self.approaches[key]=(s.robot_pose.copy(),d.qpos[s.qa].copy())
        start,start_q=self.approaches[key];p=self.load_plans
        if u<.15:pose=s.route(start,a,u/.15);q=(1-smooth(u/.15))*start_q+smooth(u/.15)*p['source_up']
        elif u<.25:pose=a;q=(1-smooth((u-.15)/.10))*p['source_up']+smooth((u-.15)/.10)*p['source']
        elif u<.35:pose=a;q=(1-smooth((u-.25)/.10))*p['source']+smooth((u-.25)/.10)*p['source_up']
        elif u<.65:pose=s.route(a,b,(u-.35)/.30);q=(1-smooth((u-.35)/.30))*p['source_up']+smooth((u-.35)/.30)*p['place_up']
        elif u<.78:pose=b;q=(1-smooth((u-.65)/.13))*p['place_up']+smooth((u-.65)/.13)*p['place']
        else:pose=b;q=(1-smooth((u-.78)/.22))*p['place']+smooth((u-.78)/.22)*p['place_up']
        self.base(pose);d.qpos[s.qa]=q
        jaw=-.025 if .25<=u<.78 else 0.
        for side in ('a','b'):d.qpos[s.q['lehisto_jaw_'+side+'_slide']]=jaw
        d.qpos[s.q['lehisto_pinion_joint']]=jaw/.0072;kinematics(m,d)
        if u<.78:
            mid=m.body_mocapid[m.body(job.id+'_folder_loading_slide').id]
            if u<.25:d.mocap_pos[mid]=source-[-.029,0,.0005];d.mocap_quat[mid]=[1,0,0,0]
            else:
                sid=m.site('nori_right_slide_groove').id;rotation=d.site_xmat[sid].reshape(3,3)@FOLDER_ROTATION.T
                d.mocap_pos[mid]=d.site_xpos[sid]-rotation@np.array([-.029,0,.0005])
                mujoco.mju_mat2Quat(d.mocap_quat[mid],rotation.ravel())

    def close(self,job):
        from first_pass import smooth,kinematics
        s=self.s;d=s.d
        if job.id not in self.approaches:self.approaches[job.id]=(s.robot_pose.copy(),d.qpos[s.qa].copy())
        start,start_q=self.approaches[job.id]
        duration=job.route[job.index].seconds;elapsed=duration-job.remaining;approach=duration-12
        dock=np.array([FOLDER_CENTERS[job.slot]-.2,FOLDER_Y-.3,math.pi/2])
        t=elapsed-approach;left=self.close_plans['L'];right=self.close_plans['R'];carry=s.plans['leica_rack'][2]
        def sample(poses,u):
            f=smooth(u)*40;i=min(int(f),39);return (1-(f-i))*poses[i]+(f-i)*poses[i+1]
        if t<0:s.robot_pose=s.route(start,dock,elapsed/approach);q=(1-smooth(elapsed/approach))*start_q+smooth(elapsed/approach)*carry
        else:
            s.robot_pose=dock
            if t<1:q=(1-smooth(t))*carry+smooth(t)*left[0]
            elif t<4:q=sample(left,(t-1)/3)
            elif t<5:q=left[-1]
            elif t<6:q=(1-smooth(t-5))*left[-1]+smooth(t-5)*right[0]
            elif t<9:q=sample(right,(t-6)/3)
            else:q=(1-smooth((t-9)/3))*right[-1]+smooth((t-9)/3)*carry
        d.qpos[s.qa]=q
        d.qpos[s.q[job.id+'_folder_hinge_L']]=math.pi*smooth((t-1)/3)
        d.qpos[s.q[job.id+'_folder_hinge_R']]=-math.pi*smooth((t-6)/3)
        d.qpos[s.q['base_x']]=s.robot_pose[0];d.qpos[s.q['base_y']]=s.robot_pose[1]+1;d.qpos[s.q['base_yaw']]=s.robot_pose[2]
        delta=s.robot_pose-s.previous_pose;forward=np.dot(delta[:2],[math.cos(s.robot_pose[2]),math.sin(s.robot_pose[2])])
        for name,sign in [('left_wheel_joint',-1),('right_wheel_joint',1)]:d.qpos[s.q[name]]+=(forward+sign*.145*delta[2])/.075
        s.previous_pose=s.robot_pose.copy();s.rx=s.robot_pose[0];s.lab.robot_x=s.rx
        for side in ('a','b'):d.qpos[s.q['lehisto_jaw_'+side+'_slide']]=-.025
        d.qpos[s.q['lehisto_pinion_joint']]=-.025/.0072
        kinematics(s.m,d)
