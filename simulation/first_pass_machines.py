"""Actual v3 Leica film choreography embedded in the lab's MuJoCo renderer.

This intentionally uses explicit kinematic playback, not fake contact forces.
The legacy CAD joints remain available in the separate readiness/inspection model.
"""
import json,sys,math,re
import numpy as np
import mujoco
from run_lab import ROOT

class FilmMachines:
    def __init__(self,m,d):
        self.m=m;self.d=d;self.meta=json.loads((ROOT/'first_pass_assets.json').read_text())
        sys.path.insert(0,self.meta['film_dir'])
        import film_v3,film_v2,diagnostic_frames,cv_motion
        from types import SimpleNamespace
        # The omitted cosmetic flexible hose was >95% of the offline renderer's
        # CPU time. Do not calculate its arc-length solver for a rigid-only layer.
        diagnostic_frames.hose=SimpleNamespace(points=lambda *args: [])
        self.f=film_v2;self.cv=cv_motion
        diagnostic_frames.HOSE_REFERENCE=self.meta['hose_reference']
        all_segments=film_v3.timeline()
        self.cycles={};self.ends={}
        for count in (6,8,10,12):
            selected=set(cv_motion.SLOTS[:count])
            # Input has already been sorted into a rack; do not re-play operator slide loading.
            input_segment=[s for s in all_segments if s['mode']=='load'][-1:]
            self.cycles[count]={
                'stain':input_segment+[s for s in all_segments if s['mode'] in ('approach','stain','transfer')],
                'coverslip':[s for s in all_segments if (s['mode']=='cv' and s['states'][-1]['slot'] in selected) or s['mode']=='empty'],
                'cure':[dict(mode='output',title='CURE / OUTPUT READY',detail='Covered slides retained in indexed magazine; empty rack returned separately',frames=1,states=[dict(removal=0.)])],
            }
            self.ends[count]=cv_motion.cycle(cv_motion.SLOTS[:count])[-1][1]
        self.groups=[]
        for row in self.meta['groups']:
            t=np.array(row['item']['matrix']).reshape(4,4);t[:3,3]*=10
            self.groups.append((row,t,{s:int(m.body_mocapid[m.body(f'{s}_film_{row["index"]}').id]) for s in self.meta['bases']}))
        self.last={};self.labels={};self.cutaway=False;self.seen=set();self.rack_trace=[];self.slide_counts={}
        self.alphas=m.geom_rgba[:,3].copy()
        self.geom_ids={s:[[m.geom(n).id for n in row['geom_names'][s]] for row,_,_ in self.groups] for s in self.meta['bases']}

    def sample(self,count,phase,fraction):
        segments=self.cycles[count][phase];total=sum(s['frames'] for s in segments)
        at=np.clip(fraction,0,1)*max(0,total-1)
        for s in segments:
            if at<s['frames']:break
            at-=s['frames']
        states=s['states'];u=min(1,at/max(1,s['frames']-1))*(len(states)-1)
        lo=int(u);hi=min(lo+1,len(states)-1)
        s=dict(s,detail=re.sub(r'(\d{2})/28',lambda match:match.group(1)+f'/{count:02d}',s['detail']))
        return s,self.f.interpolate(states[lo],states[hi],u-lo)

    def update(self,lab):
        for station,base in self.meta['bases'].items():
            jobs=[j for j in lab.jobs if j.holds==station]
            job=jobs[0] if jobs else None
            if job:
                phase=job.route[job.index].name
                if phase not in self.cycles[len(job.slides)]:phase='cure'
                progress=1-job.remaining/job.route[job.index].seconds if job.state=='PROCESSING' else 1.
                self.last[station]=(job.id,len(job.slides),phase,progress)
            ident,count,phase,progress=self.last.get(station,('',8,'stain',0.))
            active=bool(job)
            tr=lab.transport
            if tr and tr['source']==station and tr['elapsed']>=tr['approach']+.7:active=False
            segment,state=self.sample(count,phase,progress)
            if active and segment['mode']=='cv':
                self.slide_counts[job.id]=max(self.slide_counts.get(job.id,0),len(state.get('done',())))
            self.labels[station]=(ident+' '+segment['detail']) if active else 'IDLE / empty-rack return bay'
            self.f.FINAL_CV=self.ends[count]
            resolve=self.f.film_resolver(segment['mode'],state,None)
            selected=set(self.cv.SLOTS[:count]);offset=np.asarray(base)-np.asarray(self.meta['offset_m'])
            for index,(row,t,ids) in enumerate(self.groups):
                item=row['item'];co=item['component'];value=resolve(item,t)
                if co.startswith(('CYCLE_LEICA_SLIDE_','CYCLE_COVERSLIP_')) and int(co[-2:]) not in selected:value=None
                if not ident and (co=='20 RACK L33 - liftable' or co.startswith('CYCLE_LEICA_SLIDE_')):value=None
                # Once Nori collects output, don't leave a second filled magazine behind.
                if ident and not active and (co.startswith(('CYCLE_LEICA_SLIDE_','CYCLE_COVERSLIP_')) or co=='CV5030_Output_Magazine_30'):value=None
                shell=('transparent_hood' in co or 'FIXED LID' in co or '02_OUTER_ENCLOSURE' in item['path'] or co=='14_INTERNAL_SUPPORT_STRUCTURE')
                if self.cutaway and shell:value=None
                elif shell and value is None:value=t
                mid=ids[station]
                if value is None:self.d.mocap_pos[mid]=[0,0,-10];continue
                self.d.mocap_pos[mid]=value[:3,3]*.001+offset
                mujoco.mju_mat2Quat(self.d.mocap_quat[mid],np.ascontiguousarray(value[:3,:3]).ravel())
                if co=='20 RACK L33 - liftable' and active:
                    key=(job.id,segment['mode'],segment['detail'])
                    if key not in self.seen:
                        self.seen.add(key);self.rack_trace.append(dict(batch=job.id,station=station,mode=segment['mode'],detail=segment['detail'],time=round(lab.time,3),rack_m=self.d.mocap_pos[mid].tolist(),owner=state.get('rack',state.get('owner','stainer_arm'))))
            for device in ('ST5020','CV5030'):
                busy=active and ((device=='ST5020' and phase=='stain') or (device=='CV5030' and phase=='coverslip'))
                self.m.site_rgba[self.m.site(station+'_'+device+'_activity').id]=[1,.65,.05,1] if busy else [.1,.75,.5,1]

    def toggle_xray(self):self.cutaway=not self.cutaway

    def status(self):return '\n'.join(s+': '+v for s,v in self.labels.items())
