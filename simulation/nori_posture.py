"""Shared startup posture for lab inspection, not a runtime lift override.

Contact controllers own their task-specific lift trajectories. Never change
lift height after grasping just to improve the appearance of a camera view.
"""
import numpy as np

BENCH_VIEW_LIFT_M=.55
ARM_SUFFIXES=('shoulder_pitch','shoulder_roll','bicep_yaw','elbow_pitch','forearm_yaw','wrist_pitch','wrist_roll')

def initialize_bench_view(m,d,relax_arms=True):
    if d.time!=0.:raise ValueError('Startup posture cannot override a running simulation')
    values={'lift_extension_joint':BENCH_VIEW_LIFT_M,'lift_middle_joint':BENCH_VIEW_LIFT_M/2}
    if relax_arms:
        for side in ('left','right'):
            values.update({side+'_'+s+'_joint':v for s,v in zip(ARM_SUFFIXES,[0,1.25 if side=='left' else -1.25,0,-.45,0,0,0])})
    for name,value in values.items():
        j=m.joint(name)
        if m.jnt_limited[j.id] and not j.range[0]<=value<=j.range[1]:raise ValueError('Startup posture exceeds CAD joint range: '+name)
    for name,value in values.items():d.qpos[m.joint(name).qposadr[0]]=value

def height_measurements(m,d,bench='sorting_top'):
    g=m.geom(bench);top=float(d.geom(bench).xpos[2]+g.size[2])
    shoulder=float(d.body('left_shoulder_pitch_link').xpos[2])
    return dict(bench_top_m=top,shoulder_height_m=shoulder,shoulder_above_bench_m=shoulder-top,
        lift_m=float(d.qpos[m.joint('lift_extension_joint').qposadr[0]]))
