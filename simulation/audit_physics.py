"""Reproducible measurements of current CAD/physics readiness; read-only sources."""
import json,struct,math
from pathlib import Path
import numpy as np
import mujoco
from operate_lab import Scene,ROOT
from contact_bench import SOURCE

def main():
    s=Scene();m=s.m;d=s.d;mujoco.mj_forward(m,d)
    driven={m.joint(int(m.actuator_trnid[i,0])).name for i in range(m.nu)}
    force_rows=[]
    for name in ("lift_extension_joint","right_shoulder_roll_joint"):
        ji=m.joint(name).id
        force_rows.append(dict(joint=name,unit="N" if m.jnt_type[ji]==mujoco.mjtJoint.mjJNT_SLIDE else "Nm",
            gravity_generalized_force=float(d.qfrc_bias[m.jnt_dofadr[ji]]),imported_effort_range=m.jnt_actfrcrange[ji].tolist(),
            actuator_present=name in driven,pose="exported arm pose, lift .25 m / middle .125 m"))
    src=json.loads((ROOT/"assets/leica_rack_handled.json").read_text())
    slides=[np.asarray(p["vertices"]).reshape(-1,3) for p in src["parts"] if p["name"].startswith("LEICA_SLIDE_SLOT_")]
    dims=np.median(np.array([v.max(0)-v.min(0) for v in slides]),axis=0)
    centers=sorted((v.max(0)[1]+v.min(0)[1])/2 for v in slides);pitch=float(np.median(np.diff(centers)))
    raw=(SOURCE.parent/"lehisto_meshes/lehisto_part_14.stl").read_bytes();count=struct.unpack_from("<I",raw,80)[0]
    v=np.array([struct.unpack_from("<9f",raw,84+i*50+12) for i in range(count)]).reshape(-1,3)
    tips=v[v[:,1]<-.10];open_gap=2*float(tips[:,0].min());stroke=.0355
    # CAD-assigned inertias, not a measured physical mass.
    wrist=m.body("right_wrist_roll_link").id
    ids=[]
    for bi in range(m.nbody):
        parent=bi
        while parent not in (0,wrist):parent=int(m.body_parentid[parent])
        if parent==wrist:ids.append(bi)
    report=dict(full_lab_physics_ready=False,default_viewer="PHYSICS_READINESS_HOLD",
        full_lab_model=dict(joints=m.njnt,actuators=m.nu,actuated_joints=sorted(driven),
            mocap_carriers=m.nmocap,passive_cad_doors=5),
        measurements=dict(slide_dimensions_m=dims.tolist(),loaded_slide_count=len(slides),
            slide_pitch_m=pitch,clear_slide_gap_m=pitch-float(dims[1]),
            glass_mass_kg_nominal_2500kg_m3=float(np.prod(dims)*2500*len(slides)),
            gripper_subtree_estimated_mass_kg=float(sum(m.body_mass[bi] for bi in ids)),
            superseded_all_steel_CAD_mass_kg=1.0304952573570185,
            fingertip_open_gap_m=open_gap,fingertip_closed_gap_m=open_gap-2*stroke,
            rack_handle_neck_width_m=.009,rack_handle_topbar_width_m=.020,
            flat_gap_is_not_groove_grasp_clearance=True,
            grip_interfaces='Original upper handle groove / lower slide groove; no extra pads',
            physical_grasp_evidence='User reports successful printed slide and rack grasps; prior LeHisto Physical Training task records lift-and-return demonstration',
            nominal_wheel_radius_m=.0762,nominal_wheel_track_m=.3),
        gravity_audit=force_rows,
        blockers=[
            "Full lab is still a kinematic model: 20 mocap carrier variants; only right gripper has an actuator.",
            "Arm, lift and wheel servos, measured inertia, torque limits and full collision models are missing.",
            "Nori preview base uses x/y/yaw coordinates. It does not obey rolling/no-lateral-slip physics.",
            "Source rear caster contacts are fixed spheres, not articulated rolling/swiveling caster assemblies.",
            "Robot TCP orientation and actual slide/rack handle frames are not calibrated in the lab.",
            "Do not use flat-tip gap versus neck width as a grip rejection. Original grooved grasp works in user physical tests; loaded-rack simulation and detachable-handle retention remain to be validated.",
            "Loaded slides remain merged in carrier meshes, not independent free bodies with slot contacts.",
            "Machine enclosure colliders are solid envelopes, not open cavities or decomposed insertion geometry.",
            "Passive CAD doors have estimated inertias and no validated collision/contact model.",
            "Leica source CAD includes provisional joints and illustrative filling/transport references.",
            "No verified matching 24-slide bucket CAD; scanner depth and carrier compatibility provisional.",
            "No physically supported rack-type conversion, slide-folder mechanism or block-QC handling.",
            "No calibrated glass fracture model; loss/impact must flag quarantine, not assert breakage."])
    (ROOT/"physics_readiness.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=="__main__":main()
