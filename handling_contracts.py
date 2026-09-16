"""Fail-closed handling checks. Unknown geometry/sensor evidence is NOT success.

These are admission checks, not a motion planner or a replacement for physics.
Tolerance values must come from an application-specific, calibrated contract.
"""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class Decision:
    reasons:tuple
    @property
    def allowed(self):return not self.reasons

def pose_check(position_error,relative_rotation,position_tolerance,angle_tolerance,calibrated):
    p=np.asarray(position_error,float);r=np.asarray(relative_rotation,float);tol=np.asarray(position_tolerance,float)
    if p.shape!=(3,) or r.shape!=(3,3) or tol.shape!=(3,):return Decision(("invalid_pose_shape",))
    if not np.isfinite(np.r_[p,r.ravel(),tol,angle_tolerance]).all():return Decision(("nonfinite_pose",))
    reasons=[]
    if not calibrated:reasons.append("uncalibrated_grasp_frame")
    if np.any(tol<=0) or angle_tolerance<=0:reasons.append("invalid_tolerance")
    if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1,atol=1e-6):
        reasons.append("invalid_rotation")
    if np.any(np.abs(p)>tol):reasons.append("position_misaligned")
    angle=math.acos(float(np.clip((np.trace(r)-1)/2,-1,1)))
    if angle>angle_tolerance:reasons.append("orientation_misaligned")
    return Decision(tuple(reasons))

def grasp_check(*,normal_forces,normal_force_range,bilateral_dwell_s,minimum_dwell_s,
                relative_speed_mps,max_speed_mps,relative_slip_m,max_slip_m,
                lifted_m,minimum_lift_m,joint_limits_ok,effort_limits_ok,unexpected_contact):
    values=[*normal_forces,*normal_force_range,bilateral_dwell_s,minimum_dwell_s,relative_speed_mps,
            max_speed_mps,relative_slip_m,max_slip_m,lifted_m,minimum_lift_m]
    if len(normal_forces)!=2 or not np.isfinite(values).all():return Decision(("invalid_contact_evidence",))
    lo,hi=normal_force_range
    if not (0<lo<hi and minimum_dwell_s>0 and max_speed_mps>0 and max_slip_m>0 and minimum_lift_m>0):
        return Decision(("invalid_grasp_contract",))
    reasons=[]
    if min(normal_forces)<lo:reasons.append("missing_two_jaw_contact")
    if max(normal_forces)>hi:reasons.append("grip_force_exceeded")
    if bilateral_dwell_s<minimum_dwell_s:reasons.append("contact_not_sustained")
    if relative_speed_mps<0 or relative_speed_mps>max_speed_mps:reasons.append("relative_motion")
    if relative_slip_m<0 or relative_slip_m>max_slip_m:reasons.append("slip_detected")
    if lifted_m<minimum_lift_m:reasons.append("lift_not_proven")
    if joint_limits_ok is not True:reasons.append("joint_limit_or_unknown")
    if effort_limits_ok is not True:reasons.append("effort_limit_or_unknown")
    if unexpected_contact is not False:reasons.append("collision_or_unknown")
    return Decision(tuple(reasons))

def insertion_check(*,object_extents,container_clear_extents,relative_rotation,center_error,
                    clearance_m,geometry_verified,swept_path_clear,orientation_ok):
    if container_clear_extents is None:return Decision(("missing_container_cad",))
    dims=np.asarray(object_extents,float);cavity=np.asarray(container_clear_extents,float)
    r=np.asarray(relative_rotation,float);offset=np.asarray(center_error,float)
    if dims.shape!=(3,) or cavity.shape!=(3,) or r.shape!=(3,3) or offset.shape!=(3,):
        return Decision(("invalid_fit_shape",))
    if not np.isfinite(np.r_[dims,cavity,r.ravel(),offset,clearance_m]).all():return Decision(("nonfinite_fit",))
    reasons=[]
    if not geometry_verified:reasons.append("container_geometry_unverified")
    if not swept_path_clear:reasons.append("insertion_path_unverified_or_blocked")
    if not orientation_ok:reasons.append("insertion_orientation_misaligned")
    if np.any(dims<=0) or np.any(cavity<=0) or clearance_m<=0:reasons.append("invalid_fit_contract")
    if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1,atol=1e-6):reasons.append("invalid_rotation")
    # Necessary bounding-volume test only. A positive result also requires exact
    # slot/rail/notch and full swept-path collision evidence supplied above.
    if np.any(np.abs(r)@dims+2*np.abs(offset)+2*clearance_m>cavity):reasons.append("insufficient_container_clearance")
    return Decision(tuple(reasons))

def release_check(*,insertion_decision,support_contact,settled_s,minimum_settled_s,relative_speed_mps,max_speed_mps):
    reasons=list(insertion_decision.reasons)
    if support_contact is not True:reasons.append("no_verified_support")
    if not np.isfinite([settled_s,minimum_settled_s,relative_speed_mps,max_speed_mps]).all():reasons.append("invalid_release_evidence")
    elif minimum_settled_s<=0 or max_speed_mps<=0 or settled_s<minimum_settled_s or not 0<=relative_speed_mps<=max_speed_mps:
        reasons.append("object_not_settled")
    return Decision(tuple(reasons))
