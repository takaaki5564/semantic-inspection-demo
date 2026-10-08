"""Reachability screening and measured arm-scene observation; no Isaac Sim imports."""

import numpy as np

from arm_kinematics import pose_error
from arm_motion import IK_TOLERANCES, vector, within_limits
from arm_inspection_geometry import ROBOT_ROOT
from camera_executors import camera_to_flange
from capture_metadata import rigid_transform
from closed_loop import validate_frame
from visibility import evaluate_visibility


def solve_posture_ik(solve, model_pose, target, start, limits, seed_postures, preferred_posture):
    """Deterministic robot-only IK branch preference, independent of part/spec/view IDs."""
    start = vector(start, len(limits["lower"]))
    preferred = vector(preferred_posture, len(start))
    seeds = [start, *(vector(seed, len(start)) for seed in seed_postures)]
    if not within_limits(preferred, limits) or any(not within_limits(seed, limits) for seed in seeds):
        raise ValueError("IK seed/reference posture exceeds model limits")
    attempts, candidates = [], []
    for seed in seeds:
        answer = solve(target, seed.copy())
        attempts.append({"numerical_seed_rad": seed.tolist(), "solver": answer})
        if not answer["success"]:
            continue
        q = vector(answer["joints_rad"], len(start)).copy()
        if not within_limits(q, limits):
            continue
        # Equivalent revolute values closest to the actual state, without exceeding limits.
        for i in range(len(q)):
            choices = [q[i]+2*np.pi*k for k in range(-2,3) if limits["lower"][i] <= q[i]+2*np.pi*k <= limits["upper"][i]]
            q[i] = min(choices, key=lambda value: abs(value-start[i]))
        error = pose_error(model_pose(q), target)
        if error["position_error_m"] <= IK_TOLERANCES["position_m"] and error["orientation_error_rad"] <= IK_TOLERANCES["orientation_rad"]:
            candidates.append(q)
    if not candidates:
        return {"success": False, "attempts": attempts}
    def score(q):
        wrapped = np.arctan2(np.sin(q-preferred), np.cos(q-preferred))
        return float(np.linalg.norm(wrapped)), float(np.linalg.norm(q-start)), tuple(q)
    q = min(candidates, key=score)
    return {"success": True, "joints_rad": q.tolist(), "attempts": attempts,
            "selection": "nearest configured reference posture in wrapped joint distance, then actual joints",
            "preferred_posture_rad": preferred.tolist(), "collision_aware": False}


def screen_candidates(adapter, world_part, mount, candidates):
    state = adapter.read_state()
    start = np.asarray(state["joints_rad"], dtype=float)
    world_part = rigid_transform(world_part)
    feasible, records = [], []
    for view in candidates:
        world_camera = world_part@view.T_part_camera
        world_flange, base_flange = camera_to_flange(world_camera, mount, state["T_world_base"])
        ik = adapter.solve_ik(base_flange, start.copy())
        reason, residual = "ik_failed", None
        if ik["success"]:
            q = vector(ik["joints_rad"], len(start))
            residual = pose_error(adapter.model_pose(q), base_flange)
            if not within_limits(q, adapter.limits):
                reason = "ik_joint_limit_violation"
            elif (residual["position_error_m"] > IK_TOLERANCES["position_m"]
                  or residual["orientation_error_rad"] > IK_TOLERANCES["orientation_rad"]):
                reason = "ik_pose_residual"
            else:
                reason = "bounded_ik_solution"
                feasible.append(view)
        records.append({"view_id": view.view_id, "status": "feasible" if reason == "bounded_ik_solution" else "blocked",
                        "reason": reason, "T_part_camera_target": view.T_part_camera.tolist(),
                        "T_world_camera_target": world_camera.tolist(), "T_world_flange_target": world_flange.tolist(),
                        "T_base_flange_target": base_flange.tolist(), "ik": ik, "ik_model_residual": residual,
                        "screened_from_physics_step": state["physics_step_index"],
                        "collision_safety_validated": False, "motion_executed": False})
    if adapter.read_state() != state:
        raise RuntimeError("Reachability screening must not advance physics")
    return tuple(feasible), records


def require_static_scene(initial, current):
    if (initial[0].geometry_version != current[0].geometry_version
            or initial[3]["static_scene_sha256"] != current[3]["static_scene_sha256"]
            or not np.allclose(initial[0].T_world_part, current[0].T_world_part, rtol=0, atol=1e-9)):
        raise ValueError("Part or static cell changed; reset the inspection session")


def observe_capture(before, after, rgb, capture, state, spec, capture_id, *, previous_reference=None, previous_rgb=None):
    # Full geometry must be unchanged while held; it may change between arm motions.
    for snapshot in (before, after):
        meta = snapshot[3]
        if (meta["physics_step_index"] != capture["physics_step_index"]
                or abs(meta["physics_time_seconds"]-capture["physics_time_seconds"]) > 1e-10):
            raise ValueError("Geometry snapshot does not match the captured physics step")
    if before[3]["full_scene_sha256"] != after[3]["full_scene_sha256"]:
        raise ValueError("Visibility geometry changed during capture")
    clock = validate_frame(rgb, capture, before[:3], after[:3], previous_reference, previous_rgb, capture["image"])
    report = evaluate_visibility(*after[:3], required_region_ids=spec.required_region_ids, inspection_spec_version=spec.version)
    report["geometry_snapshot"] = after[3]
    report["robot_occluded_point_ids"] = {
        name: [p["point_id"] for p in value["points"] if p["reason"] == "scene_occlusion" and p["hit"]
               and p["hit"]["prim_path"].startswith(ROBOT_ROOT+"/")]
        for name, value in report["regions"].items()}
    state.record_capture(capture_id)
    state.apply_visibility(capture_id, report)
    return report, clock
