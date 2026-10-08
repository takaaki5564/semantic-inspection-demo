"""Bounded arm-motion probe and smooth joint targets; no simulator imports."""

import math
import time
import xml.etree.ElementTree as ET
import json

import numpy as np

from arm_kinematics import pose_error
from capture_metadata import rigid_transform


TOLERANCES = {"position_m": 0.005, "orientation_rad": math.radians(1),
              "joint_rad": 0.01, "velocity_rad_s": 0.02, "settle_seconds": 0.25}
IK_TOLERANCES = {"position_m": 1e-5, "orientation_rad": 1e-3}


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"Expected a finite {size}-joint vector")
    return result


def motion_limits(model_dir, names):
    """Use the validated model's explicit demo limits, matching by joint name."""
    urdf = ET.parse(model_dir / "robot.urdf").getroot()
    joints = {j.attrib["name"]: j.find("limit") for j in urdf.findall("joint") if j.attrib["type"] != "fixed"}
    xrdf = json.loads((model_dir / "robot.xrdf").read_text())
    cspace = xrdf["cspace"]
    if cspace["joint_names"] != list(names) or set(joints) != set(names):
        raise ValueError("Motion limit joint names do not match the model")
    result = {key: vector([float(joints[n].attrib[attr]) for n in names], len(names))
              for key, attr in (("lower", "lower"), ("upper", "upper"), ("velocity", "velocity"))}
    result.update(acceleration=vector(cspace["acceleration_limits"], len(names)),
                  jerk=vector(cspace["jerk_limits"], len(names)))
    if np.any(result["lower"] >= result["upper"]) or any(np.any(result[k] <= 0) for k in ("velocity", "acceleration", "jerk")):
        raise ValueError("Invalid motion limits")
    return result


def within_limits(q, limits):
    q = vector(q, len(limits["lower"]))
    return bool(np.all(q >= limits["lower"]) and np.all(q <= limits["upper"]))


def trajectory(start, goal, limits):
    """Quintic blend: analytic velocity/acceleration/jerk bounds on COMMANDS."""
    start, goal = vector(start, len(limits["lower"])), vector(goal, len(limits["lower"]))
    if not within_limits(start, limits) or not within_limits(goal, limits):
        raise ValueError("Trajectory endpoint exceeds joint limits")
    distance = np.abs(goal - start)
    duration = max(3.0, float(np.max(1.875 * distance / limits["velocity"])),
                   float(np.max(np.sqrt((10 / math.sqrt(3)) * distance / limits["acceleration"]))),
                   float(np.max(np.cbrt(60 * distance / limits["jerk"]))))
    return {"duration_seconds": duration, "start_rad": start.tolist(), "goal_rad": goal.tolist(),
            "maximum_command_velocity_rad_s": (1.875 * distance / duration).tolist(),
            "maximum_command_acceleration_rad_s2": ((10 / math.sqrt(3)) * distance / duration**2).tolist(),
            "maximum_command_jerk_rad_s3": (60 * distance / duration**3).tolist()}


def trajectory_sample(profile, elapsed):
    if not math.isfinite(elapsed):
        raise ValueError("Trajectory time must be finite")
    u = np.clip(elapsed / profile["duration_seconds"], 0, 1)
    s = 10*u**3 - 15*u**4 + 6*u**5
    return np.asarray(profile["start_rad"]) + s * (np.asarray(profile["goal_rad"]) - profile["start_rad"])


def execute_goal(adapter, goal, limits, emit, *, max_motion_seconds=12.0,
                 wall_timeout_seconds=60.0, clock=time.monotonic):
    """IK -> bounded targets -> measured pose + joint/velocity settling.

    Adapter methods: read_state, solve_ik, model_pose, command, step, is_running.
    Actual flange poses must come from physics, independently of model_pose.
    """
    if not math.isfinite(max_motion_seconds) or max_motion_seconds <= 0:
        raise ValueError("Motion timeout must be positive and finite")
    target = rigid_transform(goal["T_base_flange_target"])
    initial = adapter.read_state()
    start = vector(initial["joints_rad"], len(limits["lower"]))
    base = rigid_transform(initial["T_world_base"])
    world_target = base @ target
    result = {"goal_id": goal["goal_id"], "status": "blocked", "reason": None,
              "T_base_flange_target": target.tolist(), "T_world_flange_target": world_target.tolist(),
              "initial_state": initial, "final_state": initial, "physics_steps": 0,
              "max_actual_joint_displacement_rad": 0.0, "settled_seconds": 0.0}

    def finish(reason):
        result["reason"] = reason
        emit({"event": "goal_finished", **result})
        return result

    emit({"event": "goal_started", "goal_id": goal["goal_id"], "initial_state": initial,
          "T_base_flange_target": target.tolist(), "T_world_flange_target": world_target.tolist()})
    if not within_limits(start, limits):
        return finish("initial_joint_limit_violation")
    ik = adapter.solve_ik(target, start)
    result["ik"] = ik
    emit({"event": "ik_result", "goal_id": goal["goal_id"], **ik})
    if not ik["success"]:
        return finish("ik_failed")
    q_goal = vector(ik["joints_rad"], len(start))
    if not within_limits(q_goal, limits):
        return finish("ik_joint_limit_violation")
    residual = pose_error(adapter.model_pose(q_goal), target)
    result["ik_model_residual"] = residual
    if residual["position_error_m"] > IK_TOLERANCES["position_m"] or residual["orientation_error_rad"] > IK_TOLERANCES["orientation_rad"]:
        return finish("ik_pose_residual")
    if np.max(np.abs(q_goal - start)) < 0.03:
        return finish("goal_does_not_demonstrate_motion")
    profile = trajectory(start, q_goal, limits)
    result["trajectory"] = profile
    if profile["duration_seconds"] + TOLERANCES["settle_seconds"] > max_motion_seconds:
        return finish("insufficient_motion_time_budget")

    start_time = previous_time = float(initial["simulation_time_s"])
    if not math.isfinite(start_time) or not math.isfinite(adapter.dt) or adapter.dt <= 0:
        raise ValueError("Invalid physics clock")
    wall_start, settled_since = clock(), None
    for _ in range(math.ceil(max_motion_seconds / adapter.dt) + 1):
        if not adapter.is_running():
            return finish("simulator_closed")
        if clock() - wall_start > wall_timeout_seconds:
            return finish("wall_clock_timeout")
        command = trajectory_sample(profile, previous_time - start_time + adapter.dt)
        adapter.command(command)
        adapter.step()
        state = adapter.read_state()
        result["physics_steps"] += 1
        result["final_state"] = state
        now = float(state["simulation_time_s"])
        if not math.isfinite(now) or now <= previous_time:
            return finish("physics_clock_did_not_advance")
        elapsed = now - start_time
        q = vector(state["joints_rad"], len(start))
        velocity = vector(state["velocities_rad_s"], len(start))
        error = pose_error(state["T_world_flange"], world_target)
        result["final_error"] = {**error, "maximum_joint_error_rad": float(np.max(np.abs(q - q_goal))),
                                  "maximum_joint_velocity_rad_s": float(np.max(np.abs(velocity)))}
        result["max_actual_joint_displacement_rad"] = max(result["max_actual_joint_displacement_rad"], float(np.max(np.abs(q - start))))
        emit({"event": "physics_sample", "goal_id": goal["goal_id"], "command_joints_rad": command.tolist(),
              "elapsed_seconds": elapsed, "state": state, "error": result["final_error"]})
        if not within_limits(q, limits):
            return finish("actual_joint_limit_violation")
        base_error = pose_error(state["T_world_base"], base)
        if base_error["position_error_m"] > 1e-5 or base_error["orientation_error_rad"] > 1e-5:
            return finish("fixed_base_moved")
        converged = (elapsed >= profile["duration_seconds"]
                     and error["position_error_m"] <= TOLERANCES["position_m"]
                     and error["orientation_error_rad"] <= TOLERANCES["orientation_rad"]
                     and result["final_error"]["maximum_joint_error_rad"] <= TOLERANCES["joint_rad"]
                     and result["final_error"]["maximum_joint_velocity_rad_s"] <= TOLERANCES["velocity_rad_s"])
        settled_since = (now if settled_since is None else settled_since) if converged else None
        result["settled_seconds"] = 0.0 if settled_since is None else now - settled_since
        if result["settled_seconds"] >= TOLERANCES["settle_seconds"] and result["max_actual_joint_displacement_rad"] >= 0.03:
            result["status"] = "reached"
            return finish("measured_pose_and_joint_settled")
        if elapsed >= max_motion_seconds:
            return finish("motion_timeout")
        previous_time = now
    return finish("physics_step_limit")
