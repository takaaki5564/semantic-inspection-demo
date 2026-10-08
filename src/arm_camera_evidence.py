"""Checks for a camera rigidly attached to a measured flange; simulator independent."""

import numpy as np

from arm_kinematics import pose_error
from arm_motion import TOLERANCES
from camera_executors import camera_error_ok
from capture_metadata import reference_time, rigid_transform


# USD camera optical axes: +X right, +Y up, -Z viewing; rotation is relative to flange.
DEFAULT_MOUNT = [[0, -1, 0, 0.18], [1, 0, 0, 0], [0, 0, 1, 0.08], [0, 0, 0, 1]]
EVIDENCE_TOLERANCES = {"mount_position_m":1e-6,"mount_orientation_rad":1e-6,
                       "physical_attachment_position_m":1e-5,"physical_attachment_orientation_rad":1e-5,
                       "capture_drift_position_m":1e-7,"capture_drift_orientation_rad":1e-6,
                       "reference_clock_seconds":1e-7}


def assert_pose_close(actual, expected, label, *, position=1e-5, angle=1e-5):
    error = pose_error(actual, expected)
    if error["position_error_m"] > position or error["orientation_error_rad"] > angle:
        raise ValueError(f"{label}: {error}")
    return error


def mounted_capture_record(*, image, movement, state_before, state_after, camera_before, camera_after,
                           local_mount, reference, previous_reference, readback_utc, render_step_index):
    if movement["status"] != "reached":
        raise ValueError("Cannot capture an unreached camera goal")
    velocity = np.asarray(state_after["velocities_rad_s"],dtype=float)
    if velocity.shape != (6,) or not np.isfinite(velocity).all() or np.max(np.abs(velocity)) > TOLERANCES["velocity_rad_s"]:
        raise ValueError("Arm no longer settled at RGB readback")
    if state_before["simulation_time_s"] != state_after["simulation_time_s"] or state_before["physics_step_index"] != state_after["physics_step_index"]:
        raise ValueError("Physics advanced during RGB capture")
    assert_pose_close(state_after["T_world_flange"], state_before["T_world_flange"], "Flange moved during capture",
                      position=EVIDENCE_TOLERANCES["capture_drift_position_m"],angle=EVIDENCE_TOLERANCES["capture_drift_orientation_rad"])
    assert_pose_close(camera_after, camera_before, "Camera moved during capture",
                      position=EVIDENCE_TOLERANCES["capture_drift_position_m"],angle=EVIDENCE_TOLERANCES["capture_drift_orientation_rad"])
    # CameraSensor/RTX transform storage introduces sub-micrometer float roundoff.
    mount_error = assert_pose_close(local_mount, movement["T_flange_camera"], "Camera mount changed",
                                    position=EVIDENCE_TOLERANCES["mount_position_m"],angle=EVIDENCE_TOLERANCES["mount_orientation_rad"])
    actual_flange = rigid_transform(state_after["T_world_flange"])
    predicted = actual_flange @ rigid_transform(local_mount)
    attachment_error = assert_pose_close(camera_after, predicted, "USD camera disagrees with physical flange",
                                         position=EVIDENCE_TOLERANCES["physical_attachment_position_m"],angle=EVIDENCE_TOLERANCES["physical_attachment_orientation_rad"])
    goal_error = pose_error(camera_after, movement["T_world_camera_target"])
    if not camera_error_ok(goal_error):
        raise ValueError("Camera goal not reached at RGB readback")
    clock = reference_time(reference)
    if previous_reference is not None and clock <= previous_reference:
        raise ValueError("Stale RGB reference time")
    if abs(float(clock)-state_after["simulation_time_s"]) > EVIDENCE_TOLERANCES["reference_clock_seconds"]:
        raise ValueError("RGB reference time does not match the held physical snapshot")
    world_camera, world_part = rigid_transform(camera_after), rigid_transform(movement["T_world_part"])
    return {"image": image, "status": "captured", "evidence_source": "simulator_rgb", "inspection_evaluated": False,
            "pose_source": "USD camera world transform checked against PhysX flange and fixed local mount",
            "goal_id": movement["goal_id"], "T_world_part": world_part.tolist(),
            "T_part_camera": (np.linalg.inv(world_part) @ world_camera).tolist(),
            "T_world_camera": world_camera.tolist(), "T_world_camera_from_physics": predicted.tolist(),
            "T_world_flange": actual_flange.tolist(), "T_flange_camera": rigid_transform(local_mount).tolist(),
            "T_flange_camera_nominal": movement["T_flange_camera"],
            "T_world_camera_target": movement["T_world_camera_target"], "T_world_flange_target": movement["T_world_flange_target"],
            "world_position_m": world_camera[:3,3].tolist(), "camera_pose_error": goal_error,
            "mount_error": mount_error, "physical_attachment_error": attachment_error,
            "joints_rad": state_after["joints_rad"], "velocities_rad_s": state_after["velocities_rad_s"],
            "physics_time_seconds": state_after["simulation_time_s"], "physics_step_index": state_after["physics_step_index"],
            "physics_held_during_capture": True, "render_step_index": render_step_index,
            "render_reference_time": {k:int(reference[k]) for k in ("referenceTimeNumerator","referenceTimeDenominator")},
            "readback_utc": readback_utc}
