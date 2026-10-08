"""Part-relative camera-pose interface; no Isaac Sim imports.

Both executors expose move_to_part_pose(pose, goal_id=..., emit=...) and capture(image).
Existing Day 1-4 entrypoints are not switched to this interface in this substep.
"""

import numpy as np

from arm_kinematics import pose_error
from arm_motion import TOLERANCES, execute_goal
from capture_metadata import rigid_transform


def camera_to_flange(T_world_camera, T_flange_camera, T_world_base):
    world_flange = rigid_transform(T_world_camera) @ np.linalg.inv(rigid_transform(T_flange_camera))
    return world_flange, np.linalg.inv(rigid_transform(T_world_base)) @ world_flange


def camera_error_ok(error):
    return error["position_error_m"] <= TOLERANCES["position_m"] and error["orientation_error_rad"] <= TOLERANCES["orientation_rad"]


class FreeCameraExecutor:
    def __init__(self, adapter, T_world_part):
        self.adapter = adapter
        self.world_part = rigid_transform(T_world_part).copy()
        self.last_result = None

    def move_to_part_pose(self, pose, *, goal_id="view", emit=lambda event: None):
        pose = rigid_transform(pose)
        self.adapter.move_to_part_pose(pose)
        actual = rigid_transform(self.adapter.world_transform(self.adapter.CAMERA_PATH))
        error = pose_error(actual, self.world_part @ pose)
        self.last_result = {"goal_id": goal_id, "executor": "free_camera", "status": "reached" if camera_error_ok(error) else "blocked",
                            "reason": "measured_camera_pose_reached" if camera_error_ok(error) else "camera_pose_error",
                            "T_part_camera_target": pose.tolist(), "T_world_camera_target": (self.world_part @ pose).tolist(),
                            "T_world_camera_actual": actual.tolist(), "camera_pose_error": error}
        emit({"event": "camera_goal_finished", **self.last_result})
        return self.last_result

    def capture(self, image):
        if self.last_result is None or self.last_result["status"] != "reached":
            raise ValueError("A camera goal must be reached before capture")
        actual = self.adapter.world_transform(self.adapter.CAMERA_PATH)
        if not camera_error_ok(pose_error(actual,self.last_result["T_world_camera_target"])):
            raise ValueError("Free camera moved away from the reached goal")
        return self.adapter.capture(image)


class ArmMountedCameraExecutor:
    def __init__(self, adapter, T_world_part, T_flange_camera, *, max_motion_seconds=12.0):
        self.adapter = adapter
        self.world_part = rigid_transform(T_world_part).copy()
        self.mount = rigid_transform(T_flange_camera).copy()
        self.max_motion_seconds = max_motion_seconds
        self.last_result = None

    def move_to_part_pose(self, pose, *, goal_id="view", emit=lambda event: None):
        pose = rigid_transform(pose)
        state = self.adapter.read_state()
        world_camera = self.world_part @ pose
        world_flange, base_flange = camera_to_flange(world_camera, self.mount, state["T_world_base"])
        result = execute_goal(self.adapter, {"goal_id": goal_id, "T_base_flange_target": base_flange.tolist()},
                              self.adapter.limits, emit, max_motion_seconds=self.max_motion_seconds)
        actual = rigid_transform(result["final_state"]["T_world_flange"]) @ self.mount
        error = pose_error(actual, world_camera)
        result.update(executor="arm_mounted_camera", T_world_part=self.world_part.tolist(),
                      T_flange_camera=self.mount.tolist(), T_part_camera_target=pose.tolist(),
                      T_world_camera_target=world_camera.tolist(), T_world_flange_target=world_flange.tolist(),
                      T_world_camera_actual_from_physics=actual.tolist(), camera_pose_error=error)
        if result["status"] == "reached" and not camera_error_ok(error):
            result.update(status="blocked", reason="camera_pose_error")
        self.last_result = result
        emit({"event": "camera_goal_finished", **result})
        return result

    def capture(self, image):
        if self.last_result is None or self.last_result["status"] != "reached":
            raise ValueError("A settled camera goal must be reached before capture")
        return self.adapter.capture(image, self.last_result)
