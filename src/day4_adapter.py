"""Day 4 matrix-pose movement; verified Day 1/2 adapters remain unchanged."""

import time

import numpy as np
from pxr import Gf, UsdGeom

from capture_metadata import rigid_transform
from day2_adapter import VisibilityAdapter


class ClosedLoopAdapter(VisibilityAdapter):
    def __init__(self, app, *, headless=False):
        super().__init__(app, headless=headless)
        self.motion_seconds = 0.1 if headless else 3.0

    def set_part_relative_pose(self, pose):
        world_camera = self.world_transform(self.PART_PATH) @ rigid_transform(pose)
        camera = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.CAMERA_PATH))
        camera.MakeMatrixXform().Set(Gf.Matrix4d(world_camera.T.tolist()))

    def move_to_part_pose(self, pose):
        target = rigid_transform(pose)
        start_pose = np.linalg.inv(self.world_transform(self.PART_PATH)) @ self.world_transform(self.CAMERA_PATH)
        start_q = Gf.Matrix4d(start_pose.T.tolist()).ExtractRotationQuat()
        target_q = Gf.Matrix4d(target.T.tolist()).ExtractRotationQuat()
        start_time = time.monotonic()
        while True:
            fraction = min((time.monotonic() - start_time) / self.motion_seconds, 1.0)
            smooth = fraction * fraction * (3 - 2 * fraction)
            quaternion = Gf.Slerp(smooth, start_q, target_q)
            intermediate = np.array(Gf.Matrix4d(1).SetRotate(quaternion), dtype=float).T.copy()
            intermediate[:3, 3] = (1 - smooth) * start_pose[:3, 3] + smooth * target[:3, 3]
            self.set_part_relative_pose(target if fraction == 1 else intermediate)
            self.update()
            if fraction == 1:
                self.hold(0.1 if self.motion_seconds < 1 else 0.5)
                return
            time.sleep(1 / 120)
