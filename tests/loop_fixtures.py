"""Synthetic adapter for core tests only; never used by the simulator runner."""

from dataclasses import replace

import numpy as np

from capture_metadata import capture_record
from geometry_fixtures import demo_camera, demo_scene


class FakeAdapter:
    def __init__(self, rib_height=0.12, *, world_part=None, fault=None):
        self.scene, self.regions = demo_scene(rib_height, world_part)
        self.camera = demo_camera(world_part=world_part)
        self.fault = fault
        self.capture_count = 0
        self.moves = []
        self.images = []

    def geometry_snapshot(self):
        return self.scene, self.regions

    def camera_model(self):
        return self.camera

    def move_to_part_pose(self, pose):
        self.moves.append(np.asarray(pose).copy())
        if self.fault != "ignored_motion":
            self.camera = replace(self.camera, T_world_camera=self.scene.T_world_part @ pose)
        if self.fault == "no_actual_gain":
            self.camera = replace(self.camera, horizontal_aperture_mm=0.001)

    def capture(self, image):
        self.capture_count += 1
        clock = 1 if self.fault == "stale_reference" else self.capture_count
        rgb = np.zeros((*self.camera.resolution_hw, 3), dtype=np.uint8)
        rgb[:, :, 0] = np.arange(rgb.shape[1], dtype=np.uint16) % 256
        rgb[0, 0] = self.capture_count
        if self.fault == "stale_rgb" and self.images:
            rgb = self.images[0].copy()
        record = capture_record(image=image, world_part=self.scene.T_world_part,
                                world_camera=self.camera.T_world_camera, quaternion_wxyz=[1, 0, 0, 0],
                                reference={"referenceTimeNumerator": clock, "referenceTimeDenominator": 60},
                                timeline_seconds=clock / 60, app_update_count=self.capture_count,
                                readback_utc="test_fixture_not_real_capture")
        if self.capture_count > 1 and self.fault == "wrong_capture_pose":
            record["T_world_camera"][0][3] += 0.1
        if self.capture_count > 1 and self.fault == "scene_changed":
            self.scene, self.regions = demo_scene(0.2, self.scene.T_world_part)
        self.images.append(rgb.copy())
        return rgb, record
