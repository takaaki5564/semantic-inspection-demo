from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from capture_metadata import camera_look_at
from render_camera_evidence import validate_render_camera
from visibility import CameraModel
from render_camera_fixtures import renderer_parameters


class RendererCameraTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel(camera_look_at((.25, -1.05, 1), (-.6, 0, .33)), 24, 36, 27, (.01, 100))
        self.parameters = renderer_parameters(self.camera)

    def test_row_vector_view_transform_recovers_actual_camera_pose(self):
        evidence = validate_render_camera(self.parameters, self.camera)
        np.testing.assert_allclose(evidence["T_world_camera"], self.camera.T_world_camera, atol=1e-12)

    def test_renderer_identity_or_wrong_pose_is_rejected_even_when_usd_pose_is_valid(self):
        for view in (np.eye(4), np.linalg.inv(camera_look_at((.25, 1.05, 1), (-.6, 0, .33))).T):
            data = deepcopy(self.parameters)
            data["cameraViewTransform"] = view.ravel().tolist()
            with self.assertRaisesRegex(ValueError, "renderer/USD camera"):
                validate_render_camera(data, self.camera)

    def test_wrong_lens_projection_resolution_clipping_or_units_is_rejected(self):
        for key, value in (("cameraModel", "fisheyeOpenCV"), ("cameraFocalLength", 1),
                           ("cameraProjection", np.eye(4).ravel().tolist()),
                           ("renderProductResolution", [480, 640]), ("cameraNearFar", [.1, 10]),
                           ("metersPerSceneUnit", .01), ("cameraApertureOffset", [1, 0])):
            data = deepcopy(self.parameters)
            data[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "optics"):
                validate_render_camera(data, self.camera)


if __name__ == "__main__":
    unittest.main()
