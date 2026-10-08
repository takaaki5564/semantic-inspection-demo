"""Exercise the actual adapter capture loops with held numerical sensor fixtures."""

from copy import deepcopy
from fractions import Fraction
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from arm_camera_adapter import ArmCameraAdapter
from arm_camera_evidence import DEFAULT_MOUNT
from inspection_loop_adapter import InspectionLoopAdapter
from visibility import CameraModel
from render_camera_fixtures import renderer_parameters


class CaptureRenderSyncTests(unittest.TestCase):
    def fixture(self, mode, *, legacy=False):
        adapter = object.__new__(ArmCameraAdapter if legacy else InspectionLoopAdapter)
        adapter.executor_mode = mode
        adapter.CAMERA_PATH, adapter.PART_PATH = "/World/Camera", "/World/Part"
        adapter.render_product_path = "/Render/Inspection"
        adapter.world_part = np.eye(4)
        camera = np.eye(4)
        camera[:3, 3] = [.25, 1.05, 1]
        model = CameraModel(camera, 24, 36, 27, (.01, 100))
        mount = np.asarray(DEFAULT_MOUNT, dtype=float)
        state = {"simulation_time_s": 2., "physics_step_index": 480,
                 "joints_rad": [0.]*6, "velocities_rad_s": [0.]*6,
                 "T_world_flange": (camera@np.linalg.inv(mount)).tolist()}
        movement = {"status": "reached", "goal_id": "rear", "T_world_part": np.eye(4).tolist(),
                    "T_flange_camera": mount.tolist(), "T_world_camera_target": camera.tolist(),
                    "T_world_flange_target": state["T_world_flange"]}
        adapter.read_state = Mock(side_effect=lambda: deepcopy(state))
        adapter.world_transform = lambda path: camera.copy() if path == adapter.CAMERA_PATH else np.eye(4)
        adapter.is_running = Mock(return_value=True)
        adapter.timeline = Mock()
        adapter.timeline.is_playing.return_value = False
        adapter.timeline.get_current_time.return_value = 2.
        adapter.rep, adapter.sensor, adapter.stage = Mock(), Mock(), Mock()
        adapter.sensor.resolution = (480, 640)
        adapter.sensor.render_product.GetCameraRel.return_value.GetTargets.return_value = [adapter.CAMERA_PATH]
        adapter.command, adapter.step = Mock(), Mock()
        adapter.parking_joints = np.zeros(6)
        def pixels():
            rgb = np.full((480, 640, 3), adapter.render_steps, dtype=np.uint8)
            rgb[0, 0] = 0
            return rgb
        adapter.sensor.get_data.return_value = (SimpleNamespace(numpy=pixels), {})
        adapter.reference_annotator = Mock()
        adapter.reference_annotator.get_data.return_value = {"referenceTimeNumerator": 2, "referenceTimeDenominator": 1}
        adapter.last_reference, adapter.render_steps = Fraction(1), 0
        adapter.camera_parameters = Mock()
        adapter.camera_parameters.get_data.return_value = renderer_parameters(model)
        lens = Mock()
        for name, value in (("FocalLength", 24.), ("HorizontalAperture", 36.),
                            ("VerticalAperture", 27.), ("ClippingRange", (.01, 100.))):
            getattr(lens, "Get"+name+"Attr").return_value.Get.return_value = value
        local = Mock()
        local.GetLocalTransformation.return_value = mount.T
        local.GetResetXformStack.return_value = False
        quaternion = SimpleNamespace(GetReal=lambda: 1., GetImaginary=lambda: [0., 0., 0.])
        matrix = SimpleNamespace(ExtractRotationQuat=lambda: quaternion)
        pxr = SimpleNamespace(Usd=SimpleNamespace(TimeCode=SimpleNamespace(Default=lambda: None)),
                              UsdGeom=SimpleNamespace(Xformable=lambda prim: local, Camera=lambda prim: lens),
                              Gf=SimpleNamespace(Matrix4d=lambda values: matrix))
        return adapter, model, movement, state, {"pxr": pxr}

    def test_old_renderer_pose_is_discarded_without_consuming_freshness_or_moving_physics(self):
        for mode in ("arm", "free"):
            adapter, model, movement, _, modules = self.fixture(mode)
            old = deepcopy(renderer_parameters(model))
            old_pose = model.T_world_camera.copy()
            old_pose[1, 3] = -1.05
            old["cameraViewTransform"] = np.linalg.inv(old_pose).T.ravel().tolist()
            def parameters():
                self.assertEqual(adapter.last_reference, Fraction(1))
                return old if adapter.render_steps <= 2 else renderer_parameters(model)
            adapter.camera_parameters.get_data.side_effect = parameters
            with self.subTest(mode=mode), patch.dict(sys.modules, modules):
                rgb, record = adapter.capture("images/capture_01.png", movement)
            self.assertEqual(adapter.render_steps, 3)
            self.assertEqual(int(rgb[1, 1, 0]), 3)
            self.assertEqual(record["render_synchronization"]["accepted_attempt"], 3)
            self.assertEqual(len(record["render_synchronization"]["rejected_renderer_frames"]), 2)
            self.assertEqual(adapter.last_reference, Fraction(2))
            adapter.command.assert_not_called()
            adapter.step.assert_not_called()
            for call in adapter.rep.orchestrator.step.call_args_list:
                self.assertEqual(call.kwargs["delta_time"], 0.)

    def test_permanent_pose_mismatch_stops_after_ten_renders_without_advancing_reference(self):
        for mode in ("arm", "free"):
            adapter, model, movement, _, modules = self.fixture(mode)
            parameters = renderer_parameters(model)
            parameters["cameraViewTransform"] = np.eye(4).ravel().tolist()
            adapter.camera_parameters.get_data.return_value = parameters
            with self.subTest(mode=mode), patch.dict(sys.modules, modules), self.assertRaisesRegex(RuntimeError, "did not synchronize after 10"):
                adapter.capture("images/capture_01.png", movement)
            self.assertEqual(adapter.render_steps, 10)
            self.assertEqual(adapter.last_reference, Fraction(1))

    def test_optics_failure_is_immediate_and_cannot_be_hidden_as_pose_warmup(self):
        for mode in ("arm", "free"):
            adapter, model, movement, _, modules = self.fixture(mode)
            parameters = renderer_parameters(model)
            parameters["cameraFocalLength"] = 99.
            adapter.camera_parameters.get_data.return_value = parameters
            with self.subTest(mode=mode), patch.dict(sys.modules, modules), self.assertRaisesRegex(ValueError, "optics mismatch"):
                adapter.capture("images/capture_01.png", movement)
            self.assertEqual(adapter.render_steps, 1)
            self.assertEqual(adapter.last_reference, Fraction(1))

    def test_physical_change_during_retry_is_rejected(self):
        for mode in ("arm", "free"):
            adapter, model, movement, state, modules = self.fixture(mode)
            parameters = renderer_parameters(model)
            parameters["cameraViewTransform"] = np.eye(4).ravel().tolist()
            adapter.camera_parameters.get_data.return_value = parameters
            def read():
                value = deepcopy(state)
                if adapter.render_steps >= 2:
                    value["physics_step_index"] += 1
                    value["simulation_time_s"] += 1/240
                return value
            adapter.read_state.side_effect = read
            with self.subTest(mode=mode), patch.dict(sys.modules, modules), self.assertRaisesRegex((ValueError, RuntimeError), "Physics|physics"):
                adapter.capture("images/capture_01.png", movement)
            self.assertEqual(adapter.render_steps, 2)
            self.assertEqual(adapter.last_reference, Fraction(1))

    def test_legacy_arm_capture_keeps_default_protocol_without_camera_params(self):
        adapter, _, movement, _, modules = self.fixture("arm", legacy=True)
        with patch.dict(sys.modules, modules):
            _, record = adapter.capture("images/view.png", movement)
        adapter.camera_parameters.get_data.assert_not_called()
        self.assertNotIn("renderer_camera", record)
        self.assertEqual(adapter.last_reference, Fraction(2))


if __name__ == "__main__":
    unittest.main()
