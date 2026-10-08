from pathlib import Path
import sys
import unittest
from unittest.mock import MagicMock, Mock, patch
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from inspection_loop_adapter import InspectionLoopAdapter


class RobotParkingTests(unittest.TestCase):
    def adapter(self,*,velocity=0,drift=0):
        adapter = object.__new__(InspectionLoopAdapter)
        adapter.dt = 1/240
        adapter.camera_settings = {}
        adapter.command,adapter.step = Mock(),Mock()
        adapter.is_running = lambda:True
        first = {"joints_rad":[0]*6,"velocities_rad_s":[0]*6}
        later = {"joints_rad":[drift]*6,"velocities_rad_s":[velocity]*6}
        adapter.read_state = Mock(side_effect=lambda:first if adapter.step.call_count == 0 else later)
        return adapter

    def test_parking_waits_for_settling_using_unchanged_joint_targets(self):
        adapter = self.adapter()
        adapter._settle_parked_robot()
        self.assertGreaterEqual(adapter.step.call_count*adapter.dt,.25)
        for command in adapter.command.call_args_list:
            np.testing.assert_array_equal(command.args[0],np.zeros(6))

    def test_drifting_parking_pose_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,"maintain"):
            self.adapter(drift=.1)._settle_parked_robot()

    def test_nonsettling_parking_has_a_finite_step_limit(self):
        adapter = self.adapter(velocity=.1)
        with self.assertRaisesRegex(RuntimeError,"within 3"):
            adapter._settle_parked_robot()
        self.assertEqual(adapter.step.call_count,720)

    def test_free_sensor_switch_keeps_original_texture_alive(self):
        adapter = object.__new__(InspectionLoopAdapter)
        original = adapter.sensor = Mock()
        adapter.stage, adapter.rep = Mock(), Mock()
        adapter.reference_annotator = Mock()
        adapter.render_product_path = "/Render/OldWrist"
        adapter.camera_settings = {}
        replacement = Mock()
        replacement.render_product.GetPath.return_value = "/Render/Free"
        replacement.render_product.GetCameraRel.return_value.GetTargets.return_value = ["/World/FreeInspectionCamera"]
        sensors = SimpleNamespace(CameraSensor=Mock(return_value=replacement), RtxCamera=Mock())
        pxr = SimpleNamespace(Gf=MagicMock(), UsdGeom=MagicMock())
        with patch.dict(sys.modules, {"isaacsim.sensors.experimental.rtx": sensors, "pxr": pxr}):
            adapter._create_free_camera()
        self.assertIs(adapter.parked_wrist_sensor, original)
        self.assertIs(adapter.sensor, replacement)
        original.detach_annotators.assert_called_once_with("rgb")


if __name__ == "__main__":
    unittest.main()
