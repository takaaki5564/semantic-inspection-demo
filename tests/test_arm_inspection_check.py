from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
import arm_inspection_check as runner
import test_arm_camera_check as camera_test_fixtures
import test_arm_inspection as inspection_test_fixtures
from viewpoint_planner import CandidateView


class InspectionRunnerTests(unittest.TestCase):
    def fixture(self, folder):
        motion_path, output, modules, app, _, _, args = camera_test_fixtures.CameraRunnerTests().fixture(folder)
        motion = json.loads(motion_path.read_text())
        camera = Path(folder)/"camera_report.json"
        camera.write_text(json.dumps({"demo_step": "arm_camera_two_rgb_views", "run_status": "passed", "checks": {"ok": True},
                                      "captures": [{},{}], "runtime": motion["runtime"], "model_manifest": {}}))
        snapshot, _, _, _, _ = inspection_test_fixtures.CapturedObservationTests().fixture()
        adapter = SimpleNamespace(settings={}, camera_settings={}, world_part=np.eye(4), mount=np.eye(4),
                                  snapshot=Mock(return_value=snapshot), close=Mock())
        modules["arm_inspection_adapter"] = SimpleNamespace(ArmInspectionAdapter=Mock(return_value=adapter))
        # No simulator camera/physics modules may be imported during core runner tests.
        modules["day2_visibility"] = SimpleNamespace(save_overlay=Mock())
        args += ["--camera-report", str(camera)]
        return camera, output, modules, app, adapter, args

    def test_failed_camera_input_and_existing_output_do_not_start_simulator(self):
        for fault in ("camera", "output"):
            with tempfile.TemporaryDirectory() as folder:
                camera, output, modules, _, _, args = self.fixture(folder)
                if fault == "camera":
                    data = json.loads(camera.read_text())
                    data["run_status"] = "failed"
                    camera.write_text(json.dumps(data))
                else:
                    output.mkdir()
                    (output/"original.txt").write_text("preserve")
                with patch.dict(sys.modules, modules), patch.object(runner.importlib.metadata, "version", return_value="6.1.0.0"), redirect_stderr(StringIO()):
                    self.assertEqual(runner.main(args), 1)
                modules["isaacsim"].SimulationApp.assert_not_called()
                if fault == "output":
                    self.assertEqual((output/"original.txt").read_text(), "preserve")

    def test_infeasible_requested_view_stops_before_movement_and_capture(self):
        with tempfile.TemporaryDirectory() as folder:
            _, output, modules, app, adapter, args = self.fixture(folder)
            executor = Mock()
            feasible = (CandidateView("front", np.eye(4)),)
            with patch.dict(sys.modules, modules), patch.object(runner.importlib.metadata, "version", return_value="6.1.0.0"), \
                    patch.object(runner, "ArmMountedCameraExecutor", return_value=executor), \
                    patch.object(runner, "screen_candidates", return_value=(feasible, [])), redirect_stdout(StringIO()):
                self.assertEqual(runner.main(args), 2)
            executor.move_to_part_pose.assert_not_called()
            executor.capture.assert_not_called()
            self.assertEqual(list((output/"images").iterdir()), [])
            report = json.loads((output/"arm_inspection_report.json").read_text())
            self.assertEqual(report["blocked_reason"], "requested_probe_view_infeasible")
            self.assertEqual(report["captures"], [])
            adapter.close.assert_called_once()
            app.close.assert_called_once_with(exit_code=2)

    def test_save_failure_propagates_nonzero_exit_and_no_pass_message(self):
        with tempfile.TemporaryDirectory() as folder:
            _, _, modules, app, _, args = self.fixture(folder)
            stdout, stderr = StringIO(), StringIO()
            with patch.dict(sys.modules, modules), patch.object(runner.importlib.metadata, "version", return_value="6.1.0.0"), \
                    patch.object(runner, "screen_candidates", return_value=((), [])), \
                    patch.object(runner, "write_json", side_effect=OSError("disk full")), redirect_stdout(stdout), redirect_stderr(stderr):
                self.assertEqual(runner.main(args), 1)
            self.assertIn("disk full", stderr.getvalue())
            self.assertNotIn("ARM INSPECTION PASS", stdout.getvalue())
            app.close.assert_called_once_with(exit_code=1)


if __name__ == "__main__":
    unittest.main()
