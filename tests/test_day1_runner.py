"""Failure must remain visible even when SimulationApp performs fast shutdown."""

from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import day1_capture


class RunnerTests(unittest.TestCase):
    def test_sensor_failure_logs_traceback_and_preserves_nonzero_exit(self):
        app = SimpleNamespace(close=Mock())
        fake_modules = {
            "isaacsim": SimpleNamespace(SimulationApp=Mock(return_value=app)),
            "sim_adapter": SimpleNamespace(CaptureAdapter=Mock(side_effect=RuntimeError("sensor unavailable"))),
        }
        with tempfile.TemporaryDirectory() as root:
            argv = ["day1_capture.py", "--headless", "--output-dir", str(Path(root) / "failed_run")]
            stderr, stdout = StringIO(), StringIO()
            with patch.dict(sys.modules, fake_modules), patch.object(sys, "argv", argv):
                with redirect_stderr(stderr), redirect_stdout(stdout):
                    result = day1_capture.main()
            self.assertEqual(result, 1)
            app.close.assert_called_once_with(exit_code=1)
            self.assertIn("DAY1 CAPTURE FAILED", stderr.getvalue())
            self.assertIn("sensor unavailable", stderr.getvalue())
            self.assertNotIn("DAY1 CAPTURE COMPLETE", stdout.getvalue())
            self.assertFalse((Path(root) / "failed_run" / "camera_poses.json").exists())


if __name__ == "__main__":
    unittest.main()
