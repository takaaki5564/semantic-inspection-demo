from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import day4_closed_loop
from loop_fixtures import FakeAdapter


class Day4RunnerTests(unittest.TestCase):
    def test_adapter_failure_preserves_nonzero_exit_and_saves_failed_status(self):
        app = SimpleNamespace(close=Mock())
        modules = {"isaacsim": SimpleNamespace(SimulationApp=Mock(return_value=app)),
                   "day4_adapter": SimpleNamespace(ClosedLoopAdapter=Mock(side_effect=RuntimeError("camera unavailable")))}
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "run"
            stdout, stderr = StringIO(), StringIO()
            with patch.dict(sys.modules, modules), redirect_stdout(stdout), redirect_stderr(stderr):
                result = day4_closed_loop.main(["--headless", "--output-dir", str(output)])
            self.assertEqual(result, 1)
            app.close.assert_called_once_with(exit_code=1)
            self.assertIn("DAY4 FAILED", stderr.getvalue())
            self.assertNotIn("DAY4 COMPLETE", stdout.getvalue())
            self.assertEqual(json.loads((output / "closed_loop_report.json").read_text())["run_status"], "failed")

    def test_runner_saves_and_reverifies_actual_steps_and_rejects_existing_directory(self):
        app = SimpleNamespace(close=Mock())
        adapter = FakeAdapter()
        adapter.set_part = Mock()
        adapter.set_part_relative_pose = Mock()
        adapter.hold = Mock()
        adapter.close = Mock()
        modules = {"isaacsim": SimpleNamespace(SimulationApp=Mock(return_value=app)),
                   "day4_adapter": SimpleNamespace(ClosedLoopAdapter=Mock(return_value=adapter))}
        with tempfile.TemporaryDirectory() as root:
            output = Path(root) / "run"
            argv = ["--headless", "--output-dir", str(output)]
            with patch.dict(sys.modules, modules), redirect_stdout(StringIO()):
                self.assertEqual(day4_closed_loop.main(argv), 0)
            app.close.assert_called_once_with(exit_code=0)
            self.assertTrue((output / "events.jsonl").exists())
            self.assertEqual(len(list((output / "images").glob("*.png"))), 4)
            data = (output / "closed_loop_report.json").read_bytes()
            report = json.loads(data)
            self.assertTrue(all(report["acceptance_checks"].values()))
            self.assertEqual(report["session"]["actions_executed"], 1)
            with redirect_stderr(StringIO()):
                self.assertEqual(day4_closed_loop.main(argv), 1)
            self.assertEqual((output / "closed_loop_report.json").read_bytes(), data)


if __name__ == "__main__":
    unittest.main()
