from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import day2_visibility


class Day2RunnerTests(unittest.TestCase):
    def test_adapter_failure_does_not_claim_success_or_hide_exit_status(self):
        app = SimpleNamespace(close=Mock())
        fake_modules = {
            "isaacsim": SimpleNamespace(SimulationApp=Mock(return_value=app)),
            "day2_adapter": SimpleNamespace(VisibilityAdapter=Mock(side_effect=RuntimeError("snapshot unavailable"))),
        }
        with tempfile.TemporaryDirectory() as root:
            stderr, stdout = StringIO(), StringIO()
            argv = ["day2_visibility.py", "--headless", "--output-dir", str(Path(root) / "run")]
            with patch.dict(sys.modules, fake_modules), patch.object(sys, "argv", argv):
                with redirect_stderr(stderr), redirect_stdout(stdout):
                    result = day2_visibility.main()
            self.assertEqual(result, 1)
            app.close.assert_called_once_with(exit_code=1)
            self.assertIn("DAY2 FAILED", stderr.getvalue())
            self.assertNotIn("DAY2 COMPLETE", stdout.getvalue())
            self.assertFalse((Path(root) / "run/visibility_report.json").exists())


if __name__ == "__main__":
    unittest.main()
