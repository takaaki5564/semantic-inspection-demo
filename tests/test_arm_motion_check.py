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
import arm_motion_check


class MotionEvidenceTests(unittest.TestCase):
    def fixture(self, folder):
        folder = Path(folder)
        asset, model_dir, report_path, output = folder/"robot.usd", folder/"model", folder/"step2.json", folder/"output"
        asset.touch()
        model_dir.mkdir()
        report = {"demo_step": "arm_step2_model_check", "run_status": "passed", "comparison": {"checks": {"ok": True}},
                  "runtime": {"isaacsim_package": "6.1.0.0", "cumotion_version": "test"},
                  "asset_path": str(asset), "model_directory": str(model_dir), "model_manifest": {},
                  "usd_reference": {"source_layer_sha256": {}}}
        report_path.write_text(json.dumps(report))
        adapter = SimpleNamespace(settings={}, limits={}, read_state=lambda: {"simulation_time_s":0})
        app = SimpleNamespace(close=Mock())
        model = SimpleNamespace(read_usd_reference=lambda asset: (None, {"source_layer_sha256": {}}), validate_manifest=Mock(return_value={}))
        modules = {"isaacsim": SimpleNamespace(SimulationApp=Mock(return_value=app)), "cumotion": SimpleNamespace(__version__="test"),
                   "arm_model_adapter": model, "arm_motion_adapter": SimpleNamespace(ArmMotionAdapter=Mock(return_value=adapter))}
        args = ["--step2-report",str(report_path),"--output-dir",str(output),"--headless"]
        return report_path, output, modules, app, args

    def test_existing_output_is_preserved_without_starting_simulator(self):
        with tempfile.TemporaryDirectory() as folder:
            _, output, modules, _, args = self.fixture(folder)
            output.mkdir()
            evidence=output/"arm_motion_report.json"
            evidence.write_text("existing evidence\n")
            with patch.dict(sys.modules, modules), patch.object(arm_motion_check.importlib.metadata,"version",return_value="6.1.0.0"), redirect_stderr(StringIO()):
                self.assertEqual(arm_motion_check.main(args),1)
            self.assertEqual(evidence.read_text(),"existing evidence\n")
            modules["isaacsim"].SimulationApp.assert_not_called()

    def test_blocked_first_goal_prevents_second_goal(self):
        with tempfile.TemporaryDirectory() as folder:
            _, output, modules, app, args = self.fixture(folder)
            execute=Mock(return_value={"goal_id":"goal_01","status":"blocked","reason":"motion_timeout","physics_steps":3,"max_actual_joint_displacement_rad":0})
            with patch.dict(sys.modules, modules), patch.object(arm_motion_check.importlib.metadata,"version",return_value="6.1.0.0"), \
                    patch.object(arm_motion_check,"execute_goal",execute), redirect_stdout(StringIO()):
                self.assertEqual(arm_motion_check.main(args),2)
            execute.assert_called_once()
            report=json.loads((output/"arm_motion_report.json").read_text())
            self.assertEqual(report["run_status"],"blocked")
            self.assertFalse(report["checks"]["two_goals_reached"])
            app.close.assert_called_once_with(exit_code=2)

    def test_report_persistence_error_cannot_print_pass_or_exit_zero(self):
        with tempfile.TemporaryDirectory() as folder:
            _, _, modules, app, args = self.fixture(folder)
            execute=Mock(return_value={"goal_id":"test","status":"reached","physics_steps":10,"max_actual_joint_displacement_rad":.3,
                                       "settled_seconds":.25,"final_error":{"position_error_m":0,"orientation_error_rad":0}})
            stdout,stderr=StringIO(),StringIO()
            with patch.dict(sys.modules, modules), patch.object(arm_motion_check.importlib.metadata,"version",return_value="6.1.0.0"), \
                    patch.object(arm_motion_check,"execute_goal",execute), patch.object(arm_motion_check.json,"dump",side_effect=OSError("disk full")), \
                    redirect_stdout(stdout),redirect_stderr(stderr):
                self.assertEqual(arm_motion_check.main(args),1)
            self.assertNotIn("UR10E MOTION PASS",stdout.getvalue())
            self.assertIn("disk full",stderr.getvalue())
            app.close.assert_called_once_with(exit_code=1)

    def test_changed_model_is_rejected_before_controller_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            _, output, modules, app, args = self.fixture(folder)
            modules["arm_model_adapter"].validate_manifest.return_value={"changed":True}
            with patch.dict(sys.modules, modules), patch.object(arm_motion_check.importlib.metadata,"version",return_value="6.1.0.0"), \
                    redirect_stdout(StringIO()),redirect_stderr(StringIO()):
                self.assertEqual(arm_motion_check.main(args),1)
            modules["arm_motion_adapter"].ArmMotionAdapter.assert_not_called()
            self.assertEqual(json.loads((output/"arm_motion_report.json").read_text())["run_status"],"failed")
            app.close.assert_called_once_with(exit_code=1)


if __name__ == "__main__":
    unittest.main()
