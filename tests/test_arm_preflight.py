"""Protect preflight evidence and verify real USD composition without starting Kit."""

from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import arm_preflight


class PreflightRunnerTests(unittest.TestCase):
    def setUp(self):
        distribution = SimpleNamespace(version="6.1.0.0")
        patcher = patch.object(arm_preflight.importlib.metadata, "distribution", return_value=distribution)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_load_failure_saves_failed_report_and_nonzero_exit(self):
        app = SimpleNamespace(close=Mock())
        modules = {"isaacsim": SimpleNamespace(SimulationApp=Mock(return_value=app)),
                   "arm_preflight_adapter": SimpleNamespace(load_scene=Mock(side_effect=ValueError("flange missing")))}
        with tempfile.TemporaryDirectory() as root:
            asset, output = Path(root) / "robot.usd", Path(root) / "run"
            asset.touch()
            with patch.dict(sys.modules, modules), redirect_stdout(StringIO()), redirect_stderr(StringIO()):
                result = arm_preflight.main(["--headless", "--robot-usd", str(asset), "--output-dir", str(output)])
            self.assertEqual(result, 1)
            app.close.assert_called_once_with(exit_code=1)
            report = json.loads((output / "arm_preflight_report.json").read_text())
            self.assertEqual(report["run_status"], "failed")
            self.assertIn("flange missing", report["error"])

    def test_existing_output_is_preserved_without_starting_simulator(self):
        start = Mock()
        with tempfile.TemporaryDirectory() as root:
            asset, output = Path(root) / "robot.usd", Path(root) / "run"
            asset.touch()
            output.mkdir()
            evidence = output / "arm_preflight_report.json"
            evidence.write_text('existing evidence\n')
            with patch.dict(sys.modules, {"isaacsim": SimpleNamespace(SimulationApp=start)}), redirect_stderr(StringIO()):
                result = arm_preflight.main(["--robot-usd", str(asset), "--output-dir", str(output)])
            self.assertEqual(result, 1)
            start.assert_not_called()
            self.assertEqual(evidence.read_text(), 'existing evidence\n')


@unittest.skipUnless(importlib.util.find_spec("pxr") and importlib.util.find_spec("isaacsim"),
                     "Run real USD checks in the existing Isaac Sim environment")
class RealUsdTests(unittest.TestCase):
    def test_local_asset_reports_joints_flange_and_instanced_meshes(self):
        from pxr import Usd, UsdGeom
        from arm_preflight_adapter import inspect_robot, ROBOT_PATH

        distribution = arm_preflight.importlib.metadata.distribution("isaacsim")
        asset = Path(distribution.locate_file("isaacsim")) / arm_preflight.TEST_ASSET
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        stage.DefinePrim(ROBOT_PATH, "Xform").GetReferences().AddReference(str(asset))
        result = inspect_robot(stage)
        self.assertTrue(all(result["checks"].values()))
        self.assertEqual(result["joint_count"], 6)
        self.assertGreater(result["mesh_count"], 0)
        self.assertTrue(all(mesh["point_count"] > 0 for mesh in result["meshes"]))
        self.assertEqual(result["flange_prim_path"], ROBOT_PATH + "/wrist_3_link/flange")

    def test_incomplete_robot_cannot_pass(self):
        from pxr import Usd, UsdGeom
        from arm_preflight_adapter import inspect_robot, ROBOT_PATH, FLANGE_PATH

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, ROBOT_PATH)
        UsdGeom.Xform.Define(stage, FLANGE_PATH)
        with self.assertRaisesRegex(ValueError, "USD checks failed"):
            inspect_robot(stage)


if __name__ == "__main__":
    unittest.main()
