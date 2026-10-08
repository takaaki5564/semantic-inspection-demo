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
import arm_model_check
from arm_model_adapter import file_hash, validate_manifest


class ModelEvidenceTests(unittest.TestCase):
    def test_rejected_step1_does_not_create_output_or_start_simulator(self):
        with tempfile.TemporaryDirectory() as root:
            report, output = Path(root) / "step1.json", Path(root) / "step2"
            report.write_text(json.dumps({"demo_step": "arm_step1_load", "run_status": "failed"}))
            with redirect_stderr(StringIO()):
                self.assertEqual(arm_model_check.main(["--step1-report", str(report), "--output-dir", str(output)]), 1)
            self.assertFalse(output.exists())

    def test_existing_output_preserves_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            asset, step1, output = Path(root) / "robot.usd", Path(root) / "step1.json", Path(root) / "step2"
            asset.touch()
            step1.write_text(json.dumps({"demo_step": "arm_step1_load", "run_status": "passed", "robot": {"checks": {"ok": True}},
                                         "runtime": {"isaacsim_package": "6.1.0.0"}, "asset": {"path": str(asset)}}))
            output.mkdir()
            evidence = output / "arm_model_report.json"
            evidence.write_text("existing evidence\n")
            with patch.object(arm_model_check.importlib.metadata, "version", return_value="6.1.0.0"), redirect_stderr(StringIO()):
                self.assertEqual(arm_model_check.main(["--step1-report", str(step1), "--output-dir", str(output)]), 1)
            self.assertEqual(evidence.read_text(), "existing evidence\n")

    def test_changed_model_or_source_layer_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            folder = Path(root)
            for name in ("robot.urdf", "robot.xrdf"):
                (folder / name).write_text(name)
            reference = {"source_layer_sha256": {"source.usd": "original"}, "base_frame": "base", "tool_frame": "flange"}
            manifest = {**reference, "isaacsim_version": "6.1.0.0",
                        "model_file_sha256": {name: file_hash(folder / name) for name in ("robot.urdf", "robot.xrdf")}}
            (folder / "model_manifest.json").write_text(json.dumps(manifest))
            self.assertEqual(validate_manifest(folder, reference, "6.1.0.0"), manifest)
            changed = {**reference, "source_layer_sha256": {"source.usd": "changed"}}
            with self.assertRaisesRegex(ValueError, "source layers"):
                validate_manifest(folder, changed, "6.1.0.0")
            (folder / "robot.urdf").write_text("modified model")
            with self.assertRaisesRegex(ValueError, "Model files changed"):
                validate_manifest(folder, reference, "6.1.0.0")

    def test_report_write_failure_cannot_be_hidden_by_fast_shutdown(self):
        app = SimpleNamespace(close=Mock())
        timeline = SimpleNamespace(is_playing=Mock(return_value=False), get_current_time=Mock(return_value=0.0))
        timeline_module = SimpleNamespace(get_timeline_interface=Mock(return_value=timeline))
        kin = SimpleNamespace(base_frame_name=lambda: "base_link",
                              cspace_coord_limits=lambda i: SimpleNamespace(lower=-1.0, upper=1.0))
        robot = SimpleNamespace(kinematics=kin, controlled_joint_names=["joint"],
                                robot_description=SimpleNamespace(tool_frame_names=lambda: ["flange"]))
        motion = SimpleNamespace(load_cumotion_robot=Mock(return_value=robot))
        modules = {"isaacsim": SimpleNamespace(SimulationApp=Mock(return_value=app)),
                   "isaacsim.robot_motion": SimpleNamespace(cumotion=motion), "isaacsim.robot_motion.cumotion": motion,
                   "omni": SimpleNamespace(timeline=timeline_module), "omni.timeline": timeline_module,
                   "cumotion": SimpleNamespace(__version__="test"),
                   "arm_model_adapter": SimpleNamespace(read_usd_reference=Mock(return_value=(None, {"base_frame": "base_link", "tool_frame": "flange"})),
                                                         export_model=Mock(), validate_manifest=Mock(return_value={}))}
        with tempfile.TemporaryDirectory() as root:
            asset, step1, output = Path(root) / "robot.usd", Path(root) / "step1.json", Path(root) / "step2"
            asset.touch()
            step1.write_text(json.dumps({"demo_step": "arm_step1_load", "run_status": "passed", "robot": {"checks": {"ok": True}},
                                         "runtime": {"isaacsim_package": "6.1.0.0"}, "asset": {"path": str(asset)}}))
            stdout, stderr = StringIO(), StringIO()
            with patch.dict(sys.modules, modules), patch.object(arm_model_check.importlib.metadata, "version", return_value="6.1.0.0"), \
                    patch.object(arm_model_check, "compare_model", return_value={"checks": {"ok": True}, "cases": [{}]}), \
                    patch.object(arm_model_check.json, "dump", side_effect=OSError("disk full")), redirect_stdout(stdout), redirect_stderr(stderr):
                self.assertEqual(arm_model_check.main(["--step1-report", str(step1), "--output-dir", str(output)]), 1)
            app.close.assert_called_once_with(exit_code=1)
            self.assertIn("could not save report", stderr.getvalue())
            self.assertNotIn("UR10E MODEL PASS", stdout.getvalue())


@unittest.skipUnless(importlib.util.find_spec("pxr") and importlib.util.find_spec("isaacsim"), "Requires existing Isaac Sim environment")
class UsdReferenceTests(unittest.TestCase):
    def test_joint_zero_pose_matches_authored_flange_and_frozen_source(self):
        import importlib.metadata
        from arm_kinematics import pose_error, usd_joint_fk
        from arm_model_adapter import read_usd_reference
        from arm_preflight import TEST_ASSET

        distribution = importlib.metadata.distribution("isaacsim")
        asset = Path(distribution.locate_file("isaacsim")) / TEST_ASSET
        stage, reference = read_usd_reference(asset)
        error = pose_error(usd_joint_fk(reference, dict.fromkeys([j["name"] for j in reference["joints"]], 0.0)),
                           reference["T_base_flange_authored"])
        self.assertLess(error["position_error_m"], 1e-5)
        self.assertLess(error["orientation_error_rad"], 1e-5)
        validate_manifest(ROOT / "config/robots/ur10e", reference, distribution.version)


if __name__ == "__main__":
    unittest.main()
