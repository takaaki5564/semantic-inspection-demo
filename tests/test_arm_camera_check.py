from contextlib import redirect_stderr,redirect_stdout
from io import StringIO
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
import arm_camera_check


class CameraRunnerTests(unittest.TestCase):
    def fixture(self,folder):
        folder = Path(folder)
        asset,model,path,output = folder/"robot.usd",folder/"model",folder/"step3.json",folder/"output"
        asset.touch()
        model.mkdir()
        source = folder/"goals.json"
        source.write_text(json.dumps({"goals":[{"T_base_flange_target":np.eye(4).tolist()}]*2}))
        evidence = {"demo_step":"arm_step3_controlled_motion","run_status":"passed","checks":{"ok":True},
                    "goals":[{"status":"reached"}]*2,"runtime":{"isaacsim_package":"6.1.0.0","cumotion_version":"test"},
                    "goals_source":{"path":str(source),"sha256":hashlib.sha256(source.read_bytes()).hexdigest()},
                    "asset_path":str(asset),"model_directory":str(model),"model_manifest":{}}
        path.write_text(json.dumps(evidence))
        adapter = SimpleNamespace(settings={},camera_settings={},world_part=np.eye(4),mount=np.eye(4),
                                  read_state=lambda:{"T_world_base":np.eye(4).tolist()},close=Mock())
        executor = SimpleNamespace(move_to_part_pose=Mock(return_value={"status":"blocked","reason":"ik_failed"}),capture=Mock())
        app = SimpleNamespace(close=Mock())
        modules = {"isaacsim":SimpleNamespace(SimulationApp=Mock(return_value=app)),"cumotion":SimpleNamespace(__version__="test"),
                   "arm_model_adapter":SimpleNamespace(read_usd_reference=Mock(return_value=(None,{})),validate_manifest=Mock(return_value={})),
                   "arm_camera_adapter":SimpleNamespace(ArmCameraAdapter=Mock(return_value=adapter))}
        args = ["--step3-report",str(path),"--output-dir",str(output),"--headless"]
        return path,output,modules,app,adapter,executor,args

    def test_failed_step3_or_changed_goal_source_never_starts_simulator(self):
        for case in ("failed","changed"):
            with tempfile.TemporaryDirectory() as folder:
                path,output,modules,_,_,_,args = self.fixture(folder)
                evidence = json.loads(path.read_text())
                if case == "failed":
                    evidence["run_status"] = "failed"
                    path.write_text(json.dumps(evidence))
                else:
                    Path(evidence["goals_source"]["path"]).write_text("changed")
                with patch.dict(sys.modules,modules),patch.object(arm_camera_check.importlib.metadata,"version",return_value="6.1.0.0"),redirect_stderr(StringIO()):
                    self.assertEqual(arm_camera_check.main(args),1)
                self.assertFalse(output.exists())
                modules["isaacsim"].SimulationApp.assert_not_called()

    def test_existing_output_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            _,output,modules,_,_,_,args = self.fixture(folder)
            output.mkdir()
            file = output/"camera_poses.json"
            file.write_text("original evidence\n")
            with patch.dict(sys.modules,modules),patch.object(arm_camera_check.importlib.metadata,"version",return_value="6.1.0.0"),redirect_stderr(StringIO()):
                self.assertEqual(arm_camera_check.main(args),1)
            self.assertEqual(file.read_text(),"original evidence\n")
            modules["isaacsim"].SimulationApp.assert_not_called()

    def test_blocked_movement_prevents_capture_and_second_move(self):
        with tempfile.TemporaryDirectory() as folder:
            _,output,modules,app,adapter,executor,args = self.fixture(folder)
            with patch.dict(sys.modules,modules),patch.object(arm_camera_check.importlib.metadata,"version",return_value="6.1.0.0"), \
                    patch.object(arm_camera_check,"ArmMountedCameraExecutor",return_value=executor),redirect_stdout(StringIO()):
                self.assertEqual(arm_camera_check.main(args),2)
            executor.move_to_part_pose.assert_called_once()
            executor.capture.assert_not_called()
            report = json.loads((output/"arm_camera_report.json").read_text())
            self.assertEqual(report["run_status"],"blocked")
            self.assertEqual(report["captures"],[])
            self.assertFalse(report["checks"]["two_fresh_distinct_images"])
            adapter.close.assert_called_once()
            app.close.assert_called_once_with(exit_code=2)

    def test_save_error_cannot_print_pass_or_exit_zero(self):
        with tempfile.TemporaryDirectory() as folder:
            _,_,modules,app,_,executor,args = self.fixture(folder)
            stdout,stderr = StringIO(),StringIO()
            with patch.dict(sys.modules,modules),patch.object(arm_camera_check.importlib.metadata,"version",return_value="6.1.0.0"), \
                    patch.object(arm_camera_check,"ArmMountedCameraExecutor",return_value=executor), \
                    patch.object(arm_camera_check.json,"dump",side_effect=OSError("disk full")),redirect_stdout(stdout),redirect_stderr(stderr):
                self.assertEqual(arm_camera_check.main(args),1)
            self.assertNotIn("WRIST CAMERA PASS",stdout.getvalue())
            self.assertIn("disk full",stderr.getvalue())
            app.close.assert_called_once_with(exit_code=1)


if __name__ == "__main__":
    unittest.main()
