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
import camera_closed_loop as runner
from executor_loop_fixtures import FakeCell
import test_arm_camera_check as motion_fixtures


class CameraLoopRunnerTests(unittest.TestCase):
    def fixture(self,folder):
        motion_path,output,modules,app,_,_,args = motion_fixtures.CameraRunnerTests().fixture(folder)
        motion = json.loads(motion_path.read_text())
        common = {"run_status":"passed","checks":{"ok":True},"captures":[{},{}],"runtime":motion["runtime"],"model_manifest":{}}
        camera,inspection = Path(folder)/"camera.json",Path(folder)/"inspection.json"
        camera.write_text(json.dumps({**common,"demo_step":"arm_camera_two_rgb_views"}))
        source = lambda path:{"path":str(path),"sha256":hashlib.sha256(path.read_bytes()).hexdigest()}
        inspection.write_text(json.dumps({**common,"demo_step":"arm_inspection_scene_probe",
                                          "cell":source(ROOT/"config/arm_inspection_cell.json"),
                                          "viewpoints":source(ROOT/"config/camera_views.json")}))
        adapter = FakeCell()
        adapter.settings,adapter.camera_settings,adapter.mount = {},{},np.eye(4)
        adapter.close = Mock()
        modules["inspection_loop_adapter"] = SimpleNamespace(InspectionLoopAdapter=Mock(return_value=adapter))
        args += ["--camera-report",str(camera),"--inspection-report",str(inspection)]
        return inspection,output,modules,app,adapter,args

    def test_failed_predecessor_or_existing_output_cannot_start_simulator(self):
        for fault in ("predecessor","output"):
            with tempfile.TemporaryDirectory() as folder:
                source,output,modules,_,_,args = self.fixture(folder)
                if fault == "predecessor":
                    report = json.loads(source.read_text())
                    report["run_status"] = "failed"
                    source.write_text(json.dumps(report))
                else:
                    output.mkdir()
                    (output/"original").write_text("preserve")
                with patch.dict(sys.modules,modules),patch.object(runner.importlib.metadata,"version",return_value="6.1.0.0"),redirect_stderr(StringIO()):
                    self.assertEqual(runner.main(args),1)
                modules["isaacsim"].SimulationApp.assert_not_called()
                if fault == "output":
                    self.assertEqual((output/"original").read_text(),"preserve")

    def test_blocked_session_is_saved_and_exits_two(self):
        with tempfile.TemporaryDirectory() as folder:
            _,output,modules,app,adapter,args = self.fixture(folder)
            session = {"status":"blocked","stop_reason":"initial_view_infeasible","captures":[],"actions_executed":0}
            with patch.dict(sys.modules,modules),patch.object(runner.importlib.metadata,"version",return_value="6.1.0.0"), \
                    patch.object(runner,"run_camera_session",return_value=session),redirect_stdout(StringIO()):
                self.assertEqual(runner.main(args),2)
            report = json.loads((output/"camera_loop_report.json").read_text())
            self.assertEqual(report["run_status"],"blocked")
            self.assertEqual(report["session"]["stop_reason"],"initial_view_infeasible")
            adapter.close.assert_called_once()
            app.close.assert_called_once_with(exit_code=2)

    def test_report_save_failure_cannot_print_pass_or_exit_zero(self):
        with tempfile.TemporaryDirectory() as folder:
            _,_,modules,app,_,args = self.fixture(folder)
            session = {"status":"inspection_observation_satisfied","captures":[{}],"actions_executed":0}
            stdout,stderr = StringIO(),StringIO()
            with patch.dict(sys.modules,modules),patch.object(runner.importlib.metadata,"version",return_value="6.1.0.0"), \
                    patch.object(runner,"run_camera_session",return_value=session), \
                    patch.object(runner,"write_json",side_effect=OSError("disk full")),redirect_stdout(stdout),redirect_stderr(stderr):
                self.assertEqual(runner.main(args),1)
            self.assertNotIn("CAMERA LOOP PASSED",stdout.getvalue())
            self.assertIn("disk full",stderr.getvalue())
            app.close.assert_called_once_with(exit_code=1)


if __name__ == "__main__":
    unittest.main()
