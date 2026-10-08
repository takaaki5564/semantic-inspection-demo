from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import day3_plan
from geometry_fixtures import demo_camera, demo_scene
from inspection_knowledge import InspectionKnowledge
from viewpoint_planner import load_viewpoints


def saved_fixture():
    cases = []
    for name, height in (("A", 0.025), ("B", 0.12)):
        scene, regions = demo_scene(height)
        cases.append({"case_id": f"{name}_initial", "scene": scene.to_dict(),
                      "surface_regions": {key: value.to_dict() for key, value in regions.items()},
                      "camera": demo_camera().to_dict(),
                      "capture": {"evidence_source": "test_fixture_only", "status": "captured"},
                      "visibility": {"max_incidence_angle_deg": 75.0, "surface_tolerance_m": 1e-5}})
    return {"cases": cases}


class Day3RunnerTests(unittest.TestCase):
    def test_full_comparison_and_repeat_replay_are_deterministic(self):
        knowledge = InspectionKnowledge.load(ROOT / "knowledge/inspection_knowledge.json")
        candidates, weight, _ = load_viewpoints(ROOT / "config/camera_views.json")
        document = saved_fixture()
        first = day3_plan.run_comparison(document, knowledge, candidates, weight)
        self.assertEqual(first, day3_plan.run_comparison(document, knowledge, candidates, weight))
        self.assertTrue(all(first["acceptance_checks"].values()))
        self.assertEqual(first["cases"]["B_Y"]["state_after_planning"]["regions"]["R2"]["observed_count"], 3)
        self.assertEqual(first["cases"]["B_Y"]["plan"]["action"]["predicted_new_point_count"], 60)

    def test_rejected_day2_evidence_fails_without_creating_output(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            stdout, stderr = StringIO(), StringIO()
            with patch.object(day3_plan, "verify", side_effect=ValueError("visibility replay differs")):
                with redirect_stdout(stdout), redirect_stderr(stderr):
                    code = day3_plan.main(["--input-dir", directory, "--output-dir", str(output)])
            self.assertEqual(code, 1)
            self.assertFalse(output.exists())
            self.assertIn("DAY3 FAILED", stderr.getvalue())
            self.assertNotIn("DAY3 COMPLETE", stdout.getvalue())

    def test_cli_writes_predictions_and_never_overwrites_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "visibility_report.json").write_text(json.dumps(saved_fixture()))
            output = root / "run"
            argv = ["--input-dir", directory, "--output-dir", str(output)]
            # Fixture input is deliberately synthetic: only the real CLI invokes Day 2 verification.
            with patch.object(day3_plan, "verify") as verify, redirect_stdout(StringIO()):
                self.assertEqual(day3_plan.main(argv), 0)
                verify.assert_called_once_with(root)
            report_path = output / "plan_report.json"
            before = report_path.read_bytes()
            document = json.loads(before)
            self.assertEqual(document["input_evidence"]["additional_captures"], 0)
            self.assertEqual(document["execution_status"], "not_executed")
            self.assertTrue(all(document["acceptance_checks"].values()))
            self.assertFalse((output / "images").exists())
            with redirect_stderr(StringIO()), redirect_stdout(StringIO()):
                self.assertEqual(day3_plan.main(argv), 1)
            self.assertEqual(report_path.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
