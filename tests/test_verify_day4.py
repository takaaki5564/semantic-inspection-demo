import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from closed_loop import run_session
from inspection_knowledge import InspectionKnowledge
from loop_fixtures import FakeAdapter
from verify_day4 import verify_document
from viewpoint_planner import load_viewpoints


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.output = Path(self.directory.name)
        (self.output / "images").mkdir()
        self.knowledge = InspectionKnowledge.load(ROOT / "knowledge/inspection_knowledge.json")
        self.views, self.weight, self.configuration = load_viewpoints(ROOT / "config/camera_views.json")

    def generate(self, capabilities=("camera_pose_change",), **kwargs):
        def save(rgb, capture, report):
            Image.fromarray(rgb).save(self.output / capture["image"])
        session = run_session(FakeAdapter(), self.knowledge, "Y", self.views, session_id="test_fixture",
                              available_capabilities=capabilities, on_capture=save, **kwargs)
        document = {"run_status": "completed", "demo_step": "day4_closed_loop",
                    "approval_status": "prototype_not_approved", "defect_decision": "not_evaluated",
                    "session": session, "viewpoint_configuration": self.configuration,
                    "knowledge_snapshot": self.knowledge.document, "available_capability_ids": list(capabilities)}
        self.write_events(document)
        return document

    def write_events(self, document):
        (self.output / "events.jsonl").write_text("".join(json.dumps(event) + "\n" for event in document["session"]["events"]))

    def test_actual_motion_and_blocked_runs_replay_without_simulator(self):
        for capabilities in (("camera_pose_change",), ()):
            with self.subTest(capabilities=capabilities):
                document = self.generate(capabilities)
                checks = verify_document(document, self.output)
                self.assertTrue(all(checks.values()))
                document["acceptance_checks"] = checks
                self.assertEqual(checks, verify_document(document, self.output))

    def test_modified_image_is_detected_even_if_resolution_and_color_are_valid(self):
        document = self.generate()
        capture = next(event["capture"] for event in document["session"]["events"] if event["event_type"] == "capture_completed")
        path = self.output / capture["image"]
        with Image.open(path) as image:
            rgb = np.asarray(image).copy()
        rgb[0, 0] = 199
        Image.fromarray(rgb).save(path)
        with self.assertRaisesRegex(ValueError, "did not replay"):
            verify_document(document, self.output)

    def test_plan_execution_observation_and_event_order_tampering_are_rejected(self):
        source = self.generate()
        for problem in ("plan", "executed_pose", "observed_count", "event_order", "missing_receipt", "rgb_hash"):
            document = copy.deepcopy(source)
            events = document["session"]["events"]
            if problem == "plan":
                next(event for event in events if event["event_type"] == "plan_computed")["plan"]["action"]["predicted_new_point_count"] += 1
            elif problem == "executed_pose":
                next(event for event in events if event["event_type"] == "action_completed")["T_world_camera"][0][3] += 0.1
            elif problem == "observed_count":
                next(event for event in events if event["event_type"] == "observation_applied")["state_after_visibility"]["regions"]["R2"]["observed_count"] = 63
            elif problem == "event_order":
                events[1], events[2] = events[2], events[1]
            elif problem == "missing_receipt":
                events[:] = [event for event in events if event["event_type"] != "action_completed"]
            else:
                next(event for event in events if event["event_type"] == "capture_completed")["capture"]["rgb_array_sha256"] = "invalid"
            self.write_events(document)
            with self.subTest(problem=problem), self.assertRaises((ValueError, RuntimeError)):
                verify_document(document, self.output)

    def test_partial_runs_and_divergent_jsonl_are_rejected(self):
        document = self.generate()
        document["run_status"] = "failed"
        with self.assertRaisesRegex(ValueError, "incomplete"):
            verify_document(document, self.output)
        document["run_status"] = "completed"
        (self.output / "events.jsonl").write_text("")
        with self.assertRaisesRegex(ValueError, "JSONL"):
            verify_document(document, self.output)


if __name__ == "__main__":
    unittest.main()
