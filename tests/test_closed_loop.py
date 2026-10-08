import copy
from fractions import Fraction
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from capture_metadata import camera_look_at
from closed_loop import run_session
from inspection_knowledge import InspectionKnowledge
from loop_fixtures import FakeAdapter
from viewpoint_planner import CandidateView, load_viewpoints


class ClosedLoopTests(unittest.TestCase):
    def setUp(self):
        self.knowledge = InspectionKnowledge.load(ROOT / "knowledge/inspection_knowledge.json")
        self.views, self.weight, _ = load_viewpoints(ROOT / "config/camera_views.json")

    def run_loop(self, adapter=None, spec="Y", **kwargs):
        return run_session(adapter or FakeAdapter(), self.knowledge, spec, self.views,
                           rotation_cost_m_per_rad=self.weight, **kwargs)

    def test_B_Y_executes_predicted_view_then_updates_from_new_capture_and_replans(self):
        adapter = FakeAdapter()
        result = self.run_loop(adapter)
        self.assertEqual(result["status"], "inspection_observation_satisfied")
        self.assertEqual(result["actions_executed"], 1)
        self.assertEqual(adapter.capture_count, 2)
        observations = [event for event in result["events"] if event["event_type"] == "observation_applied"]
        self.assertEqual(observations[0]["state_after_visibility"]["regions"]["R2"]["observed_count"], 3)
        self.assertEqual(observations[1]["state_after_visibility"]["regions"]["R2"]["observed_count"], 63)
        self.assertEqual(observations[1]["actual_new_point_count"], 60)
        plans = [event["plan"] for event in result["events"] if event["event_type"] == "plan_computed"]
        self.assertEqual(plans[0]["action"]["execution_status"], "not_executed")
        self.assertEqual(plans[0]["observation_state"]["regions"]["R2"]["observed_count"], 3)
        self.assertEqual(plans[1]["status"], "inspection_observation_satisfied")
        self.assertEqual(result["final_state"]["defect_decision"], "not_evaluated")

    def test_A_Y_and_B_X_stop_without_camera_motion_or_extra_capture(self):
        for adapter, spec in ((FakeAdapter(0.025), "Y"), (FakeAdapter(), "X")):
            with self.subTest(spec=spec):
                result = self.run_loop(adapter, spec)
                self.assertEqual(result["status"], "inspection_observation_satisfied")
                self.assertEqual(result["actions_executed"], 0)
                self.assertEqual(adapter.capture_count, 1)

    def test_no_motion_capability_is_honestly_blocked_after_initial_capture(self):
        adapter = FakeAdapter()
        result = self.run_loop(adapter, available_capabilities=())
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(result["stop_reason"], "required_capability_unavailable")
        self.assertEqual(adapter.moves, [])
        self.assertFalse(result["final_state"]["inspection_observation_satisfied"])

    def test_zero_action_budget_stops_before_executing_ready_plan(self):
        adapter = FakeAdapter()
        result = self.run_loop(adapter, max_actions=0)
        self.assertEqual(result["stop_reason"], "max_actions_reached")
        self.assertEqual(adapter.moves, [])
        self.assertEqual(adapter.capture_count, 1)

    def test_multi_step_replans_remaining_points_and_honors_budget(self):
        document = copy.deepcopy(self.knowledge.document)
        document["knowledge_version"] = "test_additional_cause_v1"
        document["missing_observation_causes"]["outside_frustum"] = {"eligible_method_ids": ["change_viewpoint"]}
        knowledge = InspectionKnowledge(document)
        views = (CandidateView("left", camera_look_at((-.2, .12, .35), (-.2, .17, .06))),
                 CandidateView("right", camera_look_at((.3, .12, .35), (.3, .17, .06))))
        limited = run_session(FakeAdapter(), knowledge, "Y", views, max_actions=1)
        self.assertEqual(limited["stop_reason"], "max_actions_reached")
        self.assertEqual(limited["actions_executed"], 1)
        longer = run_session(FakeAdapter(), knowledge, "Y", views, max_actions=3)
        self.assertEqual(longer["actions_executed"], 2)
        self.assertEqual(longer["stop_reason"], "no_candidate_views")
        receipts = [event["view_id"] for event in longer["events"] if event["event_type"] == "action_completed"]
        self.assertEqual(len(receipts), len(set(receipts)))
        self.assertFalse(longer["final_state"]["inspection_observation_satisfied"])

    def test_stale_time_image_wrong_pose_and_scene_change_fail_before_applying_observation(self):
        for fault, message in (("stale_reference", "reference time"), ("stale_rgb", "RGB unchanged"),
                               ("wrong_capture_pose", "Captured camera"), ("scene_changed", "Scene"),
                               ("ignored_motion", "Executed camera")):
            events = []
            with self.subTest(fault=fault), self.assertRaisesRegex(RuntimeError, message):
                self.run_loop(FakeAdapter(fault=fault), on_event=events.append)
            self.assertEqual(events[-1]["event_type"], "session_failed")
            self.assertEqual(events[-1]["state"]["regions"]["R2"]["observed_count"], 3)
            self.assertEqual(sum(event["event_type"] == "observation_applied" for event in events), 1)

    def test_prediction_success_does_not_override_actual_zero_gain(self):
        result = self.run_loop(FakeAdapter(fault="no_actual_gain"))
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(result["stop_reason"], "executed_view_did_not_improve_observation")
        self.assertEqual(result["actions_executed"], 1)
        self.assertEqual(result["final_state"]["regions"]["R2"]["observed_count"], 3)

    def test_part_transform_uses_same_relative_plan_and_actual_world_readback(self):
        transform = np.eye(4)
        transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        transform[:3, 3] = (2.5, -1.5, 0.4)
        first = self.run_loop()
        second = self.run_loop(FakeAdapter(world_part=transform))
        first_receipt = next(event for event in first["events"] if event["event_type"] == "action_completed")
        second_receipt = next(event for event in second["events"] if event["event_type"] == "action_completed")
        self.assertEqual(first_receipt["view_id"], second_receipt["view_id"])
        np.testing.assert_allclose(first_receipt["T_part_camera"], second_receipt["T_part_camera"])
        np.testing.assert_allclose(transform @ np.asarray(second_receipt["T_part_camera"]), second_receipt["T_world_camera"])

    def test_each_new_session_resets_state_and_event_callbacks_cannot_mutate_history(self):
        adapter = FakeAdapter()
        y = self.run_loop(adapter)
        def tamper(event):
            event["event_type"] = "tampered"
        x = self.run_loop(adapter, "X", on_event=tamper)
        start = x["events"][0]
        self.assertEqual(start["event_type"], "session_started")
        self.assertEqual(start["state"]["status"], "not_captured")
        self.assertEqual(start["state"]["capture_ids"], [])
        self.assertEqual(set(x["final_state"]["regions"]), {"R1"})
        self.assertNotEqual(y["inspection_spec_version"], x["inspection_spec_version"])

    def test_optional_reference_floor_and_invalid_action_budgets(self):
        with self.assertRaisesRegex(RuntimeError, "Stale RGB"):
            self.run_loop(previous_reference=Fraction(10, 60))
        for invalid in (-1, 1.5, True):
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                self.run_loop(max_actions=invalid)


if __name__ == "__main__":
    unittest.main()
