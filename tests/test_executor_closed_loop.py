from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from executor_closed_loop import run_camera_session
from executor_loop_fixtures import FakeCell,FakeExecutor
from inspection_knowledge import InspectionKnowledge
from viewpoint_planner import load_viewpoints


class ExecutorLoopTests(unittest.TestCase):
    def setUp(self):
        self.knowledge = InspectionKnowledge.load(ROOT/"knowledge/inspection_knowledge.json")
        self.views,self.weight,_ = load_viewpoints(ROOT/"config/camera_views.json")

    def run_loop(self,adapter=None,spec="Y",views=None,**kwargs):
        adapter = adapter or FakeCell()
        return run_camera_session(adapter,FakeExecutor(adapter),self.knowledge,spec,
                                   self.views[:2] if views is None else views,self.views[0],
                                   rotation_cost_m_per_rad=self.weight,**kwargs)

    def test_B_Y_plans_from_actual_observation_and_captures_selected_view(self):
        adapter = FakeCell()
        result = self.run_loop(adapter)
        self.assertEqual(result["status"],"inspection_observation_satisfied")
        self.assertEqual(result["actions_executed"],1)
        self.assertEqual(result["initial_positioning_count"],1)
        self.assertEqual(result["selected_view_ids"],["rear"])
        self.assertEqual(len(result["captures"]),2)
        self.assertEqual(result["captures"][0]["state_after_visibility"]["regions"]["R2"]["observed_count"],3)
        self.assertEqual(result["captures"][1]["actual_new_point_count"],60)
        plan = next(e["plan"] for e in result["events"] if e["event_type"] == "plan_computed")
        self.assertEqual(plan["observation_state"]["regions"]["R2"]["observed_count"],3)
        self.assertIn("candidate_specific_scene",plan["prediction_geometry_policy"])

    def test_A_Y_and_B_X_stop_after_initial_capture_without_additional_IK(self):
        for adapter,spec in ((FakeCell(.025),"Y"),(FakeCell(),"X")):
            result = self.run_loop(adapter,spec)
            self.assertEqual(result["actions_executed"],0)
            self.assertEqual(len(result["captures"]),1)
            self.assertEqual(adapter.prepare_count,1)
            self.assertEqual(result["status"],"inspection_observation_satisfied")

    def test_infeasible_initial_and_remaining_views_have_distinct_blocked_reasons(self):
        initial = FakeCell(blocked=("front",))
        stopped = self.run_loop(initial)
        self.assertEqual(stopped["stop_reason"],"initial_view_infeasible")
        self.assertEqual(initial.capture_count,0)
        self.assertEqual(initial.move_count,0)
        later = FakeCell(blocked=("rear",))
        stopped = self.run_loop(later)
        self.assertEqual(stopped["stop_reason"],"no_reachable_candidate_views")
        self.assertEqual(later.capture_count,1)
        self.assertEqual(later.move_count,1)

    def test_capability_and_action_budget_stop_after_initial_setup_only(self):
        for options,reason in (({"available_capabilities":()},"required_capability_unavailable"),
                               ({"max_actions":0},"max_actions_reached"),({"views":()},"no_candidate_views")):
            adapter = FakeCell()
            result = self.run_loop(adapter,**options)
            self.assertEqual(result["stop_reason"],reason)
            self.assertEqual(adapter.capture_count,1)
            self.assertEqual(adapter.move_count,1)

    def test_blocked_or_inaccurate_movement_never_triggers_planned_capture(self):
        for fault,reason in (("blocked_move","fixture_motion_timeout"),("ignored_motion","measured_camera_pose_error")):
            adapter = FakeCell(fault=fault)
            result = self.run_loop(adapter)
            self.assertEqual(result["stop_reason"],reason)
            self.assertEqual(adapter.capture_count,1)
            self.assertEqual(result["actions_executed"],0)

    def test_actual_arm_occlusion_overrides_optimistic_prediction(self):
        result = self.run_loop(FakeCell(fault="actual_arm_occlusion"))
        self.assertEqual(result["status"],"blocked")
        self.assertEqual(result["stop_reason"],"executed_view_did_not_improve_observation")
        self.assertEqual(result["actions_executed"],1)
        self.assertEqual(result["captures"][1]["actual_new_point_count"],0)
        self.assertGreater(len(result["captures"][1]["visibility"]["robot_occluded_point_ids"]["R2"]),0)

    def test_stale_frames_and_changed_static_scene_fail_before_second_observation(self):
        for fault in ("stale_reference","stale_rgb","wrong_capture_pose","scene_changed"):
            events = []
            with self.subTest(fault=fault), self.assertRaises((ValueError,RuntimeError)):
                self.run_loop(FakeCell(fault=fault),on_event=events.append)
            self.assertEqual(events[-1]["event_type"],"session_failed")
            self.assertEqual(events[-1]["state"]["regions"]["R2"]["observed_count"],3)

    def test_new_spec_resets_state_and_event_callback_cannot_rewrite_history(self):
        adapter = FakeCell()
        self.run_loop(adapter)
        result = self.run_loop(adapter,"X",on_event=lambda e:e.update(event_type="tampered"))
        self.assertEqual(result["events"][0]["event_type"],"session_started")
        self.assertEqual(result["events"][0]["state"]["capture_ids"],[])
        self.assertEqual(set(result["final_state"]["regions"]),{"R1"})

    def test_invalid_action_budget_is_rejected_before_any_movement(self):
        for budget in (-1,1.5,True):
            adapter = FakeCell()
            with self.assertRaises(ValueError):
                self.run_loop(adapter,max_actions=budget)
            self.assertEqual(adapter.move_count,0)


if __name__ == "__main__":
    unittest.main()
