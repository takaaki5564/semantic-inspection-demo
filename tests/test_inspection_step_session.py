from dataclasses import replace
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from capture_metadata import camera_look_at
from executor_closed_loop import CameraInspectionSession, run_camera_session
from executor_loop_fixtures import FakeCell, FakeExecutor
from inspection_knowledge import InspectionKnowledge
from inspection_region_state import load_region_metadata, region_snapshot
from inspection_state import ObservationState
from viewpoint_planner import load_viewpoints
from visibility import evaluate_visibility


class StepSessionTests(unittest.TestCase):
    def setUp(self):
        self.knowledge = InspectionKnowledge.load(ROOT/"knowledge/inspection_knowledge.json")
        self.views, self.weight, _ = load_viewpoints(ROOT/"config/camera_views.json")

    def make(self, adapter=None, spec="Y", **options):
        adapter = adapter or FakeCell()
        return CameraInspectionSession(adapter, FakeExecutor(adapter), self.knowledge, spec,
                                       self.views[:2], self.views[0], rotation_cost_m_per_rad=self.weight, **options)

    def test_one_step_stops_after_first_saved_observation_and_exposes_only_R2_target(self):
        cell = FakeCell()
        session = self.make(cell)
        self.assertEqual([r["state"] for r in session.snapshot()["region_states"]], ["unconfirmed"]*2)
        first = session.step()
        self.assertEqual((cell.capture_count, cell.move_count, first["status"]), (1, 1, "paused"))
        self.assertEqual(first["next_plan"]["action"]["view_id"], "rear")
        r1, r2 = first["region_states"]
        self.assertEqual((r1["state"], r1["next_capture_target"]), ("confirmed", False))
        self.assertEqual((r2["state"], r2["observed_count"], r2["next_capture_target"]), ("partial", 3, True))
        second = session.step()
        self.assertEqual(second["status"], "inspection_observation_satisfied")
        self.assertTrue(all(r["state"] == "confirmed" and not r["next_capture_target"] for r in second["region_states"]))
        session.step()
        self.assertEqual(cell.capture_count, 2)

    def test_stepped_and_continuous_results_and_event_histories_are_identical(self):
        for height, spec in ((.35, "Y"), (.025, "Y"), (.35, "X")):
            cell = FakeCell(height)
            stepped = self.make(cell, spec)
            while not stepped.done:
                stepped.step()
            other = FakeCell(height)
            continuous = run_camera_session(other, FakeExecutor(other), self.knowledge, spec,
                                             self.views[:2], self.views[0], rotation_cost_m_per_rad=self.weight)
            self.assertEqual(stepped.report(), continuous)

    def test_save_failure_preserves_previous_valid_observation_and_clears_next_target(self):
        def save(rgb, record, snapshot):
            if record["capture_id"] == "capture_01":
                raise OSError("fixture disk full")
        session = self.make(on_capture=save)
        session.step()
        previous = session.state.summary()
        with self.assertRaisesRegex(OSError, "disk full"):
            session.step()
        self.assertEqual(session.state.summary(), previous)
        self.assertEqual(len(session.report()["captures"]), 1)
        self.assertEqual(session.events[-1]["failure_phase"], "saving")
        self.assertEqual(session.snapshot()["status"], "failed")
        self.assertFalse(any(r["next_capture_target"] for r in session.snapshot()["region_states"]))
        self.assertEqual(sum(e["event_type"] == "capture_saved" for e in session.events), 1)

    def test_validation_save_and_observation_events_are_ordered_and_detached(self):
        events = []
        session = self.make(on_capture=lambda *args: None, on_event=events.append)
        snapshot = session.step()
        kinds = [e["event_type"] for e in events]
        self.assertLess(kinds.index("capture_validated"), kinds.index("capture_saved"))
        self.assertLess(kinds.index("capture_saved"), kinds.index("observation_applied"))
        snapshot["region_states"][1]["observed_count"] = 63
        snapshot["next_plan"]["action"]["view_id"] = "tampered"
        events[0]["event_type"] = "tampered"
        self.assertEqual(session.snapshot()["region_states"][1]["observed_count"], 3)
        self.assertEqual(session.snapshot()["next_plan"]["action"]["view_id"], "rear")
        self.assertEqual(session.events[0]["event_type"], "session_started")

    def test_interrupted_save_ends_session_without_committing_observation(self):
        def interrupt(*args):
            raise KeyboardInterrupt()
        session = self.make(on_capture=interrupt)
        with self.assertRaises(KeyboardInterrupt):
            session.step()
        snapshot = session.snapshot()
        self.assertEqual((snapshot["status"], snapshot["capture_count"]), ("failed", 0))
        self.assertEqual(snapshot["stop_reason"], "KeyboardInterrupt")
        self.assertEqual([r["state"] for r in snapshot["region_states"]], ["unconfirmed"]*2)
        self.assertTrue(session.done)

    def test_scene_or_camera_change_during_pause_is_rejected_before_motion(self):
        for change in ("part", "camera"):
            cell = FakeCell()
            session = self.make(cell)
            session.step()
            if change == "camera":
                cell.move_to_part_pose(self.views[1].T_part_camera)
            else:
                cell.scene = FakeCell(.025).scene
            moves = cell.move_count
            with self.assertRaises((ValueError, RuntimeError)):
                session.step()
            self.assertEqual(cell.move_count, moves)
            self.assertEqual(cell.capture_count, 1)
            self.assertEqual(session.snapshot()["status"], "failed")

    def test_reentrant_step_cannot_execute_duplicate_motion(self):
        rejected = []
        def event_callback(event):
            if event["event_type"] == "movement_started":
                with self.assertRaisesRegex(RuntimeError, "already running"):
                    session.step()
                rejected.append(True)
        session = self.make(on_event=event_callback)
        session.step()
        self.assertEqual(rejected, [True])
        self.assertEqual(len(session.captures), 1)

    def test_stop_requested_during_move_finishes_current_capture_then_stops(self):
        def event_callback(event):
            if event["event_type"] == "movement_started":
                session.request_stop()
        session = self.make(on_event=event_callback)
        state = session.step()
        self.assertEqual((state["status"], state["stop_reason"], state["capture_count"]), ("stopped", "stop_requested", 1))
        self.assertIsNone(state["next_plan"])
        self.assertFalse(any(r["next_capture_target"] for r in state["region_states"]))

    def test_stop_while_paused_does_not_move_again_and_new_session_resets(self):
        cell = FakeCell()
        old = self.make(cell, session_id="old")
        old.step()
        old.request_stop()
        old.step()
        self.assertEqual(cell.move_count, 1)
        new = self.make(cell, "X", session_id="new")
        snapshot = new.snapshot()
        self.assertEqual(snapshot["session_id"], "new")
        self.assertEqual(snapshot["capture_count"], 0)
        self.assertIsNone(snapshot["next_plan"])
        self.assertEqual([r["state"] for r in snapshot["region_states"]], ["unconfirmed", "not_required"])
        self.assertIsNone(snapshot["region_states"][1]["coverage"])

    def test_blocked_session_keeps_partial_state_instead_of_turning_all_regions_red(self):
        session = self.make(available_capabilities=())
        state = session.step()
        self.assertEqual(state["status"], "blocked")
        self.assertEqual([r["state"] for r in state["region_states"]], ["confirmed", "partial"])
        self.assertFalse(any(r["next_capture_target"] for r in state["region_states"]))

    def test_region_capture_alone_zero_valid_visibility_and_later_hidden_views(self):
        cell = FakeCell()
        spec = self.knowledge.specification("Y")
        state = ObservationState(cell.scene.geometry_version, spec.version, spec.required_points(cell.regions))
        metadata = load_region_metadata()
        def regions():
            return region_snapshot(state, cell.regions, metadata)
        state.record_capture("zero")
        self.assertEqual([r["state"] for r in regions()], ["unconfirmed"]*2)
        away = replace(cell.camera, T_world_camera=camera_look_at((0,0,3), (1,0,4)))
        hidden = evaluate_visibility(cell.scene, cell.regions, away, required_region_ids=spec.required_region_ids, inspection_spec_version=spec.version)
        state.apply_visibility("zero", hidden)
        self.assertEqual([r["state"] for r in regions()], ["needs_recheck"]*2)
        visible = evaluate_visibility(cell.scene, cell.regions, cell.camera, required_region_ids=spec.required_region_ids, inspection_spec_version=spec.version)
        state.record_capture("front")
        state.apply_visibility("front", visible)
        self.assertEqual([r["state"] for r in regions()], ["confirmed", "partial"])
        before = regions()
        state.record_capture("hidden_again")
        state.apply_visibility("hidden_again", hidden)
        self.assertEqual(regions(), before)
        state.record_capture("duplicate_view")
        state.apply_visibility("duplicate_view", visible)
        self.assertEqual(regions(), before)

    def test_bad_metadata_is_rejected_before_any_motion(self):
        cell = FakeCell()
        metadata = load_region_metadata()
        metadata["R2"]["prim_path"] = "/wrong/region"
        with self.assertRaisesRegex(ValueError, "metadata"):
            self.make(cell, region_metadata=metadata)
        self.assertEqual(cell.move_count, 0)


if __name__ == "__main__":
    unittest.main()
