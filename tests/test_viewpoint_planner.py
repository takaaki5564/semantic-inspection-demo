import copy
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from capture_metadata import camera_look_at
from geometry_fixtures import demo_camera, demo_scene, matrix
from inspection_knowledge import InspectionKnowledge
from inspection_state import ObservationState
from viewpoint_planner import CandidateView, load_viewpoints, motion_cost, plan_next_view
from visibility import evaluate_visibility


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.knowledge = InspectionKnowledge.load(ROOT / "knowledge/inspection_knowledge.json")
        self.candidates, self.weight, _ = load_viewpoints(ROOT / "config/camera_views.json")

    def context(self, spec_id="Y", rib_height=0.12, world_part=None, camera=None, evaluate=True):
        scene, regions = demo_scene(rib_height, world_part)
        camera = demo_camera(world_part=world_part) if camera is None else camera
        spec = self.knowledge.specification(spec_id)
        state = ObservationState(scene.geometry_version, spec.version, spec.required_points(regions))
        state.record_capture("existing_initial_frame")
        if evaluate:
            report = evaluate_visibility(scene, regions, camera, required_region_ids=spec.required_region_ids,
                                         inspection_spec_version=spec.version)
            state.apply_visibility("existing_initial_frame", report)
        return scene, regions, camera, state

    def plan(self, context, spec_id="Y", candidates=None, capabilities=("camera_pose_change",), knowledge=None):
        return plan_next_view(*context, self.knowledge if knowledge is None else knowledge, spec_id,
                              self.candidates if candidates is None else candidates, capabilities,
                              rotation_cost_m_per_rad=self.weight)

    def test_same_geometry_has_different_decisions_for_X_and_Y(self):
        x = self.plan(self.context("X"), "X")
        y = self.plan(self.context("Y"), "Y")
        self.assertEqual(x["status"], "inspection_observation_satisfied")
        self.assertIsNone(x["action"])
        self.assertEqual(x["candidate_evaluations"], [])
        self.assertEqual(y["status"], "ready")
        self.assertGreater(y["action"]["predicted_new_point_count"], 0)
        self.assertEqual(y["missing_reason_counts"], {"self_occlusion": 60})
        self.assertEqual(y["method_lookup"][0]["applicable_causes"], ["self_occlusion"])

    def test_geometry_changes_decision_without_any_part_name_input(self):
        low = self.plan(self.context(rib_height=0.025))
        high = self.plan(self.context(rib_height=0.12))
        self.assertEqual(low["status"], "inspection_observation_satisfied")
        self.assertEqual(high["status"], "ready")

    def test_predictions_do_not_mutate_or_claim_actual_observations(self):
        context = self.context()
        state = context[-1]
        before = copy.deepcopy(state.__dict__)
        result = self.plan(context)
        self.assertEqual(state.__dict__, before)
        self.assertFalse(result["observation_state"]["inspection_observation_satisfied"])
        self.assertEqual(result["prediction_source"], "predicted_from_simulator_geometry")
        self.assertEqual(result["action"]["execution_status"], "not_executed")
        self.assertEqual(result["defect_decision"], "not_evaluated")
        for candidate in result["candidate_evaluations"]:
            for region, ids in candidate["predicted_new_point_ids"].items():
                self.assertTrue(set(ids) <= set(state.summary()["regions"][region]["missing_point_ids"]))

    def test_no_capability_blocks_but_completed_spec_does_not_require_motion(self):
        disabled = self.plan(self.context(), capabilities=())
        self.assertEqual(disabled["status"], "blocked")
        self.assertEqual(disabled["reason"], "required_capability_unavailable")
        self.assertEqual(disabled["method_lookup"][0]["missing_capability_ids"], ["camera_pose_change"])
        self.assertEqual(disabled["candidate_evaluations"], [])
        self.assertEqual(self.plan(self.context("X"), "X", capabilities=())["status"], "inspection_observation_satisfied")

    def test_capture_without_evaluation_is_blocked(self):
        result = self.plan(self.context(evaluate=False))
        self.assertEqual(result["reason"], "observation_evidence_required")
        self.assertIsNone(result["action"])

    def test_empty_candidates_and_all_zero_gain_return_explicit_blocked_reasons(self):
        context = self.context()
        self.assertEqual(self.plan(context, candidates=())["reason"], "no_candidate_views")
        current = CandidateView("same", np.linalg.inv(context[0].T_world_part) @ context[2].T_world_camera)
        result = self.plan(context, candidates=(current,))
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(result["reason"], "no_candidate_improves_required_observation")
        self.assertEqual(result["candidate_evaluations"][0]["predicted_new_point_count"], 0)

    def test_unknown_cause_has_no_invented_method(self):
        camera = replace(demo_camera(), T_world_camera=matrix((10, 0, 2)))
        result = self.plan(self.context(camera=camera))
        self.assertEqual(result["reason"], "no_method_for_missing_observation_causes")
        self.assertEqual(result["missing_reason_counts"], {"outside_frustum": 126})

    def test_eligibility_uses_knowledge_links_and_not_a_hardcoded_cause_rule(self):
        document = copy.deepcopy(self.knowledge.document)
        document["missing_observation_causes"] = {}
        result = self.plan(self.context(), knowledge=InspectionKnowledge(document))
        self.assertEqual(result["reason"], "no_method_for_missing_observation_causes")
        document = copy.deepcopy(self.knowledge.document)
        document["capabilities"]["alternative_motion"] = {"description": "test capability"}
        document["methods"]["change_viewpoint"]["required_capability_ids"] = ["alternative_motion"]
        knowledge = InspectionKnowledge(document)
        self.assertEqual(self.plan(self.context(), knowledge=knowledge)["reason"], "required_capability_unavailable")
        self.assertEqual(self.plan(self.context(), knowledge=knowledge, capabilities=("alternative_motion",))["status"], "ready")

    def test_gain_then_motion_cost_then_id_ranking_is_order_independent(self):
        rear = camera_look_at((0.85, 1.05, 0.75), (0, 0, 0.08))
        far = camera_look_at((1.7, 2.1, 1.5), (0, 0, 0.08))
        current = camera_look_at((0.85, -1.05, 0.75), (0, 0, 0.08))
        candidates = (CandidateView("z_near", rear), CandidateView("a_near", rear),
                      CandidateView("far", far), CandidateView("zero_cost", current))
        context = self.context()
        result = self.plan(context, candidates=candidates)
        self.assertEqual(result["action"]["view_id"], "a_near")
        self.assertEqual(result, self.plan(context, candidates=tuple(reversed(candidates))))
        evaluations = {value["view_id"]: value for value in result["candidate_evaluations"]}
        self.assertEqual(evaluations["far"]["predicted_new_point_count"], evaluations["a_near"]["predicted_new_point_count"])
        self.assertGreater(evaluations["far"]["motion_cost_m_equivalent"], evaluations["a_near"]["motion_cost_m_equivalent"])

    def test_knowledge_method_without_implementation_is_explicitly_blocked(self):
        document = copy.deepcopy(self.knowledge.document)
        document["methods"] = {"new_method": {"required_capability_ids": ["camera_pose_change"]}}
        document["missing_observation_causes"]["self_occlusion"]["eligible_method_ids"] = ["new_method"]
        result = self.plan(self.context(), knowledge=InspectionKnowledge(document))
        self.assertEqual(result["status"], "blocked")
        self.assertEqual(result["reason"], "method_not_implemented")
        self.assertIsNone(result["action"])

    def test_rigid_part_transform_preserves_selection_gain_and_part_relative_pose(self):
        world_part = np.eye(4)
        angle = 0.7
        world_part[:3, :3] = [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
        world_part[:3, 3] = (2.5, -1.8, 0.4)
        initial = self.plan(self.context())
        moved = self.plan(self.context(world_part=world_part))
        self.assertEqual(initial["action"]["view_id"], moved["action"]["view_id"])
        self.assertEqual(initial["action"]["predicted_new_point_count"], moved["action"]["predicted_new_point_count"])
        np.testing.assert_allclose(initial["action"]["T_part_camera"], moved["action"]["T_part_camera"])
        np.testing.assert_allclose(world_part @ np.asarray(moved["action"]["T_part_camera"]), moved["action"]["T_world_camera"])

    def test_stale_geometry_spec_and_point_ids_require_reset(self):
        context = self.context()
        wrong_scene, wrong_regions = demo_scene(0.025)
        for invalid, spec in (((wrong_scene, wrong_regions, *context[2:]), "Y"), (context, "X")):
            with self.assertRaisesRegex(ValueError, "reset"):
                self.plan(invalid, spec)
        context[-1].required["R2"] = frozenset(["R2:unknown"])
        with self.assertRaisesRegex(ValueError, "reset"):
            self.plan(context)

    def test_cost_includes_orientation_and_duplicate_view_ids_are_rejected(self):
        identity = np.eye(4)
        rotated = np.diag([-1.0, -1.0, 1.0, 1.0])
        cost = motion_cost(identity, rotated, 0.1)
        self.assertEqual(cost["translation_distance_m"], 0)
        self.assertAlmostEqual(cost["rotation_distance_rad"], np.pi)
        self.assertAlmostEqual(cost["motion_cost_m_equivalent"], np.pi * 0.1)
        with self.assertRaisesRegex(ValueError, "unique"):
            self.plan(self.context(), candidates=(self.candidates[0], self.candidates[0]))

    def test_viewpoint_file_rejects_wrong_units_frame_and_duplicate_ids(self):
        document = json.loads((ROOT / "config/camera_views.json").read_text())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "views.json"
            for field, value in (("coordinate_frame", "world"), ("length_unit", "millimeter"),
                                 ("rotation_cost_m_per_rad", -1), ("views", [document["views"][0]] * 2)):
                invalid = {**document, field: value}
                path.write_text(json.dumps(invalid))
                with self.subTest(field=field), self.assertRaises(ValueError):
                    load_viewpoints(path)


if __name__ == "__main__":
    unittest.main()
