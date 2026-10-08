import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from inspection_state import ObservationState


def report(visible_ids=("R1:000",), geometry="geometry_a", spec="spec_v1"):
    return {"part_geometry_version": geometry, "inspection_spec_version": spec,
            "evidence_source": "derived_from_simulator_geometry", "required_region_ids": ["R1"],
            "regions": {"R1": {"points": [
                {"point_id": key, "visible": key in visible_ids,
                 "reason": "visible" if key in visible_ids else "self_occlusion"}
                for key in ("R1:000", "R1:001")]}}}


class ObservationStateTests(unittest.TestCase):
    def setUp(self):
        self.required = {"R1": ("R1:000", "R1:001")}
        self.state = ObservationState("geometry_a", "spec_v1", self.required)

    def test_capture_alone_never_satisfies_observation(self):
        self.assertEqual(self.state.summary()["status"], "not_captured")
        self.state.record_capture("first")
        result = self.state.summary()
        self.assertEqual(result["status"], "captured")
        self.assertFalse(result["inspection_observation_satisfied"])
        self.assertEqual(result["regions"]["R1"]["observed_count"], 0)

    def test_accumulate_distinct_points_without_double_counting_or_defect_claim(self):
        self.state.record_capture("first")
        self.state.apply_visibility("first", report())
        self.assertEqual(self.state.summary()["status"], "missing_observation")
        self.state.record_capture("second")
        self.state.apply_visibility("second", report(visible_ids=("R1:000", "R1:001")))
        self.assertEqual(self.state.summary()["regions"]["R1"]["observed_count"], 2)
        self.assertEqual(self.state.summary()["status"], "inspection_observation_satisfied")
        self.assertEqual(self.state.summary()["defect_decision"], "not_evaluated")

    def test_geometry_or_spec_switch_requires_reset_and_clears_captures(self):
        self.state.record_capture("first")
        self.state.apply_visibility("first", report())
        for geometry, spec in (("geometry_b", "spec_v1"), ("geometry_a", "spec_v2")):
            self.state.record_capture(geometry + spec)
            before = copy.deepcopy(self.state.summary())
            with self.assertRaisesRegex(ValueError, "reset"):
                self.state.apply_visibility(geometry + spec, report(geometry=geometry, spec=spec))
            self.assertEqual(self.state.summary(), before)
        self.state.reset("geometry_b", "spec_v2", self.required)
        self.assertEqual(self.state.summary()["status"], "not_captured")
        self.assertEqual(self.state.summary()["regions"]["R1"]["missing_point_ids"], list(self.required["R1"]))
        self.assertEqual(self.state.summary()["capture_ids"], [])

    def test_no_evidence_without_capture_or_duplicate_application(self):
        with self.assertRaisesRegex(ValueError, "recorded capture"):
            self.state.apply_visibility("unknown", report())
        self.state.record_capture("first")
        with self.assertRaises(ValueError):
            self.state.record_capture("first")
        self.state.apply_visibility("first", report())
        with self.assertRaises(ValueError):
            self.state.apply_visibility("first", report())

    def test_sample_or_flag_mismatch_is_rejected_without_mutation(self):
        self.state.record_capture("first")
        for alteration in ("unknown", "duplicate", "wrong_flag", "wrong_region"):
            invalid = report()
            points = invalid["regions"]["R1"]["points"]
            if alteration == "unknown":
                points[1]["point_id"] = "R1:999"
            elif alteration == "duplicate":
                points[1]["point_id"] = points[0]["point_id"]
            elif alteration == "wrong_flag":
                points[1]["visible"] = True
            else:
                invalid["required_region_ids"] = ["R2"]
            before = self.state.summary()
            with self.subTest(alteration=alteration), self.assertRaises(ValueError):
                self.state.apply_visibility("first", invalid)
            self.assertEqual(self.state.summary(), before)

    def test_empty_specification_does_not_report_vacuous_success(self):
        for required in ({}, {"R1": []}):
            with self.assertRaises(ValueError):
                ObservationState("geometry_a", "spec_v1", required)

    def test_missing_causes_require_evaluation_and_do_not_include_observed_points(self):
        self.assertEqual(self.state.missing_evidence()["R1"]["R1:000"],
                         {"reason": "not_evaluated", "capture_id": None})
        self.state.record_capture("first")
        self.state.apply_visibility("first", report())
        self.assertEqual(self.state.missing_evidence(), {"R1": {
            "R1:001": {"reason": "self_occlusion", "capture_id": "first"}}})
        self.state.record_capture("not_evaluated_yet")
        self.assertEqual(self.state.missing_evidence()["R1"]["R1:001"]["capture_id"], "first")
        self.state.record_capture("second")
        self.state.apply_visibility("second", report(visible_ids=("R1:001",)))
        self.assertEqual(self.state.missing_evidence(), {"R1": {}})

    def test_missing_evidence_is_copied_and_reset_with_context(self):
        self.state.record_capture("first")
        self.state.apply_visibility("first", report())
        snapshot = self.state.missing_evidence()
        snapshot["R1"]["R1:001"]["reason"] = "tampered"
        self.assertEqual(self.state.missing_evidence()["R1"]["R1:001"]["reason"], "self_occlusion")
        self.state.reset("geometry_b", "spec_v2", self.required)
        self.assertTrue(all(value["reason"] == "not_evaluated" for value in self.state.missing_evidence()["R1"].values()))

    def test_invalid_evaluation_does_not_replace_missing_causes(self):
        self.state.record_capture("first")
        self.state.apply_visibility("first", report())
        self.state.record_capture("second")
        before = self.state.missing_evidence()
        with self.assertRaises(ValueError):
            self.state.apply_visibility("second", report(spec="different_spec"))
        self.assertEqual(self.state.missing_evidence(), before)


if __name__ == "__main__":
    unittest.main()
