import copy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from geometry_fixtures import demo_scene
from inspection_knowledge import InspectionKnowledge


class KnowledgeTests(unittest.TestCase):
    def setUp(self):
        self.knowledge = InspectionKnowledge.load(ROOT / "knowledge/inspection_knowledge.json")

    def test_requirements_come_from_file_and_unknown_region_or_spec_fails(self):
        _, regions = demo_scene(0.12)
        self.assertEqual(set(self.knowledge.specification("X").required_points(regions)), {"R1"})
        self.assertEqual(set(self.knowledge.specification("Y").required_points(regions)), {"R1", "R2"})
        changed = copy.deepcopy(self.knowledge.document)
        changed["specifications"]["X"]["required_region_ids"] = ["R2"]
        self.assertEqual(set(InspectionKnowledge(changed).specification("X").required_points(regions)), {"R2"})
        with self.assertRaises(ValueError):
            self.knowledge.specification("unknown")
        with self.assertRaises(ValueError):
            self.knowledge.specification("Y").required_points({"R1": regions["R1"]})

    def test_cause_method_capability_links_are_retrieved_and_traceable(self):
        enabled = self.knowledge.lookup_methods({"self_occlusion"}, {"camera_pose_change"})
        self.assertEqual(enabled, [{"method_id": "change_viewpoint", "applicable_causes": ["self_occlusion"],
                                   "required_capability_ids": ["camera_pose_change"], "missing_capability_ids": []}])
        disabled = self.knowledge.lookup_methods({"self_occlusion"}, set())
        self.assertEqual(disabled[0]["missing_capability_ids"], ["camera_pose_change"])
        self.assertEqual(self.knowledge.lookup_methods({"surface_not_hit"}, {"camera_pose_change"}), [])
        self.assertEqual(self.knowledge.provenance()["approval_status"], "prototype_not_approved")

    def test_undefined_relations_duplicate_requirements_and_versions_are_rejected(self):
        for problem in ("method", "capability", "region_duplicate", "version_duplicate", "state"):
            document = copy.deepcopy(self.knowledge.document)
            if problem == "method":
                document["missing_observation_causes"]["self_occlusion"]["eligible_method_ids"] = ["undefined"]
            elif problem == "capability":
                document["methods"]["change_viewpoint"]["required_capability_ids"] = ["undefined"]
            elif problem == "region_duplicate":
                document["specifications"]["Y"]["required_region_ids"] = ["R1", "R1"]
            elif problem == "version_duplicate":
                document["specifications"]["Y"]["version"] = "spec_X_v1"
            else:
                document["observation_states"].remove("captured")
            with self.subTest(problem=problem), self.assertRaises(ValueError):
                InspectionKnowledge(document)


if __name__ == "__main__":
    unittest.main()
