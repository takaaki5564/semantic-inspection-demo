from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from geometry_fixtures import matrix,scene_of_boxes
from part_geometry import SceneGeometry
import test_viewpoint_planner as planner_fixtures
from viewpoint_planner import CandidateView,plan_next_view


class CandidateScenePlannerTests(unittest.TestCase):
    def fixture(self):
        fixture = planner_fixtures.PlannerTests()
        fixture.setUp()
        scene,regions,camera,state = fixture.context()
        occluder = scene_of_boxes([("/World/UR10e/link/visuals/slab",matrix((0,0,.4),(4,4,.2)))])
        blocked = SceneGeometry(np.concatenate([scene.triangles_world_m,occluder.triangles_world_m]),
                                scene.prim_paths+occluder.prim_paths,scene.face_ids+occluder.face_ids,scene.T_world_part)
        pose = fixture.candidates[1].T_part_camera
        views = (CandidateView("a_blocked",pose),CandidateView("z_clear",pose))
        return fixture,(scene,regions,camera,state),blocked,views

    def test_each_candidate_uses_its_future_arm_scene_and_state_remains_actual(self):
        fixture,context,blocked,views = self.fixture()
        before = deepcopy(context[-1].__dict__)
        result = plan_next_view(*context,fixture.knowledge,"Y",views,("camera_pose_change",),
                                candidate_scenes={"a_blocked":blocked,"z_clear":context[0]})
        self.assertEqual(result["action"]["view_id"],"z_clear")
        self.assertEqual([v["predicted_new_point_count"] for v in result["candidate_evaluations"]],[0,60])
        self.assertEqual(context[-1].__dict__,before)
        self.assertEqual(context[-1].summary()["regions"]["R2"]["observed_count"],3)

    def test_missing_prediction_or_changed_part_is_rejected(self):
        fixture,context,blocked,views = self.fixture()
        changed,_ = planner_fixtures.demo_scene(.2)
        for scenes in ({"a_blocked":blocked},{"a_blocked":blocked,"z_clear":changed}):
            with self.assertRaises(ValueError):
                plan_next_view(*context,fixture.knowledge,"Y",views,("camera_pose_change",),candidate_scenes=scenes)


if __name__ == "__main__":
    unittest.main()
