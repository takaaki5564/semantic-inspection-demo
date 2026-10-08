from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from arm_inspection_geometry import scene_fingerprint
from arm_view_prediction import body_poses,predict_robot_scene
from geometry_fixtures import matrix,overhead_camera,scene_of_boxes,target_region
from part_geometry import SceneGeometry
from visibility import evaluate_visibility


class RobotPredictionTests(unittest.TestCase):
    def fixture(self):
        reference = {"base_prim_path":"/World/UR10e/base",
                     "joints":[{"name":"joint","parent_path":"/World/UR10e/base","child_path":"/World/UR10e/link",
                                "axis":[0,0,1],"T_parent_joint":matrix((1,0,1.5)).tolist(),"T_child_joint":np.eye(4).tolist(),
                                "lower_limit_rad":-np.pi,"upper_limit_rad":np.pi}]}
        poses = body_poses(reference,np.eye(4),{"joint":0})
        target = scene_of_boxes([("/World/Part/Target",np.eye(4))])
        triangle = np.array([[[1.7,-.3,1.5],[2.3,-.3,1.5],[2,.3,1.5]]])
        scene = SceneGeometry(np.concatenate([target.triangles_world_m,triangle]),target.prim_paths+("/World/UR10e/link/visuals/mesh",),
                              target.face_ids+("mesh_face:0",),np.eye(4))
        return reference,poses,scene

    def test_future_body_geometry_changes_visibility_and_preserves_static_scene(self):
        ref,poses,scene = self.fixture()
        before = scene.triangles_world_m.copy()
        predicted = predict_robot_scene(scene,ref,np.eye(4),{"joint":np.pi},poses)
        args = {"required_region_ids":["R1"],"inspection_spec_version":"test"}
        self.assertEqual(evaluate_visibility(scene,{"R1":target_region()},overhead_camera(),**args)["regions"]["R1"]["visible_count"],1)
        future = evaluate_visibility(predicted,{"R1":target_region()},overhead_camera(),**args)
        self.assertEqual(future["regions"]["R1"]["points"][0]["reason"],"scene_occlusion")
        self.assertEqual(scene.geometry_version,predicted.geometry_version)
        self.assertEqual(scene_fingerprint(scene,exclude_robot=True),scene_fingerprint(predicted,exclude_robot=True))
        self.assertNotEqual(scene_fingerprint(scene),scene_fingerprint(predicted))
        np.testing.assert_array_equal(scene.triangles_world_m,before)

    def test_identity_prediction_reconstructs_actual_mesh_and_fixed_attachment(self):
        ref,poses,scene = self.fixture()
        predicted = predict_robot_scene(scene,ref,np.eye(4),{"joint":0},poses)
        np.testing.assert_allclose(predicted.triangles_world_m,scene.triangles_world_m,atol=1e-14)
        # A flange/bracket triangle follows the wrist body through its existing fixed offset.
        bracket = SceneGeometry(scene.triangles_world_m,scene.prim_paths[:-1]+("/World/UR10e/link/flange/CameraBracket",),scene.face_ids,np.eye(4))
        rotated = predict_robot_scene(bracket,ref,np.eye(4),{"joint":np.pi},poses)
        np.testing.assert_allclose(rotated.triangles_world_m[-1,:,0],[.3,-.3,0],atol=1e-14)

    def test_missing_body_unknown_owner_limits_and_bad_chain_are_rejected(self):
        ref,poses,scene = self.fixture()
        with self.assertRaisesRegex(ValueError,"every robot body"):
            predict_robot_scene(scene,ref,np.eye(4),{"joint":0},{})
        unknown = SceneGeometry(scene.triangles_world_m,scene.prim_paths[:-1]+("/World/UR10e/unknown/mesh",),scene.face_ids,np.eye(4))
        with self.assertRaisesRegex(ValueError,"no validated body owner"):
            predict_robot_scene(unknown,ref,np.eye(4),{"joint":0},poses)
        for values in ({},{"joint":10},{"joint":np.nan}):
            with self.assertRaises(ValueError):
                body_poses(ref,np.eye(4),values)
        invalid = deepcopy(ref)
        invalid["joints"][0]["parent_path"] = "/wrong"
        with self.assertRaises(ValueError):
            body_poses(invalid,np.eye(4),{"joint":0})


if __name__ == "__main__":
    unittest.main()
