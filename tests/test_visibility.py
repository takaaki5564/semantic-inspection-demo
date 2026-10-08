from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from geometry_fixtures import (demo_camera, demo_scene, matrix, overhead_camera,
                               scene_of_boxes, target_region)
from part_geometry import PARTS, SceneGeometry, sample_cube_top
from visibility import evaluate_visibility, first_hit


def evaluate(scene, region, camera=None, **kwargs):
    return evaluate_visibility(scene, {"R1": region}, camera or overhead_camera(),
                               required_region_ids=["R1"], inspection_spec_version="test_v1", **kwargs)["regions"]["R1"]


class RayCastTests(unittest.TestCase):
    def setUp(self):
        self.box = ("/World/Part/Target", matrix())

    def test_ray_hits_top_at_known_world_distance(self):
        scene = scene_of_boxes([self.box])
        hit = first_hit(scene, [0, 0, 2], [0, 0, -1], max_distance_m=2)
        self.assertEqual(hit["prim_path"], self.box[0])
        self.assertEqual(hit["face_id"], "z+")
        self.assertAlmostEqual(hit["distance_m"], 1.5)
        np.testing.assert_allclose(hit["point_world_m"], [0, 0, 0.5])

    def test_nearest_surface_wins_regardless_of_box_order(self):
        obstacle = ("/World/Obstacle", matrix((0, 0, 1), (0.4, 0.4, 0.2)))
        for boxes in ([self.box, obstacle], [obstacle, self.box]):
            hit = first_hit(scene_of_boxes(boxes), [0, 0, 2], [0, 0, -1], max_distance_m=2)
            self.assertEqual(hit["prim_path"], "/World/Obstacle")
            self.assertAlmostEqual(hit["distance_m"], 0.9)

    def test_parallel_miss_range_limit_and_nonunit_ray(self):
        scene = scene_of_boxes([self.box])
        self.assertIsNone(first_hit(scene, [0, 0, 2], [1, 0, 0], max_distance_m=3))
        self.assertIsNone(first_hit(scene, [0, 0, 2], [0, 0, -1], max_distance_m=1))
        with self.assertRaises(ValueError):
            first_hit(scene, [0, 0, 2], [0, 0, -2], max_distance_m=3)

    def test_matching_prim_with_wrong_face_is_not_visible(self):
        report = evaluate(scene_of_boxes([self.box]), target_region(face="z-"))
        point = report["points"][0]
        self.assertFalse(point["visible"])
        self.assertEqual(point["reason"], "wrong_surface_hit")
        self.assertEqual(point["hit"]["prim_path"], self.box[0])
        self.assertEqual(point["hit"]["face_id"], "z+")

    def test_front_face_of_same_prim_occludes_required_back_surface(self):
        # Force a facing normal to isolate ray-cast surface identity from the angle gate.
        report = evaluate(scene_of_boxes([self.box]), target_region(points=[(0, 0, -0.5)]))
        self.assertEqual(report["points"][0]["reason"], "self_occlusion")

    def test_distinguish_self_and_external_occlusion(self):
        for path, reason in (("/World/Part/Rib", "self_occlusion"), ("/World/Obstacle", "scene_occlusion")):
            scene = scene_of_boxes([self.box, (path, matrix((0, 0, 1), (0.4, 0.4, 0.2)))])
            point = evaluate(scene, target_region())["points"][0]
            self.assertEqual(point["reason"], reason)
            self.assertEqual(point["hit"]["prim_path"], path)

    def test_missing_target_surface_never_fabricates_visibility(self):
        region = target_region(points=[(0, 0, 0.5)])
        scene = scene_of_boxes([("/World/Part/Elsewhere", matrix((4, 0, 0)))])
        self.assertEqual(evaluate(scene, region)["points"][0]["reason"], "surface_not_hit")


class ProjectionTests(unittest.TestCase):
    def test_center_projection_uses_negative_optical_z(self):
        pixels, depth = overhead_camera().project([[0, 0, 0.5], [0, 0, 3]])
        np.testing.assert_allclose(pixels[0], [320, 240])
        self.assertEqual(depth[0], 1.5)
        self.assertTrue(np.isnan(pixels[1]).all())

    def test_frustum_horizontal_vertical_near_far_and_behind(self):
        scene = scene_of_boxes([("/World/Part/Target", matrix(scale=(10, 10, 1)))])
        cases = [(target_region(points=[(2, 0, 0.5)]), overhead_camera()),
                 (target_region(points=[(0, 2, 0.5)]), overhead_camera()),
                 (target_region(), overhead_camera(clipping=(2, 100))),
                 (target_region(), overhead_camera(clipping=(0.01, 1))),
                 (target_region(), overhead_camera(position=(0, 0, 0))) ]
        for region, camera in cases:
            with self.subTest(camera=camera):
                self.assertEqual(evaluate(scene, region, camera)["points"][0]["reason"], "outside_frustum")

    def test_back_facing_and_grazing_are_rejected(self):
        scene = scene_of_boxes([("/World/Part/Target", matrix())])
        report = evaluate(scene, target_region(points=[(0, 0, -0.5)], normal=(0, 0, -1)))
        self.assertEqual(report["points"][0]["reason"], "back_facing_or_grazing")
        scene, regions = demo_scene(PARTS["A"].rib_height_m)
        report = evaluate_visibility(scene, regions, demo_camera(), required_region_ids=["R1"],
                                     inspection_spec_version="test_v1", max_incidence_angle_deg=20)
        self.assertEqual(report["regions"]["R1"]["visible_count"], 0)


class GeometryComparisonTests(unittest.TestCase):
    def test_low_and_high_ribs_change_computed_visibility_and_alternate_view_recovers(self):
        reports = []
        for height, eye in ((PARTS["A"].rib_height_m, (0.85, -1.05, 0.75)),
                            (PARTS["B"].rib_height_m, (0.85, -1.05, 0.75)),
                            (PARTS["B"].rib_height_m, (0.85, 1.05, 0.75))):
            scene, regions = demo_scene(height)
            reports.append(evaluate_visibility(scene, regions, demo_camera(eye), required_region_ids=["R1", "R2"],
                                               inspection_spec_version="test_v1"))
        a, b, alternate = [report["regions"] for report in reports]
        self.assertTrue(all(region["visible_count"] == region["total_count"] for region in a.values()))
        self.assertLess(b["R2"]["visible_count"], a["R2"]["visible_count"])
        self.assertGreater(b["R2"]["reason_counts"]["self_occlusion"], 0)
        self.assertGreater(alternate["R2"]["visible_count"], b["R2"]["visible_count"])
        self.assertEqual(alternate["R2"]["visible_count"], alternate["R2"]["total_count"])

    def test_common_rigid_part_camera_transform_preserves_visibility_and_shape_version(self):
        transform = np.array([[0, -1, 0, 3], [1, 0, 0, -2], [0, 0, 1, 0.7], [0, 0, 0, 1]], dtype=float)
        scene, regions = demo_scene(PARTS["B"].rib_height_m)
        moved, moved_regions = demo_scene(PARTS["B"].rib_height_m, transform)
        original = evaluate_visibility(scene, regions, demo_camera(), required_region_ids=["R1", "R2"], inspection_spec_version="test_v1")
        updated = evaluate_visibility(moved, moved_regions, demo_camera(world_part=transform),
                                      required_region_ids=["R1", "R2"], inspection_spec_version="test_v1")
        self.assertEqual(scene.geometry_version, moved.geometry_version)
        for key in ("R1", "R2"):
            self.assertEqual(original["regions"][key]["reason_counts"], updated["regions"][key]["reason_counts"])

    def test_scene_snapshot_replay_and_determinism(self):
        scene, regions = demo_scene(0.08)
        replay = SceneGeometry.from_dict(scene.to_dict())
        args = {"required_region_ids": ["R1", "R2"], "inspection_spec_version": "test_v1"}
        self.assertEqual(evaluate_visibility(scene, regions, demo_camera(), **args),
                         evaluate_visibility(replay, regions, demo_camera(), **args))
        altered = scene.to_dict()
        altered["triangles_world_m"][0][0][0] += 0.1
        with self.assertRaisesRegex(ValueError, "version mismatch"):
            SceneGeometry.from_dict(altered)

    def test_top_samples_and_normals_follow_rotated_nonuniform_cube(self):
        transform = np.array([[0, 0, 0.2, 1], [0, 0.4, 0, 2], [-0.6, 0, 0, 3], [0, 0, 0, 1]])
        region = sample_cube_top("R1", "/World/Part/Target", transform, grid=(2, 2))
        np.testing.assert_allclose(region.normal_part, [1, 0, 0])
        np.testing.assert_allclose(region.points_part_m[:, 0], [1.1] * 4)
        self.assertEqual(len(set(region.point_ids)), 4)

    def test_requirements_are_explicit_and_unknown_regions_are_rejected(self):
        scene, regions = demo_scene(0.025)
        report = evaluate_visibility(scene, regions, demo_camera(), required_region_ids=["R1"], inspection_spec_version="r1_only")
        self.assertEqual(list(report["regions"]), ["R1"])
        for required in ([], ["R3"], ["R1", "R1"]):
            with self.subTest(required=required), self.assertRaises(ValueError):
                evaluate_visibility(scene, regions, demo_camera(), required_region_ids=required, inspection_spec_version="test_v1")


if __name__ == "__main__":
    unittest.main()
