from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from arm_inspection import observe_capture, require_static_scene, screen_candidates, solve_posture_ik
from arm_inspection_geometry import scene_fingerprint
from inspection_knowledge import InspectionKnowledge, InspectionSpec
from inspection_state import ObservationState
from viewpoint_planner import CandidateView
from loop_fixtures import FakeAdapter
from geometry_fixtures import overhead_camera, scene_of_boxes, target_region
from part_geometry import SceneGeometry


class CandidateScreeningTests(unittest.TestCase):
    def fixture(self):
        base, part, mount = (np.eye(4) for _ in range(3))
        base[:3,3], part[:3,3], mount[:3,3] = [1,2,3], [4,5,6], [.1,.2,.3]
        state = {"joints_rad": [0]*6, "T_world_base": base.tolist(), "physics_step_index": 0}
        adapter = Mock()
        adapter.read_state.return_value = state
        adapter.limits = {"lower": np.full(6,-1), "upper": np.ones(6)}
        views = [CandidateView("near", np.eye(4)), CandidateView("far", np.diag([1,1,1,1]))]
        target = np.linalg.inv(base)@part@np.linalg.inv(mount)
        adapter.solve_ik.side_effect = [{"success": True, "joints_rad": [.2]*6}, {"success": False}]
        adapter.model_pose.return_value = target
        return adapter, part, mount, views, target

    def test_frames_and_filtering_without_motion_commands(self):
        adapter, part, mount, views, target = self.fixture()
        feasible, records = screen_candidates(adapter, part, mount, views)
        self.assertEqual([v.view_id for v in feasible], ["near"])
        self.assertEqual([r["status"] for r in records], ["feasible", "blocked"])
        np.testing.assert_allclose(records[0]["T_base_flange_target"], target)
        self.assertFalse(records[0]["motion_executed"])
        self.assertFalse(records[0]["collision_safety_validated"])
        adapter.command.assert_not_called()
        adapter.step.assert_not_called()

    def test_solver_success_with_bad_limits_or_residual_is_excluded(self):
        for fault in ("limits", "residual"):
            adapter, part, mount, views, target = self.fixture()
            if fault == "limits":
                adapter.solve_ik.side_effect = [{"success": True, "joints_rad": [2]*6}, {"success": False}]
            else:
                target[0,3] += .1
                adapter.model_pose.return_value = target
            feasible, records = screen_candidates(adapter, part, mount, views)
            self.assertFalse(feasible)
            self.assertEqual(records[0]["reason"], "ik_joint_limit_violation" if fault == "limits" else "ik_pose_residual")

    def test_screening_detects_advanced_physics(self):
        adapter, part, mount, views, _ = self.fixture()
        initial = adapter.read_state.return_value
        adapter.read_state.side_effect = [initial, {**initial, "physics_step_index": 1}]
        with self.assertRaisesRegex(RuntimeError, "must not advance physics"):
            screen_candidates(adapter, part, mount, views)


class PostureIkTests(unittest.TestCase):
    def fixture(self):
        limits = {"lower": np.full(6, -2*np.pi), "upper": np.full(6, 2*np.pi)}
        limits["lower"][2], limits["upper"][2] = -np.pi, np.pi
        target, start = np.eye(4), np.zeros(6)
        preferred = np.array([0,-1,1.5,-1,-1,0])
        return limits, target, start, preferred

    def test_reference_branch_can_be_found_after_current_seed_failure(self):
        limits, target, start, preferred = self.fixture()
        positive, negative = preferred.copy(), preferred.copy()
        negative[2] = -1.5
        solve = Mock(side_effect=[{"success": False}, {"success": True, "joints_rad": negative.tolist()},
                                  {"success": True, "joints_rad": positive.tolist()}])
        result = solve_posture_ik(solve, lambda q: target, target, start, limits, [negative, positive], preferred)
        self.assertTrue(result["success"])
        np.testing.assert_allclose(result["joints_rad"], positive)
        self.assertEqual(len(result["attempts"]), 3)
        np.testing.assert_array_equal(start, np.zeros(6))
        self.assertFalse(result["collision_aware"])

    def test_equivalent_revolute_solution_is_unwrapped_toward_actual_joints(self):
        limits, target, start, preferred = self.fixture()
        start[0] = 3.0
        q = preferred.copy()
        q[0] = -3.0
        solve = Mock(return_value={"success": True, "joints_rad": q.tolist()})
        result = solve_posture_ik(solve, lambda q: target, target, start, limits, [], preferred)
        self.assertAlmostEqual(result["joints_rad"][0], 2*np.pi-3)

    def test_bad_solutions_do_not_become_feasible_and_bad_seed_fails(self):
        limits, target, start, preferred = self.fixture()
        solve = Mock(return_value={"success": True, "joints_rad": preferred.tolist()})
        wrong_pose = np.eye(4)
        wrong_pose[0,3] = .02
        result = solve_posture_ik(solve, lambda q: wrong_pose, target, start, limits, [], preferred)
        self.assertFalse(result["success"])
        with self.assertRaisesRegex(ValueError, "exceeds model limits"):
            solve_posture_ik(solve, lambda q: target, target, start, limits, [[100]*6], preferred)


class CapturedObservationTests(unittest.TestCase):
    def fixture(self):
        adapter = FakeAdapter()
        scene, regions, camera = adapter.scene, adapter.regions, adapter.camera
        meta = {"full_scene_sha256": scene_fingerprint(scene), "static_scene_sha256": scene_fingerprint(scene, exclude_robot=True),
                "physics_step_index": 240, "physics_time_seconds": 1.0}
        snapshot = (scene, regions, camera, meta)
        spec = InspectionKnowledge.load(ROOT/"knowledge/inspection_knowledge.json").specification("Y")
        state = ObservationState(scene.geometry_version, spec.version, spec.required_points(regions))
        rgb, capture = adapter.capture("images/front.png")
        capture.update(physics_step_index=240, physics_time_seconds=1.0)
        return snapshot, rgb, capture, state, spec

    def test_valid_capture_updates_actual_visibility_and_logs_scene_binding(self):
        snap, rgb, capture, state, spec = self.fixture()
        report, clock = observe_capture(snap, snap, rgb, capture, state, spec, "front")
        self.assertEqual(state.summary()["regions"]["R1"]["observed_count"], 63)
        self.assertLess(state.summary()["regions"]["R2"]["observed_count"], 63)
        self.assertEqual(report["geometry_snapshot"]["physics_step_index"], 240)
        self.assertEqual(report["robot_occluded_point_ids"], {"R1": [], "R2": []})
        self.assertGreater(clock, 0)
        self.assertEqual(state.summary()["defect_decision"], "not_evaluated")

    def test_stale_step_moved_geometry_wrong_pose_or_stale_image_never_updates_state(self):
        for fault in ("step", "hash", "pose", "image"):
            before, rgb, capture, state, spec = self.fixture()
            after = (*before[:3], deepcopy(before[3]))
            kwargs = {}
            if fault == "step":
                after[3]["physics_step_index"] += 1
            elif fault == "hash":
                after[3]["full_scene_sha256"] = "different"
            elif fault == "pose":
                capture["T_world_camera"][0][3] += .1
            else:
                kwargs["previous_rgb"] = rgb.copy()
            with self.subTest(fault=fault), self.assertRaises((ValueError, RuntimeError)):
                observe_capture(before, after, rgb, capture, state, spec, "bad", **kwargs)
            self.assertEqual(state.capture_ids, [])
            self.assertEqual(state.evaluated_capture_ids, [])

    def test_static_scene_change_requires_session_reset(self):
        snap, _, _, _, _ = self.fixture()
        require_static_scene(snap, snap)
        changed = (*snap[:3], {**snap[3], "static_scene_sha256": "changed"})
        with self.assertRaisesRegex(ValueError, "reset"):
            require_static_scene(snap, changed)
        # A different camera/dynamic hash does not invalidate the static scene.
        moved = (snap[0], snap[1], replace(snap[2], T_world_camera=np.eye(4)), {**snap[3], "full_scene_sha256": "arm_moved"})
        require_static_scene(snap, moved)

    def test_robot_first_hit_is_recorded_as_arm_occlusion_and_remains_missing(self):
        _, rgb, capture, _, _ = self.fixture()
        target = scene_of_boxes([("/World/Part/Target", np.eye(4))])
        triangle = np.array([[[-1,-1,1.5],[1,-1,1.5],[0,1,1.5]]])
        scene = SceneGeometry(np.concatenate([target.triangles_world_m, triangle]),
                              target.prim_paths+("/World/UR10e/forearm/visuals/mesh",),
                              target.face_ids+("mesh_face:0",), np.eye(4))
        camera, regions = overhead_camera(), {"R1": target_region()}
        meta = {"full_scene_sha256": scene_fingerprint(scene), "static_scene_sha256": scene_fingerprint(scene, exclude_robot=True),
                "physics_step_index": 240, "physics_time_seconds": 1.0}
        snapshot = (scene, regions, camera, meta)
        spec = InspectionSpec("test", "test", ("R1",))
        state = ObservationState(scene.geometry_version, spec.version, spec.required_points(regions))
        capture.update(T_world_part=np.eye(4).tolist(), T_world_camera=camera.T_world_camera.tolist(),
                       T_part_camera=camera.T_world_camera.tolist())
        report, _ = observe_capture(snapshot, snapshot, rgb, capture, state, spec, "occluded")
        self.assertEqual(report["robot_occluded_point_ids"], {"R1": ["R1:000"]})
        self.assertEqual(state.summary()["regions"]["R1"]["observed_count"], 0)


if __name__ == "__main__":
    unittest.main()
