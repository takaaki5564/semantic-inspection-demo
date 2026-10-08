"""Compare against closed-form planar-arm FK, not another copy of the chain code."""

from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from arm_kinematics import axis_rotation, compare_model, pose_error, usd_joint_fk


def translated(x):
    result = np.eye(4)
    result[0, 3] = x
    return result.tolist()


def planar_reference():
    return {"joints": [{"name": name, "axis": [0, 0, 1], "T_parent_joint": translated(offset),
                        "T_child_joint": np.eye(4).tolist(), "lower_limit_rad": -np.pi, "upper_limit_rad": np.pi}
                       for name, offset in (("first", 0), ("second", 1))],
            "T_leaf_flange": translated(1), "T_base_flange_authored": translated(2)}


def analytic_planar(q):
    first, second = q
    angle = first + second
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0, np.cos(first) + c], [s, c, 0, np.sin(first) + s],
                     [0, 0, 1, 0], [0, 0, 0, 1]])


def compare(reference=None, model_pose=analytic_planar, **kwargs):
    return compare_model(reference or planar_reference(), model_joint_names=["first", "second"],
                         model_limits_rad=dict.fromkeys(["first", "second"], [-np.pi, np.pi]),
                         model_pose=model_pose, **kwargs)


class KinematicsTests(unittest.TestCase):
    def test_chain_matches_known_planar_quarter_turns(self):
        actual = usd_joint_fk(planar_reference(), {"first": np.pi / 2, "second": np.pi / 2})
        np.testing.assert_allclose(actual[:3, 3], [-1, 1, 0], atol=1e-12)
        np.testing.assert_allclose(actual[:3, :3], np.diag([-1, -1, 1]), atol=1e-12)
        self.assertTrue(all(compare()["checks"].values()))

    def test_name_mapping_handles_reversed_model_order(self):
        result = compare_model(planar_reference(), model_joint_names=["second", "first"],
                               model_limits_rad=dict.fromkeys(["first", "second"], [-np.pi, np.pi]),
                               model_pose=lambda q: analytic_planar([q[1], q[0]]))
        self.assertTrue(all(result["checks"].values()))

    def test_translation_error_and_axis_sign_error_cannot_pass(self):
        def shifted(q):
            pose = analytic_planar(q)
            pose[0, 3] += 0.01
            return pose
        self.assertFalse(compare(model_pose=shifted)["checks"]["all_fk_cases_within_tolerance"])
        flipped = compare(model_pose=lambda q: analytic_planar([q[0], -q[1]]))
        self.assertTrue(flipped["cases"][0]["passed"])
        self.assertFalse(flipped["checks"]["all_fk_cases_within_tolerance"])

    def test_authored_pose_and_joint_limits_are_independent_acceptance_gates(self):
        reference = deepcopy(planar_reference())
        reference["T_base_flange_authored"][0][3] += 0.02
        result = compare(reference)
        self.assertTrue(result["checks"]["all_fk_cases_within_tolerance"])
        self.assertFalse(result["checks"]["authored_zero_pose_matches_joint_frames"])
        reference = planar_reference()
        reference["joints"][0]["lower_limit_rad"] = -2.0
        self.assertFalse(compare(reference)["checks"]["joint_limits_match"])

    def test_rotation_error_handles_zero_and_half_turn_and_invalid_inputs_fail(self):
        self.assertEqual(pose_error(np.eye(4), np.eye(4))["orientation_error_rad"], 0)
        self.assertAlmostEqual(pose_error(axis_rotation([1, 0, 0], np.pi), np.eye(4))["orientation_error_rad"], np.pi)
        for positions in ({"first": 0}, {"first": float("nan"), "second": 0}, {"first": 4, "second": 0}):
            with self.assertRaises(ValueError):
                usd_joint_fk(planar_reference(), positions)
        with self.assertRaises(ValueError):
            compare(position_tolerance_m=float("nan"))
        empty = {**planar_reference(), "joints": []}
        with self.assertRaises(ValueError):
            usd_joint_fk(empty, {})


if __name__ == "__main__":
    unittest.main()
