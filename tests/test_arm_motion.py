import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from arm_kinematics import axis_rotation
from arm_motion import execute_goal, motion_limits, trajectory, trajectory_sample
from arm_motion_check import read_step2


NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class FakeArm:
    dt = 0.05

    def __init__(self, mode="follow"):
        self.mode = mode
        self.time = 0.0
        self.q = np.zeros(6)
        self.velocity = np.zeros(6)
        self.commands = []
        self.goal_q = np.array([0.3, -0.2, 0.1, 0.2, 0, 0])
        self.base = axis_rotation([0, 0, 1], 0.4)
        self.base[:3, 3] = [1, 2, 0.4]

    def model_pose(self, q):
        pose = axis_rotation([0, 0, 1], q[3])
        pose[:3, 3] = q[:3]
        return pose

    def read_state(self):
        flange = self.base @ self.model_pose(self.q)
        if self.mode == "wrong_flange":
            flange[0, 3] += 0.01
        return {"simulation_time_s": self.time, "joints_rad": self.q.tolist(), "velocities_rad_s": self.velocity.tolist(),
                "T_world_base": self.base.tolist(), "T_world_flange": flange.tolist()}

    def solve_ik(self, target, start):
        return {"success": self.mode != "ik_failed", "joints_rad": self.goal_q.tolist()}

    def command(self, q):
        self.commands.append(q.copy())

    def step(self):
        if self.mode != "stale_clock":
            self.time += self.dt
        previous = self.q.copy()
        if self.mode != "stuck":
            self.q = self.commands[-1].copy()
        self.velocity = (self.q-previous) / self.dt
        if self.mode == "moving":
            self.velocity[0] = 0.1
        if self.mode == "late_settle" and self.time < 3.5:
            self.velocity[0] = 0.1
        if self.mode == "limit_violation":
            self.q[2] = math.pi+0.1
        if self.mode == "base_moved":
            self.base[0, 3] += 0.001

    def is_running(self):
        return self.mode != "closed"


class ArmMotionTests(unittest.TestCase):
    def setUp(self):
        self.limits = motion_limits(ROOT / "config/robots/ur10e", NAMES)

    def run_goal(self, mode="follow", mutate=None, **kwargs):
        arm = FakeArm(mode)
        if mutate:
            mutate(arm)
        goal = {"goal_id": "test", "T_base_flange_target": arm.model_pose([0.3, -0.2, 0.1, 0.2, 0, 0]).tolist()}
        events = []
        result = execute_goal(arm, goal, self.limits, events.append, max_motion_seconds=4, **kwargs)
        return arm, result, events

    def test_profile_obeys_each_command_limit_and_has_smooth_endpoints(self):
        start, goal = np.zeros(6), np.array([5, -3, 2, 1, 0, -4])
        profile = trajectory(start, goal, self.limits)
        duration = profile["duration_seconds"]
        for key, limit in (("maximum_command_velocity_rad_s", "velocity"), ("maximum_command_acceleration_rad_s2", "acceleration"), ("maximum_command_jerk_rad_s3", "jerk")):
            self.assertTrue(np.all(np.array(profile[key]) <= self.limits[limit]+1e-12))
        samples = np.array([trajectory_sample(profile, t) for t in np.linspace(0, duration, 2001)])
        np.testing.assert_array_equal(samples[0], start)
        np.testing.assert_array_equal(samples[-1], goal)
        self.assertTrue(np.all(samples >= np.minimum(start, goal)-1e-12))
        self.assertTrue(np.all(samples <= np.maximum(start, goal)+1e-12))
        velocities = np.diff(samples, axis=0)/(duration/2000)
        accelerations = np.diff(velocities, axis=0)/(duration/2000)
        self.assertTrue(np.all(np.max(np.abs(velocities), axis=0) <= self.limits["velocity"]+1e-6))
        self.assertTrue(np.all(np.max(np.abs(accelerations), axis=0) <= self.limits["acceleration"]+1e-6))
        np.testing.assert_allclose(velocities[[0,-1]], 0, atol=1e-5)

    def test_invalid_profile_inputs_rejected(self):
        for q in ([math.nan]*6, [0]*5, [0,0,4,0,0,0]):
            with self.assertRaises(ValueError):
                trajectory(np.zeros(6), q, self.limits)

    def test_measured_pose_and_joints_settle_in_transformed_base(self):
        arm, result, events = self.run_goal()
        self.assertEqual(result["status"], "reached")
        self.assertGreaterEqual(result["settled_seconds"], 0.25)
        self.assertGreaterEqual(result["max_actual_joint_displacement_rad"], 0.29)
        self.assertEqual(len(arm.commands), result["physics_steps"])
        self.assertEqual(events[-1]["event"], "goal_finished")
        self.assertLess(result["final_error"]["position_error_m"], 1e-10)
        self.assertGreater(np.linalg.norm(arm.commands[0]-arm.goal_q), 0.2)

    def test_ik_failure_never_commands_motion(self):
        arm, result, _ = self.run_goal("ik_failed")
        self.assertEqual(result["reason"], "ik_failed")
        self.assertFalse(arm.commands)

    def test_ik_limit_or_bad_fk_never_commands_motion(self):
        for q, reason in (([0,0,4,0,0,0], "ik_joint_limit_violation"), ([0.4,0,0,0,0,0], "ik_pose_residual")):
            arm, result, _ = self.run_goal(mutate=lambda a: setattr(a, "goal_q", np.array(q)))
            self.assertEqual(result["reason"], reason)
            self.assertFalse(arm.commands)

    def test_stuck_robot_wrong_actual_flange_and_moving_robot_time_out(self):
        for mode in ("stuck", "wrong_flange", "moving"):
            arm, result, _ = self.run_goal(mode)
            self.assertEqual(result["status"], "blocked", mode)
            self.assertEqual(result["reason"], "motion_timeout", mode)
            self.assertLessEqual(len(arm.commands), 81)

    def test_settling_requires_continuous_low_velocity(self):
        _, result, _ = self.run_goal("late_settle")
        self.assertEqual(result["status"], "reached")
        self.assertGreaterEqual(result["final_state"]["simulation_time_s"], 3.75)

    def test_clock_limit_base_and_window_faults_stop(self):
        for mode, reason in (("stale_clock", "physics_clock_did_not_advance"), ("limit_violation", "actual_joint_limit_violation"),
                             ("base_moved", "fixed_base_moved"), ("closed", "simulator_closed")):
            _, result, _ = self.run_goal(mode)
            self.assertEqual(result["reason"], reason)
            self.assertLessEqual(result["physics_steps"], 1)

    def test_wall_clock_watchdog_is_independent_of_simulation_clock(self):
        values = iter([0,61])
        arm, result, _ = self.run_goal(clock=lambda: next(values))
        self.assertEqual(result["reason"], "wall_clock_timeout")
        self.assertFalse(arm.commands)

    def test_step2_failure_rejected(self):
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"report.json"
            path.write_text(json.dumps({"demo_step":"arm_step2_model_check", "run_status":"failed"}))
            with self.assertRaisesRegex(ValueError, "Step 2 must pass"):
                read_step2(path, "6.1.0.0")


if __name__ == "__main__":
    unittest.main()
