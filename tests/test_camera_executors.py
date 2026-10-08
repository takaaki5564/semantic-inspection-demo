from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from arm_camera_evidence import DEFAULT_MOUNT
from arm_kinematics import axis_rotation
from arm_motion import motion_limits
from camera_executors import ArmMountedCameraExecutor,FreeCameraExecutor,camera_to_flange
from test_arm_motion import FakeArm,NAMES


class CameraExecutorTests(unittest.TestCase):
    def test_camera_to_flange_round_trip_with_rotated_translated_frames(self):
        base = axis_rotation([1,0,0],0.5)
        base[:3,3] = [0.3,-0.5,1.1]
        world_camera = axis_rotation([0,0,1],0.8)
        world_camera[:3,3] = [1.2,0.4,0.6]
        mount = np.array(DEFAULT_MOUNT,dtype=float)
        world_flange,base_flange = camera_to_flange(world_camera,mount,base)
        np.testing.assert_allclose(world_flange@mount,world_camera,atol=1e-12)
        np.testing.assert_allclose(base@base_flange@mount,world_camera,atol=1e-12)
        self.assertGreater(np.linalg.norm(world_flange-world_camera),0.1)

    def test_arm_receives_camera_pose_and_moves_flange_with_real_state(self):
        adapter = FakeArm()
        adapter.limits = motion_limits(ROOT/"config/robots/ur10e",NAMES)
        adapter.capture = Mock(return_value=("rgb","record"))
        mount = np.array(DEFAULT_MOUNT,dtype=float)
        world_part = axis_rotation([0,0,1],-0.3)
        world_part[:3,3] = [0.4,0.6,0.3]
        world_camera = adapter.base@adapter.model_pose(adapter.goal_q)@mount
        executor = ArmMountedCameraExecutor(adapter,world_part,mount)
        result = executor.move_to_part_pose(np.linalg.inv(world_part)@world_camera)
        self.assertEqual(result["status"],"reached")
        np.testing.assert_allclose(result["T_world_flange_target"],adapter.base@adapter.model_pose(adapter.goal_q))
        np.testing.assert_allclose(result["T_world_camera_actual_from_physics"],world_camera)
        self.assertGreater(len(adapter.commands),60)
        self.assertEqual(executor.capture("image.png"),("rgb","record"))
        adapter.capture.assert_called_once_with("image.png",result)

    def test_blocked_arm_has_no_capture(self):
        adapter = FakeArm("ik_failed")
        adapter.limits = motion_limits(ROOT/"config/robots/ur10e",NAMES)
        adapter.capture = Mock()
        executor = ArmMountedCameraExecutor(adapter,np.eye(4),DEFAULT_MOUNT)
        with self.assertRaises(ValueError):
            executor.capture("before.png")
        result = executor.move_to_part_pose(adapter.base@adapter.model_pose(adapter.goal_q)@np.array(DEFAULT_MOUNT))
        self.assertEqual(result["status"],"blocked")
        with self.assertRaises(ValueError):
            executor.capture("blocked.png")
        adapter.capture.assert_not_called()
        self.assertFalse(adapter.commands)

    def test_mount_lever_arm_camera_error_can_block_reached_flange(self):
        # Flange convergence alone is insufficient for a camera on a long mounting lever.
        adapter = FakeArm()
        adapter.limits = motion_limits(ROOT/"config/robots/ur10e",NAMES)
        mount = np.array(DEFAULT_MOUNT,dtype=float)
        mount[:3,3] = [20,0,0]
        final = adapter.read_state()
        final["T_world_flange"] = (adapter.base@adapter.model_pose(adapter.goal_q+np.array([0,0,0,0.001,0,0]))).tolist()
        from unittest.mock import patch
        with patch("camera_executors.execute_goal",return_value={"status":"reached","final_state":final}):
            executor = ArmMountedCameraExecutor(adapter,np.eye(4),mount)
            result = executor.move_to_part_pose(adapter.base@adapter.model_pose(adapter.goal_q)@mount)
        self.assertEqual(result["status"],"blocked")
        self.assertEqual(result["reason"],"camera_pose_error")

    def test_free_camera_uses_same_part_pose_interface_and_actual_readback(self):
        world_part = axis_rotation([0,0,1],0.4)
        world_part[:3,3] = [1,2,3]
        pose = np.eye(4)
        pose[:3,3] = [0.1,0.2,0.3]
        adapter = Mock(CAMERA_PATH="/camera")
        adapter.world_transform.return_value = world_part@pose
        adapter.capture.return_value = ("rgb","record")
        executor = FreeCameraExecutor(adapter,world_part)
        result = executor.move_to_part_pose(pose,goal_id="free")
        self.assertEqual(result["status"],"reached")
        np.testing.assert_allclose(adapter.move_to_part_pose.call_args.args[0],pose)
        self.assertEqual(executor.capture("image.png"),("rgb","record"))
        adapter.world_transform.return_value = np.eye(4)
        with self.assertRaisesRegex(ValueError,"moved away"):
            executor.capture("moved.png")
        self.assertEqual(executor.move_to_part_pose(pose)["status"],"blocked")
        with self.assertRaises(ValueError):
            executor.capture("bad.png")


if __name__ == "__main__":
    unittest.main()
