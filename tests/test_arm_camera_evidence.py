from copy import deepcopy
from fractions import Fraction
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from arm_camera_evidence import DEFAULT_MOUNT,mounted_capture_record
from arm_kinematics import axis_rotation


class MountedEvidenceTests(unittest.TestCase):
    def arguments(self):
        flange = axis_rotation([0,1,0],0.3)
        flange[:3,3] = [0.7,0.2,0.6]
        mount = np.array(DEFAULT_MOUNT,dtype=float)
        camera = flange@mount
        state = {"simulation_time_s":3.25,"physics_step_index":780,"T_world_flange":flange.tolist(),
                 "joints_rad":[0]*6,"velocities_rad_s":[0]*6}
        movement = {"goal_id":"view_01","status":"reached","T_flange_camera":mount.tolist(),
                    "T_world_part":np.eye(4).tolist(),"T_world_camera_target":camera.tolist(),"T_world_flange_target":flange.tolist()}
        return {"image":"images/view_01.png","movement":movement,"state_before":deepcopy(state),"state_after":deepcopy(state),
                "camera_before":camera.copy(),"camera_after":camera.copy(),"local_mount":mount.copy(),
                "reference":{"referenceTimeNumerator":3250000000,"referenceTimeDenominator":1000000000},
                "previous_reference":Fraction(1),"readback_utc":"2026-10-08T00:00:00+00:00","render_step_index":1}

    def test_physical_pose_mount_and_renderer_reference_form_consistent_evidence(self):
        record = mounted_capture_record(**self.arguments())
        self.assertEqual(record["status"],"captured")
        self.assertFalse(record["inspection_evaluated"])
        self.assertTrue(record["physics_held_during_capture"])
        np.testing.assert_allclose(np.array(record["T_world_flange"])@record["T_flange_camera"],record["T_world_camera"])

    def test_physics_time_or_step_advance_rejected(self):
        for field in ("simulation_time_s","physics_step_index"):
            args = self.arguments()
            args["state_after"][field] += 1
            with self.assertRaisesRegex(ValueError,"Physics advanced"):
                mounted_capture_record(**args)

    def test_moving_flange_camera_or_mount_rejected(self):
        for key,message in (("flange","Flange moved"),("camera","Camera moved"),("mount","mount changed")):
            args = self.arguments()
            if key == "flange":
                args["state_after"]["T_world_flange"][0][3] += 0.01
            elif key == "camera":
                args["camera_after"][0,3] += 0.01
            else:
                args["local_mount"][0,3] += 0.01
            with self.assertRaisesRegex(ValueError,message):
                mounted_capture_record(**args)

    def test_camera_moved_independently_of_flange_rejected_even_when_still(self):
        args = self.arguments()
        args["camera_before"][0,3] += 0.01
        args["camera_after"][0,3] += 0.01
        with self.assertRaisesRegex(ValueError,"disagrees with physical flange"):
            mounted_capture_record(**args)

    def test_stale_and_wrong_frame_times_rejected(self):
        args = self.arguments()
        args["previous_reference"] = Fraction(13,4)
        with self.assertRaisesRegex(ValueError,"Stale"):
            mounted_capture_record(**args)
        args = self.arguments()
        args["reference"]["referenceTimeNumerator"] += 100000000
        with self.assertRaisesRegex(ValueError,"does not match"):
            mounted_capture_record(**args)

    def test_unreached_goal_and_readback_goal_error_rejected(self):
        args = self.arguments()
        args["movement"]["status"] = "blocked"
        with self.assertRaisesRegex(ValueError,"unreached"):
            mounted_capture_record(**args)
        args = self.arguments()
        args["movement"]["T_world_camera_target"][0][3] += 0.02
        with self.assertRaisesRegex(ValueError,"not reached"):
            mounted_capture_record(**args)

    def test_frozen_snapshot_of_unsettled_arm_is_rejected(self):
        args = self.arguments()
        args["state_after"]["velocities_rad_s"][0] = 0.1
        with self.assertRaisesRegex(ValueError,"no longer settled"):
            mounted_capture_record(**args)


if __name__ == "__main__":
    unittest.main()
