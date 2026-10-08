import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from capture_metadata import (camera_look_at, capture_record, prepare_output_directory,
                              reference_time, rigid_transform, validate_capture_pair)
from verify_day1 import verify


def reference(numerator, denominator=60):
    return {"referenceTimeNumerator": numerator, "referenceTimeDenominator": denominator}


def evidence_pair():
    records = []
    for index, y in enumerate((-1.0, 1.0), start=1):
        pose = np.eye(4)
        pose[:3, 3] = [1, y, 1]
        records.append(capture_record(
            image=f"images/view_{index:02d}.png", world_part=np.eye(4), world_camera=pose,
            quaternion_wxyz=[1, 0, 0, 0], reference=reference(index),
            timeline_seconds=index / 60, app_update_count=index,
            readback_utc="2026-10-08T03:00:00+00:00"))
    first = np.zeros((4, 5, 3), dtype=np.uint8)
    first[0, 0] = [40, 60, 80]
    second = np.zeros_like(first)
    second[-1, -1] = [20, 80, 100]
    return records, [first, second]


class CoordinateTests(unittest.TestCase):
    def test_target_is_in_front_in_negative_optical_z(self):
        eye = [0.85, -1.05, 0.75]
        target = [0, 0, 0.08]
        pose = camera_look_at(eye, target)
        camera_target = np.linalg.inv(pose) @ [*target, 1]
        np.testing.assert_allclose(camera_target[:2], [0, 0], atol=1e-12)
        self.assertLess(camera_target[2], 0)
        self.assertGreater(pose[2, 1], 0)  # local up points toward world up

    def test_part_relative_pose_survives_translated_rotated_part(self):
        part = np.array([[0, -1, 0, 3], [1, 0, 0, -2], [0, 0, 1, 0.25], [0, 0, 0, 1]], dtype=float)
        relative = camera_look_at([1, -1, 1], [0, 0, 0])
        world_camera = part @ relative
        # An independent point transform checks multiplication order and frame direction.
        np.testing.assert_allclose(world_camera[:3, 3], [4, -1, 1.25])
        recovered = np.linalg.inv(part) @ world_camera
        np.testing.assert_allclose(recovered, relative, atol=1e-12)

    def test_reject_degenerate_look_at(self):
        for eye, target in (([0, 0, 0], [0, 0, 0]), ([0, 0, 1], [0, 0, 0]),
                            ([float("nan"), 1, 1], [0, 0, 0])):
            with self.subTest(eye=eye), self.assertRaises(ValueError):
                camera_look_at(eye, target)

    def test_reject_scale_reflection_and_bad_homogeneous_row(self):
        for matrix in (np.diag([2, 1, 1, 1]), np.diag([-1, 1, 1, 1]), np.diag([1, 1, 1, 2])):
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                rigid_transform(matrix)


class EvidenceTests(unittest.TestCase):
    def test_reference_time_is_exact_and_rejects_zero_denominator(self):
        self.assertEqual(reference_time(reference(1, 3)), reference_time(reference(2, 6)))
        with self.assertRaises(ValueError):
            reference_time(reference(1, 0))

    def test_accept_valid_pair(self):
        records, images = evidence_pair()
        result = validate_capture_pair(records, images)
        self.assertGreater(result["mean_absolute_pixel_difference"], 0)
        self.assertEqual(result["changed_pixel_fraction"], 0.1)

    def test_reject_stale_or_backwards_render_time(self):
        records, images = evidence_pair()
        for stamp in (reference(2, 120), reference(0)):
            records[1]["render_reference_time"] = stamp
            with self.subTest(stamp=stamp), self.assertRaisesRegex(ValueError, "advance"):
                validate_capture_pair(records, images)

    def test_reject_identical_images_and_uniform_frame(self):
        records, images = evidence_pair()
        with self.assertRaisesRegex(ValueError, "distinct"):
            validate_capture_pair(records, [images[0], images[0].copy()])
        with self.assertRaisesRegex(ValueError, "Uniform"):
            validate_capture_pair(records, [np.zeros_like(images[0]), images[1]])

    def test_reject_identical_camera_pose(self):
        records, images = evidence_pair()
        for key in ("T_world_camera", "T_part_camera"):
            records[1][key] = copy.deepcopy(records[0][key])
        with self.assertRaisesRegex(ValueError, "poses are identical"):
            validate_capture_pair(records, images)

    def test_reject_inconsistent_composition(self):
        records, images = evidence_pair()
        records[1]["T_part_camera"][0][3] += 1
        with self.assertRaisesRegex(ValueError, "compose"):
            validate_capture_pair(records, images)

    def test_reject_inspection_success_claim(self):
        records, images = evidence_pair()
        records[1]["status"] = "inspected"
        with self.assertRaisesRegex(ValueError, "status/source"):
            validate_capture_pair(records, images)

    def test_output_directory_never_overwrites_existing_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            output = prepare_output_directory(Path(root) / "outputs")
            marker = output / "keep.txt"
            marker.write_text("existing evidence")
            with self.assertRaises(FileExistsError):
                prepare_output_directory(output)
            self.assertEqual(marker.read_text(), "existing evidence")

    def test_verify_saved_pngs_and_detect_tampered_comparison(self):
        records, images = evidence_pair()
        with tempfile.TemporaryDirectory() as root:
            output = prepare_output_directory(Path(root) / "outputs")
            for record, image in zip(records, images):
                Image.fromarray(image).save(output / record["image"])
            metadata = {"captures": records, "scene": {"inspection_evaluated": False},
                        "camera": {"resolution_hw": [4, 5]},
                        "checks": validate_capture_pair(records, images)}
            path = output / "camera_poses.json"
            path.write_text(json.dumps(metadata))
            self.assertEqual(verify(output), metadata["checks"])
            metadata["checks"]["mean_absolute_pixel_difference"] = 0
            path.write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError, "inconsistent"):
                verify(output)


if __name__ == "__main__":
    unittest.main()
