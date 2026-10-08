"""Day 1 coordinate and evidence helpers; no Isaac Sim imports."""

from fractions import Fraction
from pathlib import Path

import numpy as np


CONVENTIONS = {
    "units": "meters",
    "world": "right-handed, +Z up",
    "part": "right-handed, +Z up; origin at plate bottom center",
    "camera": "USD optical frame: +X right, +Y up, viewing along -Z",
    "matrix": "row-major JSON arrays; column vectors: p_world = T_world_camera @ p_camera",
    "composition": "T_world_camera = T_world_part @ T_part_camera",
    "quaternion_order": "wxyz; camera-to-world rotation in USD optical axes",
    "render_reference_time": "Replicator ReferenceTime numerator/denominator; renderer reference clock, not UTC",
    "readback_utc": "host UTC time after synchronized RGB readback, not exposure time",
    "app_update_count": "explicit app.update calls made by adapter; excludes internal Replicator updates",
}


def rigid_transform(value):
    """Validate a column-vector rigid transform and return a float64 array."""
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("Transform must be a finite 4x4 matrix")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("Invalid homogeneous transform row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("Transform rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1, atol=1e-6):
        raise ValueError("Transform must preserve handedness")
    return transform


def camera_look_at(eye, target):
    """Camera-to-parent matrix with USD optical axes and parent +Z as up."""
    eye, target = np.asarray(eye, dtype=float), np.asarray(target, dtype=float)
    if eye.shape != (3,) or target.shape != (3,) or not np.isfinite([eye, target]).all():
        raise ValueError("Eye and target must be finite 3-vectors")
    forward = target - eye
    distance = np.linalg.norm(forward)
    if distance < 1e-9:
        raise ValueError("Eye and target must differ")
    forward /= distance
    right = np.cross(forward, [0, 0, 1])
    if np.linalg.norm(right) < 1e-9:
        raise ValueError("Viewing direction is parallel to the chosen up axis")
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    result = np.eye(4)
    result[:3, :3] = np.column_stack((right, up, -forward))
    result[:3, 3] = eye
    return rigid_transform(result)


def reference_time(value):
    """Exact renderer reference clock value, keeping its raw rational form."""
    numerator = int(value["referenceTimeNumerator"])
    denominator = int(value["referenceTimeDenominator"])
    if denominator <= 0:
        raise ValueError("Renderer reference time denominator must be positive")
    return Fraction(numerator, denominator)


def capture_record(*, image, world_part, world_camera, quaternion_wxyz, reference,
                   timeline_seconds, app_update_count, readback_utc):
    world_part = rigid_transform(world_part)
    world_camera = rigid_transform(world_camera)
    reference_time(reference)
    quaternion = np.asarray(quaternion_wxyz, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all() or not np.isclose(np.linalg.norm(quaternion), 1):
        raise ValueError("Expected a unit quaternion in wxyz order")
    return {
        "image": image,
        "status": "captured",
        "evidence_source": "simulator_rgb",
        "pose_source": "USD world transform readback; held fixed during capture",
        "T_world_part": world_part.tolist(),
        "T_part_camera": (np.linalg.inv(world_part) @ world_camera).tolist(),
        "T_world_camera": world_camera.tolist(),
        "world_position_m": world_camera[:3, 3].tolist(),
        "world_orientation_wxyz": quaternion.tolist(),
        "render_reference_time": {key: int(reference[key]) for key in (
            "referenceTimeNumerator", "referenceTimeDenominator")},
        "timeline_seconds_at_readback": float(timeline_seconds),
        "app_update_count": int(app_update_count),
        "readback_utc": readback_utc,
    }


def validate_capture_pair(records, images):
    """Check evidence consistency; this does not evaluate inspection coverage."""
    if len(records) != 2 or len(images) != 2:
        raise ValueError("Expected exactly two captures")
    for record, image in zip(records, images):
        world_part = rigid_transform(record["T_world_part"])
        part_camera = rigid_transform(record["T_part_camera"])
        world_camera = rigid_transform(record["T_world_camera"])
        if not np.allclose(world_part @ part_camera, world_camera, atol=1e-7):
            raise ValueError("Part/world/camera transforms do not compose")
        if record["status"] != "captured" or record["evidence_source"] != "simulator_rgb":
            raise ValueError("Unexpected capture evidence status/source")
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8 or not image.size:
            raise ValueError("Expected a nonempty uint8 RGB image")
        if np.ptp(image) == 0:
            raise ValueError("Uniform image: scene may not have rendered")
    if records[0]["image"] == records[1]["image"]:
        raise ValueError("Captures must have different image paths")
    if np.allclose(records[0]["T_world_camera"], records[1]["T_world_camera"]):
        raise ValueError("Camera poses are identical")
    if reference_time(records[1]["render_reference_time"]) <= reference_time(records[0]["render_reference_time"]):
        raise ValueError("Second render reference time must advance")
    if images[0].shape != images[1].shape or np.array_equal(images[0], images[1]):
        raise ValueError("Expected same-size, distinct images")
    difference = np.abs(images[0].astype(float) - images[1].astype(float))
    return {"mean_absolute_pixel_difference": float(difference.mean()),
            "changed_pixel_fraction": float(np.any(difference > 0, axis=2).mean())}


def prepare_output_directory(path):
    """Reserve a new run directory; never mix or overwrite existing evidence."""
    path = Path(path).resolve()
    path.mkdir(parents=True, exist_ok=False)
    (path / "images").mkdir()
    return path
