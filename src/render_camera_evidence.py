"""Bind RGB renderer camera parameters to the measured USD pinhole camera."""

import numpy as np

from arm_camera_evidence import assert_pose_close
from capture_metadata import rigid_transform


def validate_render_camera(parameters, camera):
    """CameraParams uses row-vector world-to-camera matrices; our poses use columns."""
    keys = ("cameraViewTransform", "cameraProjection", "cameraModel", "cameraFocalLength",
            "cameraAperture", "cameraApertureOffset", "cameraNearFar", "metersPerSceneUnit",
            "renderProductResolution")
    data = {key: parameters[key].tolist() if hasattr(parameters[key], "tolist") else parameters[key] for key in keys}
    view = np.asarray(data["cameraViewTransform"], dtype=float).reshape(4, 4)
    projection = np.asarray(data["cameraProjection"], dtype=float).reshape(4, 4)
    if not np.isfinite(view).all() or not np.isfinite(projection).all():
        raise ValueError("Nonfinite renderer camera matrices")
    world_camera = rigid_transform(np.linalg.inv(view.T))
    error = assert_pose_close(world_camera, camera.T_world_camera, "RGB renderer/USD camera mismatch")
    aperture = np.asarray(data["cameraAperture"], dtype=float)
    focal = float(data["cameraFocalLength"])
    expected = 2*camera.focal_length_mm/np.array([camera.horizontal_aperture_mm, camera.vertical_aperture_mm])
    # CameraParams focal/aperture share units differing from USD's mm; their ratio defines the frustum.
    if (data["cameraModel"] != "pinhole" or aperture.shape != (2,) or not np.isfinite(aperture).all()
            or not np.isfinite(focal) or focal <= 0 or np.any(aperture <= 0)
            or not np.allclose(2*focal/aperture, expected, rtol=1e-6, atol=1e-7)
            or not np.allclose(np.diag(projection)[:2], expected, rtol=1e-6, atol=1e-7)
            or not np.allclose(projection[2, 3], -1) or not np.allclose(projection[3, 3], 0)
            or not np.allclose(data["cameraApertureOffset"], [0, 0])
            or not np.allclose(data["cameraNearFar"], camera.clipping_range_m, rtol=1e-6, atol=1e-7)
            or data["renderProductResolution"] != list(reversed(camera.resolution_hw))
            or data["metersPerSceneUnit"] != 1):
        raise ValueError("RGB renderer/USD pinhole optics mismatch")
    return {"source": "CameraParams annotator on RGB render product", "parameters": data,
            "T_world_camera": world_camera.tolist(), "pose_error": error}
