"""Analytical CameraParams values for numerical tests, never simulator evidence."""

import numpy as np


def renderer_parameters(camera):
    projection = np.zeros((4, 4))
    projection[0, 0] = 2*camera.focal_length_mm/camera.horizontal_aperture_mm
    projection[1, 1] = 2*camera.focal_length_mm/camera.vertical_aperture_mm
    projection[2, 3] = -1
    return {"cameraViewTransform": np.linalg.inv(camera.T_world_camera).T.ravel().tolist(),
            "cameraProjection": projection.ravel().tolist(), "cameraModel": "pinhole",
            "cameraFocalLength": camera.focal_length_mm*10,
            "cameraAperture": [camera.horizontal_aperture_mm*10, camera.vertical_aperture_mm*10],
            "cameraApertureOffset": [0, 0], "cameraNearFar": list(camera.clipping_range_m),
            "metersPerSceneUnit": 1, "renderProductResolution": list(reversed(camera.resolution_hw))}
