"""Small, explicit geometric fixtures with analytically known faces and occluders."""

import numpy as np

from capture_metadata import camera_look_at
from part_geometry import SceneGeometry, SurfaceRegion, cube_triangles, sample_cube_top
from visibility import CameraModel


def matrix(position=(0, 0, 0), scale=(1, 1, 1)):
    value = np.diag([*scale, 1.0])
    value[:3, 3] = position
    return value


def scene_of_boxes(boxes, world_part=None):
    world_part = np.eye(4) if world_part is None else world_part
    triangles, paths, face_ids = [], [], []
    for path, local_transform in boxes:
        world = world_part @ local_transform if path.startswith("/World/Part/") else local_transform
        faces, ids = cube_triangles(world)
        triangles.extend(faces)
        paths.extend([path] * len(faces))
        face_ids.extend(ids)
    return SceneGeometry(triangles, tuple(paths), tuple(face_ids), world_part)


def target_region(points=((0, 0, 0.5),), normal=(0, 0, 1), face="z+"):
    return SurfaceRegion("R1", "/World/Part/Target", face, np.asarray(points), np.asarray(normal))


def overhead_camera(position=(0, 0, 2), clipping=(0.01, 100)):
    return CameraModel(matrix(position), 24, 36, 27, clipping)


def demo_scene(rib_height, world_part=None):
    transforms = {
        "/World/Part/Plate": matrix((0, 0, 0.03), (0.9, 0.6, 0.06)),
        "/World/Part/Rib": matrix((0, 0, 0.06 + rib_height / 2), (0.75, 0.045, rib_height)),
        "/World/Part/R1Marker": matrix((-0.08, -0.17, 0.061), (0.5, 0.15, 0.002)),
        "/World/Part/R2Marker": matrix((0.08, 0.17, 0.061), (0.5, 0.15, 0.002)),
    }
    scene = scene_of_boxes(list(transforms.items()), world_part)
    regions = {key: sample_cube_top(key, f"/World/Part/{key}Marker", transforms[f"/World/Part/{key}Marker"])
               for key in ("R1", "R2")}
    return scene, regions


def demo_camera(eye=(0.85, -1.05, 0.75), world_part=None):
    world_part = np.eye(4) if world_part is None else world_part
    return CameraModel(world_part @ camera_look_at(eye, (0, 0, 0.08)), 24, 36, 27, (0.01, 100))
