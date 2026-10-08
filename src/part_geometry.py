"""Explicit USD-cube geometry snapshots and surface samples, independent of Isaac Sim."""

from dataclasses import dataclass
import hashlib
import json

import numpy as np

from capture_metadata import rigid_transform


@dataclass(frozen=True)
class PartDefinition:
    label: str
    rib_height_m: float


PARTS = {"A": PartDefinition("A", 0.025), "B": PartDefinition("B", 0.12)}


def transform_points(transform, points):
    points = np.asarray(points, dtype=float)
    return points @ np.asarray(transform)[:3, :3].T + np.asarray(transform)[:3, 3]


def cube_triangles(world_cube, size=1.0):
    """Triangulate all six USD Cube faces; face IDs distinguish the same Prim's surfaces."""
    corners = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], dtype=float) * (size / 2)
    faces = {"x-": (0, 4, 7, 3), "x+": (1, 2, 6, 5),
             "y-": (0, 1, 5, 4), "y+": (3, 7, 6, 2),
             "z-": (0, 3, 2, 1), "z+": (4, 5, 6, 7)}
    vertices = transform_points(world_cube, corners)
    triangles, face_ids = [], []
    for face, (a, b, c, d) in faces.items():
        triangles.extend((vertices[[a, b, c]], vertices[[a, c, d]]))
        face_ids.extend((face, face))
    return np.asarray(triangles), tuple(face_ids)


@dataclass(frozen=True)
class SurfaceRegion:
    region_id: str
    prim_path: str
    face_id: str
    points_part_m: np.ndarray
    normal_part: np.ndarray

    def __post_init__(self):
        points = np.array(self.points_part_m, dtype=float, copy=True)
        normal = np.array(self.normal_part, dtype=float, copy=True)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or not np.isfinite(points).all():
            raise ValueError("Region requires nonempty finite Nx3 sample points")
        if normal.shape != (3,) or not np.isfinite(normal).all() or not np.isclose(np.linalg.norm(normal), 1):
            raise ValueError("Region normal must be a finite unit vector")
        points.setflags(write=False)
        normal.setflags(write=False)
        object.__setattr__(self, "points_part_m", points)
        object.__setattr__(self, "normal_part", normal)

    @property
    def point_ids(self):
        return tuple(f"{self.region_id}:{index:03d}" for index in range(len(self.points_part_m)))

    def to_dict(self):
        return {"region_id": self.region_id, "prim_path": self.prim_path, "face_id": self.face_id,
                "points_part_m": self.points_part_m.tolist(), "normal_part": self.normal_part.tolist()}


def sample_cube_top(region_id, prim_path, part_cube, *, size=1.0, grid=(9, 7)):
    """Cell-center samples on the actual marker's z+ face, avoiding ambiguous edges."""
    nx, ny = grid
    if nx < 1 or ny < 1:
        raise ValueError("Sampling grid dimensions must be positive")
    points = [[((i + 0.5) / nx - 0.5) * size, ((j + 0.5) / ny - 0.5) * size, size / 2]
              for j in range(ny) for i in range(nx)]
    normal = np.linalg.inv(np.asarray(part_cube)[:3, :3]).T @ [0, 0, 1]
    normal /= np.linalg.norm(normal)
    return SurfaceRegion(region_id, prim_path, "z+", transform_points(part_cube, points), normal)


@dataclass(frozen=True)
class SceneGeometry:
    triangles_world_m: np.ndarray
    prim_paths: tuple
    face_ids: tuple
    T_world_part: np.ndarray
    part_root: str = "/World/Part"

    def __post_init__(self):
        triangles = np.array(self.triangles_world_m, dtype=float, copy=True)
        if triangles.ndim != 3 or triangles.shape[1:] != (3, 3) or not np.isfinite(triangles).all():
            raise ValueError("Expected finite Nx3x3 scene triangles")
        if len(triangles) != len(self.prim_paths) or len(triangles) != len(self.face_ids):
            raise ValueError("Each triangle needs a Prim path and face ID")
        if len(triangles) == 0:
            raise ValueError("Scene geometry must not be empty")
        triangles.setflags(write=False)
        part = rigid_transform(self.T_world_part).copy()
        part.setflags(write=False)
        object.__setattr__(self, "triangles_world_m", triangles)
        object.__setattr__(self, "T_world_part", part)

    @property
    def geometry_version(self):
        # Identity is part-relative: moving the complete part does not change its shape version.
        inverse = np.linalg.inv(self.T_world_part)
        rows = []
        for path, face, triangle in zip(self.prim_paths, self.face_ids, self.triangles_world_m):
            if path.startswith(self.part_root + "/"):
                coordinates = np.round(transform_points(inverse, triangle), 9)
                coordinates[np.abs(coordinates) < 0.5e-9] = 0.0  # canonicalize -0.0
                rows.append((path, face, coordinates.tolist()))
        if not rows:
            raise ValueError("No part geometry in snapshot")
        digest = hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()[:16]
        return f"usd_cube_geometry_v1:{digest}"

    def to_dict(self):
        return {"triangles_world_m": self.triangles_world_m.tolist(), "prim_paths": list(self.prim_paths),
                "face_ids": list(self.face_ids), "T_world_part": self.T_world_part.tolist(),
                "part_root": self.part_root, "geometry_version": self.geometry_version}

    @classmethod
    def from_dict(cls, value):
        scene = cls(value["triangles_world_m"], tuple(value["prim_paths"]), tuple(value["face_ids"]),
                    value["T_world_part"], value["part_root"])
        if scene.geometry_version != value["geometry_version"]:
            raise ValueError("Geometry snapshot version mismatch")
        return scene
