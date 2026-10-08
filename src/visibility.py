"""Pinhole frustum, incidence and first-surface ray visibility; no simulator imports."""

from dataclasses import dataclass
import math

import numpy as np

from capture_metadata import rigid_transform
from part_geometry import transform_points


@dataclass(frozen=True)
class CameraModel:
    T_world_camera: np.ndarray
    focal_length_mm: float
    horizontal_aperture_mm: float
    vertical_aperture_mm: float
    clipping_range_m: tuple
    resolution_hw: tuple = (480, 640)

    def __post_init__(self):
        pose = rigid_transform(self.T_world_camera).copy()
        pose.setflags(write=False)
        object.__setattr__(self, "T_world_camera", pose)
        optics = [self.focal_length_mm, self.horizontal_aperture_mm, self.vertical_aperture_mm,
                  *self.clipping_range_m]
        if not np.isfinite(optics).all() or min(optics) <= 0 or self.clipping_range_m[0] >= self.clipping_range_m[1]:
            raise ValueError("Invalid pinhole optics or clipping range")
        if len(self.resolution_hw) != 2 or min(self.resolution_hw) <= 0:
            raise ValueError("Invalid camera resolution")

    def project(self, points_world):
        """Return pixel (u,v) and forward depth, using USD -Z optical direction."""
        local = transform_points(np.linalg.inv(self.T_world_camera), points_world)
        depth = -local[:, 2]
        height, width = self.resolution_hw
        pixels = np.full((len(local), 2), np.nan)
        positive = depth > 0
        pixels[positive, 0] = width * (0.5 + local[positive, 0] * self.focal_length_mm / (depth[positive] * self.horizontal_aperture_mm))
        pixels[positive, 1] = height * (0.5 - local[positive, 1] * self.focal_length_mm / (depth[positive] * self.vertical_aperture_mm))
        return pixels, depth

    def to_dict(self):
        return {"T_world_camera": self.T_world_camera.tolist(), "focal_length_mm": self.focal_length_mm,
                "horizontal_aperture_mm": self.horizontal_aperture_mm,
                "vertical_aperture_mm": self.vertical_aperture_mm,
                "clipping_range_m": list(self.clipping_range_m), "resolution_hw": list(self.resolution_hw)}


def first_hit(scene, origin, direction, *, max_distance_m, epsilon=1e-10):
    """Double-sided Moller-Trumbore ray cast against the read-back USD geometry."""
    origin, direction = np.asarray(origin, dtype=float), np.asarray(direction, dtype=float)
    if not np.isfinite([origin, direction]).all() or not np.isclose(np.linalg.norm(direction), 1):
        raise ValueError("Ray requires finite origin and unit direction")
    triangles = scene.triangles_world_m
    edge1 = triangles[:, 1] - triangles[:, 0]
    edge2 = triangles[:, 2] - triangles[:, 0]
    pvec = np.cross(direction, edge2)
    determinant = np.einsum("ij,ij->i", edge1, pvec)
    inverse = np.divide(1.0, determinant, out=np.zeros_like(determinant), where=np.abs(determinant) > epsilon)
    tvec = origin - triangles[:, 0]
    u = np.einsum("ij,ij->i", tvec, pvec) * inverse
    qvec = np.cross(tvec, edge1)
    v = (qvec @ direction) * inverse
    distance = np.einsum("ij,ij->i", edge2, qvec) * inverse
    valid = ((np.abs(determinant) > epsilon) & (u >= -epsilon) & (v >= -epsilon)
             & (u + v <= 1 + epsilon) & (distance > epsilon) & (distance <= max_distance_m))
    if not np.any(valid):
        return None
    index = int(np.argmin(np.where(valid, distance, np.inf)))
    return {"prim_path": scene.prim_paths[index], "face_id": scene.face_ids[index],
            "distance_m": float(distance[index]),
            "point_world_m": (origin + distance[index] * direction).tolist()}


def evaluate_visibility(scene, regions, camera, *, required_region_ids, inspection_spec_version,
                        max_incidence_angle_deg=75.0, surface_tolerance_m=1e-5):
    """Evaluate all required surface points; no part-name-specific rules or percentages."""
    required = tuple(required_region_ids)
    if not required or len(set(required)) != len(required) or any(region not in regions for region in required):
        raise ValueError("Required regions must be explicit, unique and present in geometry")
    if not 0 < max_incidence_angle_deg < 90 or surface_tolerance_m <= 0:
        raise ValueError("Invalid visibility tolerances")
    eye = camera.T_world_camera[:3, 3]
    cosine_limit = math.cos(math.radians(max_incidence_angle_deg))
    height, width = camera.resolution_hw
    result = {}
    for region_id in required:
        region = regions[region_id]
        world_points = transform_points(scene.T_world_part, region.points_part_m)
        normal = scene.T_world_part[:3, :3] @ region.normal_part
        pixels, depth = camera.project(world_points)
        points = []
        for index, (point_id, point, pixel, z) in enumerate(zip(region.point_ids, world_points, pixels, depth)):
            ray = point - eye
            distance = np.linalg.norm(ray)
            hit = None
            reason = "visible"
            if not (camera.clipping_range_m[0] <= z <= camera.clipping_range_m[1]
                    and np.isfinite(pixel).all() and 0 <= pixel[0] < width and 0 <= pixel[1] < height):
                reason = "outside_frustum"
            elif distance < surface_tolerance_m or np.dot(normal, -ray / distance) < cosine_limit:
                reason = "back_facing_or_grazing"
            else:
                hit = first_hit(scene, eye, ray / distance, max_distance_m=distance + surface_tolerance_m)
                if hit is None:
                    reason = "surface_not_hit"
                elif (hit["prim_path"] == region.prim_path and hit["face_id"] == region.face_id
                      and abs(hit["distance_m"] - distance) <= surface_tolerance_m):
                    reason = "visible"
                elif hit["distance_m"] < distance - surface_tolerance_m:
                    reason = "self_occlusion" if hit["prim_path"].startswith(scene.part_root + "/") else "scene_occlusion"
                else:
                    reason = "wrong_surface_hit"
            points.append({"point_id": point_id, "point_part_m": region.points_part_m[index].tolist(),
                           "point_world_m": point.tolist(), "pixel_uv": pixel.tolist() if np.isfinite(pixel).all() else None,
                           "visible": reason == "visible", "reason": reason, "hit": hit})
        visible = sum(point["visible"] for point in points)
        reasons = {reason: sum(point["reason"] == reason for point in points)
                   for reason in sorted({point["reason"] for point in points})}
        result[region_id] = {"target_prim_path": region.prim_path, "target_face_id": region.face_id,
                             "visible_count": visible, "total_count": len(points),
                             "coverage": visible / len(points), "reason_counts": reasons, "points": points}
    return {"evidence_source": "derived_from_simulator_geometry", "method": "pinhole_incidence_cpu_triangle_raycast",
            "part_geometry_version": scene.geometry_version, "inspection_spec_version": inspection_spec_version,
            "required_region_ids": list(required), "max_incidence_angle_deg": max_incidence_angle_deg,
            "surface_tolerance_m": surface_tolerance_m, "regions": result}
