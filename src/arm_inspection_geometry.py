"""Snapshot visible USD Cubes and triangle Meshes, including robot instance proxies.

USD is imported only when reading a stage. Numeric helpers run without Isaac Sim.
Guide/proxy collision geometry is excluded; unsupported renderable geometry fails closed.
"""

import hashlib
import json

import numpy as np

from part_geometry import SceneGeometry, cube_triangles, sample_cube_top, transform_points


ROBOT_ROOT = "/World/UR10e"


def triangle_mesh(points, counts, indices, world_transform, *, subdivision="none", holes=()):
    points = np.asarray(points, dtype=float)
    counts, indices = np.asarray(counts), np.asarray(indices)
    if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
            or counts.ndim != 1 or not len(counts) or not np.all(counts == 3)
            or indices.ndim != 1 or len(indices) != int(np.sum(counts))
            or not np.issubdtype(indices.dtype, np.integer)
            or np.any(indices < 0) or np.any(indices >= len(points))
            or subdivision != "none" or len(holes)):
        raise ValueError("Visibility supports finite triangle Meshes without subdivision or holes")
    triangles = transform_points(world_transform, points)[indices.reshape(-1, 3)]
    area_twice = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]), axis=1)
    if np.any(area_twice <= 1e-14):
        raise ValueError("Degenerate Mesh face cannot be used as visibility evidence")
    return triangles, tuple(f"mesh_face:{i}" for i in range(len(counts)))


def scene_fingerprint(scene, *, exclude_robot=False):
    mask = np.array([not (exclude_robot and (p == ROBOT_ROOT or p.startswith(ROBOT_ROOT+"/")))
                     for p in scene.prim_paths])
    coordinates = np.round(scene.triangles_world_m[mask], 9)
    coordinates[np.abs(coordinates) < 0.5e-9] = 0.0
    identity = [(p, f) for p, f, keep in zip(scene.prim_paths, scene.face_ids, mask) if keep]
    digest = hashlib.sha256(json.dumps(identity, separators=(",", ":")).encode())
    digest.update(coordinates.astype("<f8").tobytes())
    return digest.hexdigest()


def read_geometry(stage, part_path="/World/Part"):
    from pxr import Usd, UsdGeom

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())  # New cache for every physical snapshot.
    def matrix(prim):
        return np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float).T

    triangles, paths, ids, meshes, skipped = [], [], [], [], []
    world_part = matrix(stage.GetPrimAtPath(part_path))
    for prim in sorted(Usd.PrimRange(stage.GetPrimAtPath("/World"), Usd.TraverseInstanceProxies()),
                       key=lambda p: str(p.GetPath())):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        path = str(prim.GetPath())
        imageable = UsdGeom.Imageable(prim)
        if (imageable.ComputeVisibility() == UsdGeom.Tokens.invisible
                or imageable.ComputePurpose() not in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render)):
            skipped.append(path)
            continue
        if prim.IsA(UsdGeom.Cube):
            faces, face_ids = cube_triangles(matrix(prim), UsdGeom.Cube(prim).GetSizeAttr().Get())
        elif prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            faces, face_ids = triangle_mesh(mesh.GetPointsAttr().Get(), mesh.GetFaceVertexCountsAttr().Get(),
                                            mesh.GetFaceVertexIndicesAttr().Get(), matrix(prim),
                                            subdivision=mesh.GetSubdivisionSchemeAttr().Get(),
                                            holes=mesh.GetHoleIndicesAttr().Get() or ())
            meshes.append({"prim_path": path, "triangle_count": len(faces), "instance_proxy": prim.IsInstanceProxy()})
        else:
            raise ValueError(f"Unsupported visible geometry for ray casting: {path} ({prim.GetTypeName()})")
        triangles.extend(faces)
        paths.extend([path]*len(faces))
        ids.extend(face_ids)
    scene = SceneGeometry(triangles, tuple(paths), tuple(ids), world_part, part_path)
    regions = {}
    for name in ("R1", "R2"):
        path = f"{part_path}/{name}Marker"
        prim = stage.GetPrimAtPath(path)
        regions[name] = sample_cube_top(name, path, np.linalg.inv(world_part)@matrix(prim),
                                        size=UsdGeom.Cube(prim).GetSizeAttr().Get())
    metadata = {"full_scene_sha256": scene_fingerprint(scene),
                "static_scene_sha256": scene_fingerprint(scene, exclude_robot=True),
                "part_geometry_version": scene.geometry_version,
                "triangle_count": len(triangles), "visible_meshes": meshes,
                "excluded_invisible_or_guide_proxy_prims": skipped,
                "ray_geometry": "visible default/render Cube and non-subdivided triangle Mesh; double-sided",
                "geometry_source": "actual composed USD at held physics snapshot"}
    return scene, regions, metadata
