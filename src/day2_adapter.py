"""Day 2 additions; the verified Day 1 adapter remains unchanged."""

import numpy as np
from pxr import Gf, Usd, UsdGeom

from part_geometry import SceneGeometry, cube_triangles, sample_cube_top
from sim_adapter import CaptureAdapter
from visibility import CameraModel


class VisibilityAdapter(CaptureAdapter):
    def set_part(self, definition):
        height = float(definition.rib_height_m)
        if not np.isfinite(height) or height <= 0:
            raise ValueError("Rib height must be positive and finite")
        rib = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.PART_PATH + "/Rib"))
        for operation in rib.GetOrderedXformOps():
            if operation.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                operation.Set(Gf.Vec3d(0, 0, 0.06 + height / 2))
            elif operation.GetOpType() == UsdGeom.XformOp.TypeScale:
                operation.Set(Gf.Vec3f(0.75, 0.045, height))
        self.update()

    def _matrix(self, prim):
        return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float).T

    def geometry_snapshot(self):
        world_part = self.world_transform(self.PART_PATH)
        triangles, prim_paths, face_ids = [], [], []
        for prim in sorted(self.stage.Traverse(), key=lambda value: str(value.GetPath())):
            if prim.IsA(UsdGeom.Mesh):
                raise ValueError("Day 2 ray-cast adapter supports USD Cube geometry only")
            if not prim.IsA(UsdGeom.Cube) or UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
                continue
            cube = UsdGeom.Cube(prim)
            faces, ids = cube_triangles(self._matrix(prim), cube.GetSizeAttr().Get())
            triangles.extend(faces)
            prim_paths.extend([str(prim.GetPath())] * len(faces))
            face_ids.extend(ids)
        scene = SceneGeometry(triangles, tuple(prim_paths), tuple(face_ids), world_part, self.PART_PATH)
        regions = {}
        for region_id in ("R1", "R2"):
            path = f"{self.PART_PATH}/{region_id}Marker"
            prim = self.stage.GetPrimAtPath(path)
            part_cube = np.linalg.inv(world_part) @ self._matrix(prim)
            regions[region_id] = sample_cube_top(region_id, path, part_cube, size=UsdGeom.Cube(prim).GetSizeAttr().Get())
        return scene, regions

    def camera_model(self):
        camera = UsdGeom.Camera(self.stage.GetPrimAtPath(self.CAMERA_PATH))
        if camera.GetProjectionAttr().Get() != UsdGeom.Tokens.perspective:
            raise ValueError("Visibility evaluator requires a perspective camera")
        return CameraModel(self.world_transform(self.CAMERA_PATH), camera.GetFocalLengthAttr().Get(),
                           camera.GetHorizontalApertureAttr().Get(), camera.GetVerticalApertureAttr().Get(),
                           tuple(camera.GetClippingRangeAttr().Get()), tuple(self.sensor.resolution))
