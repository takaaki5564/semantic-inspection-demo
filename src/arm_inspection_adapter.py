"""Independent A/B cell using the verified fixed wrist RGB camera."""

from arm_camera_adapter import ArmCameraAdapter
from arm_inspection_geometry import read_geometry
from arm_inspection import solve_posture_ik
from capture_metadata import rigid_transform


class ArmInspectionAdapter(ArmCameraAdapter):
    PART_PATH = "/World/Part"

    def __init__(self, app, asset, model_dir, reference, *, headless, definition, world_part, cell):
        self.definition = definition
        self.cell_part_pose = rigid_transform(world_part).copy()
        self.ik_seed_postures = cell["ik_seed_postures_rad"]
        self.preferred_ik_posture = cell["preferred_ik_posture_rad"]
        super().__init__(app, asset, model_dir, reference, headless=headless)

    def solve_ik(self, target, start):
        return solve_posture_ik(super().solve_ik, self.model_pose, target, start, self.limits,
                                self.ik_seed_postures, self.preferred_ik_posture)

    def _create_target(self):
        from pxr import Gf, Sdf, UsdGeom, UsdShade

        part = UsdGeom.Xform.Define(self.stage, self.PART_PATH)
        part.MakeMatrixXform().Set(Gf.Matrix4d(self.cell_part_pose.T.tolist()))
        height = self.definition.rib_height_m
        # Same part dimensions, marker surfaces and material as the Day 1-4 scene.
        self._cube(self.PART_PATH+"/Plate", (0,0,0.03), (0.9,0.6,0.06), (0.55,0.58,0.62))
        self._cube(self.PART_PATH+"/Rib", (0,0,0.06+height/2), (0.75,0.045,height), (0.55,0.58,0.62))
        self._cube(self.PART_PATH+"/R1Marker", (-0.08,-0.17,0.061), (0.5,0.15,0.002), (0.12,0.7,0.35))
        self._cube(self.PART_PATH+"/R2Marker", (0.08,0.17,0.061), (0.5,0.15,0.002), (0.95,0.45,0.08))
        support = UsdGeom.Xform.Define(self.stage, "/World/InspectionSupport")
        support.MakeMatrixXform().Set(Gf.Matrix4d(self.cell_part_pose.T.tolist()))
        self._cube(str(support.GetPath())+"/Table", (0,0,-0.13), (1.1,0.75,0.24), (0.23,0.25,0.29))
        material = UsdShade.Material.Define(self.stage, "/World/Looks/InspectionMetal")
        shader = UsdShade.Shader.Define(self.stage, str(material.GetPath())+"/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55,0.58,0.62))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.6)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        for name in ("Plate", "Rib"):
            UsdShade.MaterialBindingAPI.Apply(self.stage.GetPrimAtPath(self.PART_PATH+"/"+name)).Bind(material)

    def snapshot(self):
        from pxr import UsdGeom
        from visibility import CameraModel

        before = self.read_state()
        scene, regions, metadata = read_geometry(self.stage, self.PART_PATH)
        camera = UsdGeom.Camera(self.stage.GetPrimAtPath(self.CAMERA_PATH))
        if camera.GetProjectionAttr().Get() != UsdGeom.Tokens.perspective:
            raise ValueError("Expected a perspective wrist camera")
        model = CameraModel(self.world_transform(self.CAMERA_PATH), camera.GetFocalLengthAttr().Get(),
                            camera.GetHorizontalApertureAttr().Get(), camera.GetVerticalApertureAttr().Get(),
                            tuple(camera.GetClippingRangeAttr().Get()), tuple(self.sensor.resolution))
        after = self.read_state()
        if before != after or self.timeline.is_playing():
            raise RuntimeError("Physics moved while reading visibility geometry")
        metadata.update(physics_step_index=after["physics_step_index"], physics_time_seconds=after["simulation_time_s"])
        return scene, regions, model, metadata
