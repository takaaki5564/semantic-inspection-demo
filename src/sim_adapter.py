"""Minimal adapter for installed Isaac Sim 6.1; import AFTER SimulationApp."""

from datetime import datetime, timezone
import time

import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

from capture_metadata import camera_look_at, capture_record, reference_time, rigid_transform


class CaptureAdapter:
    PART_PATH = "/World/Part"
    CAMERA_PATH = "/World/InspectionCamera"

    def __init__(self, app, *, headless=False):
        self.app = app
        self.update_count = 0
        self.last_reference = None
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self.sensor = None
        self.reference_annotator = None
        self._create_scene()
        camera = RtxCamera(self.CAMERA_PATH, tick_rate=0.0)
        camera.camera.set_focal_lengths(24.0)
        camera.camera.set_apertures(horizontal_apertures=36.0, vertical_apertures=27.0)
        camera.camera.set_clipping_ranges(0.01, 100.0)
        self.sensor = CameraSensor(camera, resolution=(480, 640), annotators=["rgb"])
        self.render_product_path = str(self.sensor.render_product.GetPath())
        self.reference_annotator = rep.AnnotatorRegistry.get_annotator("ReferenceTime")
        self.reference_annotator.attach(self.render_product_path)
        rep.orchestrator.set_capture_on_play(False)
        if not headless:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError("No active viewport for GUI demonstration")
            viewport.camera_path = self.CAMERA_PATH

    def _cube(self, path, position, dimensions, color):
        cube = UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        xform.AddScaleOp().Set(Gf.Vec3f(*dimensions))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        return cube.GetPrim()

    def _create_scene(self):
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        world = UsdGeom.Xform.Define(self.stage, "/World")
        self.stage.SetDefaultPrim(world.GetPrim())
        part = UsdGeom.Xform.Define(self.stage, self.PART_PATH)
        UsdGeom.Xformable(part.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.25))
        self._cube("/World/Floor", (0, 0, -0.03), (4, 4, 0.06), (0.16, 0.18, 0.21))
        self._cube("/World/Support", (0, 0, 0.12), (1.1, 0.75, 0.24), (0.23, 0.25, 0.29))
        plate = self._cube(self.PART_PATH + "/Plate", (0, 0, 0.03), (0.9, 0.6, 0.06), (0.55, 0.58, 0.62))
        rib = self._cube(self.PART_PATH + "/Rib", (0, 0, 0.115), (0.75, 0.045, 0.11), (0.55, 0.58, 0.62))
        # Visualization markers on the plate. Visibility/inspection semantics are Day 2 work.
        self._cube(self.PART_PATH + "/R1Marker", (-0.08, -0.17, 0.061), (0.5, 0.15, 0.002), (0.12, 0.7, 0.35))
        self._cube(self.PART_PATH + "/R2Marker", (0.08, 0.17, 0.061), (0.5, 0.15, 0.002), (0.95, 0.45, 0.08))
        material = UsdShade.Material.Define(self.stage, "/World/Looks/Metal")
        shader = UsdShade.Shader.Define(self.stage, "/World/Looks/Metal/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.55, 0.58, 0.62))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.6)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        for prim in (plate, rib):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
        dome = UsdLux.DomeLight.Define(self.stage, "/World/FillLight")
        dome.CreateIntensityAttr(350.0)
        key = UsdLux.DistantLight.Define(self.stage, "/World/KeyLight")
        key.CreateIntensityAttr(1800.0)
        key.CreateAngleAttr(2.0)
        UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-30, -20, 15))

    def update(self):
        if not self.app.is_running():
            raise RuntimeError("Isaac Sim window closed before capture completed")
        self.app.update()
        self.update_count += 1

    def hold(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.update()
            time.sleep(1 / 120)

    def world_transform(self, path):
        matrix = UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # Gf uses row vectors; our recorded matrices use column vectors.
        return rigid_transform(np.array(matrix, dtype=float).T)

    def set_part_relative_view(self, eye, target):
        world_camera = self.world_transform(self.PART_PATH) @ camera_look_at(eye, target)
        camera = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.CAMERA_PATH))
        # A single matrix op prevents accumulation of transforms when moving.
        camera.MakeMatrixXform().Set(Gf.Matrix4d(world_camera.T.tolist()))

    def move_view(self, start_eye, end_eye, target, *, seconds=3.0):
        start = time.monotonic()
        while True:
            fraction = min((time.monotonic() - start) / seconds, 1.0)
            smooth = fraction * fraction * (3 - 2 * fraction)
            eye = (1 - smooth) * np.asarray(start_eye) + smooth * np.asarray(end_eye)
            self.set_part_relative_view(eye, target)
            self.update()
            if fraction == 1:
                return
            time.sleep(1 / 120)

    def capture(self, image):
        world_part = self.world_transform(self.PART_PATH)
        world_camera = self.world_transform(self.CAMERA_PATH)
        # Keep pose fixed throughout synchronized steps and RGB readback.
        for attempt in range(10):
            if not self.app.is_running():
                raise RuntimeError("Window closed before RGB capture")
            # RTX sensor reference time advances only with a playing timeline.
            # Commit play as in the installed CameraSensor tests; step pauses it again.
            self.timeline.play()
            self.timeline.commit()
            rep.orchestrator.step(rt_subframes=4, delta_time=1 / 60, wait_for_render=True)
            data, _ = self.sensor.get_data("rgb")
            reference = self.reference_annotator.get_data()
            if data is None or not reference:
                continue
            clock = reference_time(reference)
            if self.last_reference is not None and clock <= self.last_reference:
                continue
            rgb = data.numpy().copy()
            if rgb.shape != (480, 640, 3) or rgb.dtype != np.uint8 or np.ptp(rgb) == 0:
                continue
            if not np.allclose(world_camera, self.world_transform(self.CAMERA_PATH), atol=1e-9):
                raise RuntimeError("Camera moved during capture")
            if not np.allclose(world_part, self.world_transform(self.PART_PATH), atol=1e-9):
                raise RuntimeError("Part moved during capture")
            gf_matrix = Gf.Matrix4d(world_camera.T.tolist())
            quaternion = gf_matrix.ExtractRotationQuat()
            record = capture_record(
                image=image, world_part=world_part, world_camera=world_camera,
                quaternion_wxyz=[quaternion.GetReal(), *quaternion.GetImaginary()],
                reference=reference, timeline_seconds=self.timeline.get_current_time(),
                app_update_count=self.update_count,
                readback_utc=datetime.now(timezone.utc).isoformat(),
            )
            self.last_reference = clock
            return rgb, record
        raise RuntimeError(f"No valid fresh RGB frame after 10 synchronized capture attempts; "
                           f"previous_reference={self.last_reference}, latest_reference={reference}")

    def close(self):
        self.timeline.stop()
        if self.reference_annotator is not None:
            self.reference_annotator.detach(self.render_product_path)
        if self.sensor is not None:
            self.sensor.detach_annotators("rgb")
