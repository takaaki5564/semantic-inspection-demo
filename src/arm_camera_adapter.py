"""Fixed flange camera and synchronized held-pose RGB for Isaac Sim 6.1."""

from datetime import datetime, timezone

import numpy as np

from arm_camera_evidence import DEFAULT_MOUNT, assert_pose_close, mounted_capture_record
from arm_motion_adapter import ArmMotionAdapter
from capture_metadata import reference_time, rigid_transform
from camera_mount_geometry import MOUNT_CUBES
from render_camera_evidence import RenderCameraPosePending


class ArmCameraAdapter(ArmMotionAdapter):
    PART_PATH = "/World/CaptureBoard"

    def __init__(self, app, asset, model_dir, reference, *, headless):
        super().__init__(app, asset, model_dir, reference, headless=headless)
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from pxr import Gf, UsdGeom
        from arm_preflight_adapter import FLANGE_PATH

        self.rep = rep
        self.timeline = omni.timeline.get_timeline_interface()
        self.stage = omni.usd.get_context().get_stage()
        self.CAMERA_PATH = FLANGE_PATH + "/WristCamera"
        self.mount = rigid_transform(DEFAULT_MOUNT).copy()
        self.sensor, self.reference_annotator = None, None
        self.last_reference, self.render_steps = None, 0
        self._create_target()
        self.world_part = self.world_transform(self.PART_PATH)
        # Display geometry is flange-local; no rigid-body/mass or collision properties are added.
        for name, position, dimensions, color in MOUNT_CUBES:
            self._cube(FLANGE_PATH+"/"+name, position, dimensions, color)
        camera = UsdGeom.Camera.Define(self.stage, self.CAMERA_PATH)
        # Author once in the flange frame. Subsequent camera motion comes exclusively from the arm.
        camera.MakeMatrixXform().Set(Gf.Matrix4d(self.mount.T.tolist()))
        rtx = RtxCamera(self.CAMERA_PATH, tick_rate=0.0, reset_xform_op_properties=False)
        rtx.camera.set_focal_lengths(24.0)
        rtx.camera.set_apertures(horizontal_apertures=36.0, vertical_apertures=27.0)
        rtx.camera.set_clipping_ranges(0.01, 100.0)
        self.sensor = CameraSensor(rtx, resolution=(480,640), annotators=["rgb"])
        self.render_product_path = str(self.sensor.render_product.GetPath())
        self.render_camera_paths = [str(p) for p in self.sensor.render_product.GetCameraRel().GetTargets()]
        if self.render_camera_paths != [self.CAMERA_PATH]:
            raise RuntimeError("RGB render product is not bound to the wrist camera")
        self.reference_annotator = rep.AnnotatorRegistry.get_annotator("ReferenceTime")
        self.reference_annotator.attach(self.render_product_path)
        rep.orchestrator.set_capture_on_play(False)
        # Keep the GUI on OverviewCamera. A navigable wrist viewport can write sensor transforms.
        self.camera_settings = {"prim_path": self.CAMERA_PATH, "parent_flange_path": FLANGE_PATH,
                                "T_flange_camera": self.mount.tolist(), "resolution_hw": [480,640],
                                "focal_length_mm":24.0, "horizontal_aperture_mm":36.0,
                                "render_product_path":self.render_product_path,
                                "render_camera_paths":self.render_camera_paths,
                                "gui_view": "external OverviewCamera; sensor is not used for interactive navigation",
                                "mount_geometry": "visual only; attached camera mass and collision not modeled",
                                "mount_visual_cubes": [dict(name=n, position_m=p, dimensions_m=d) for n,p,d,_ in MOUNT_CUBES],
                                "capture": "stopped timeline; reference time set to held physics time; render delta_time=0"}

    def _create_target(self):
        from pxr import Gf, UsdGeom

        board = UsdGeom.Xform.Define(self.stage, self.PART_PATH)
        board.AddTranslateOp().Set(Gf.Vec3d(1.05, 0.3, 0.3))
        self._cube(self.PART_PATH+"/Plate", (0,0,-0.02), (1.2,1.2,0.04), (0.7,0.7,0.7))
        tiles = [(-0.25,-0.3,(0.8,0.1,0.1)), (0.25,-0.3,(0.1,0.7,0.1)),
                 (-0.25,0.3,(0.1,0.2,0.8)), (0.25,0.3,(0.8,0.6,0.1))]
        for i,(x,y,color) in enumerate(tiles):
            self._cube(self.PART_PATH+f"/Tile{i}", (x,y,0.002), (0.42,0.42,0.004), color)

    def _cube(self, path, position, dimensions, color):
        from pxr import Gf, UsdGeom

        cube = UsdGeom.Cube.Define(self.stage,path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(*dimensions))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])

    def world_transform(self, path):
        from pxr import Usd, UsdGeom

        matrix = UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return rigid_transform(np.asarray(matrix,dtype=float).T)

    def capture(self, image, movement):
        from pxr import Usd, UsdGeom

        if self.timeline.is_playing():
            raise RuntimeError("Camera capture requires a stopped timeline")
        if [str(p) for p in self.sensor.render_product.GetCameraRel().GetTargets()] != [self.CAMERA_PATH]:
            raise RuntimeError("RGB render camera changed")
        before = self.read_state()
        camera_before = self.world_transform(self.CAMERA_PATH)
        assert_pose_close(self.world_transform(self.PART_PATH), self.world_part, "Capture board moved")
        # Only the renderer clock is placed at the current physical snapshot; no physics stepping here.
        self.timeline.set_current_time(before["simulation_time_s"])
        renderer_rejections = []
        for attempt in range(1, 11):
            if not self.is_running():
                raise RuntimeError("Window closed before wrist RGB capture")
            self.rep.orchestrator.step(rt_subframes=4, delta_time=0.0, wait_for_render=True)
            self.render_steps += 1
            data, _ = self.sensor.get_data("rgb")
            reference = self.reference_annotator.get_data()
            if data is None or not reference:
                continue
            if self.last_reference is not None and reference_time(reference) <= self.last_reference:
                continue
            rgb = data.numpy().copy()
            if rgb.shape != (480,640,3) or rgb.dtype != np.uint8 or np.ptp(rgb) == 0:
                continue
            after = self.read_state()
            if self.timeline.is_playing():
                raise RuntimeError("Timeline started playing during RGB capture")
            camera_after = self.world_transform(self.CAMERA_PATH)
            xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.CAMERA_PATH))
            local = xform.GetLocalTransformation(Usd.TimeCode.Default())
            if xform.GetResetXformStack():
                raise ValueError("Camera reset its transform inheritance")
            assert_pose_close(self.world_transform(self.PART_PATH), self.world_part, "Capture board moved")
            record = mounted_capture_record(image=image, movement=movement, state_before=before, state_after=after,
                                            camera_before=camera_before, camera_after=camera_after,
                                            local_mount=np.asarray(local,dtype=float).T, reference=reference,
                                            previous_reference=self.last_reference, readback_utc=datetime.now(timezone.utc).isoformat(),
                                            render_step_index=self.render_steps)
            try:
                renderer = self._renderer_evidence()
            except RenderCameraPosePending as error:
                renderer_rejections.append({"attempt": attempt, "reason": str(error)})
                continue
            if renderer is not None:
                record.update(renderer_camera=renderer,
                              render_synchronization={"accepted_attempt": attempt, "max_attempts": 10,
                                                      "rejected_renderer_frames": renderer_rejections})
            # Advance freshness only after all pose/clock/mount/render checks pass together.
            self.last_reference = reference_time(reference)
            return rgb, record
        if renderer_rejections:
            raise RuntimeError(f"RGB renderer did not synchronize after 10 held-pose renders: {renderer_rejections[-1]['reason']}")
        raise RuntimeError("No fresh wrist RGB frame after 10 synchronized render attempts")

    def _renderer_evidence(self):
        # The closed-loop adapter opts into CameraParams validation. Earlier probes retain their protocol.
        return None

    def close(self):
        if self.reference_annotator is not None:
            self.reference_annotator.detach(self.render_product_path)
        if self.sensor is not None:
            self.sensor.detach_annotators("rgb")
        self.timeline.stop()
