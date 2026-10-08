"""Shared fixed cell with either a free camera or the verified UR10e wrist camera."""

from datetime import datetime, timezone
import time

import numpy as np

from arm_camera_evidence import assert_pose_close
from arm_inspection import screen_candidates
from arm_inspection_adapter import ArmInspectionAdapter
from arm_inspection_geometry import scene_fingerprint
from arm_view_prediction import predict_robot_scene
from capture_metadata import capture_record, reference_time, rigid_transform
from render_camera_evidence import validate_render_camera


class InspectionLoopAdapter(ArmInspectionAdapter):
    def __init__(self, app, asset, model_dir, reference, *, executor_mode, **kwargs):
        self.executor_mode = executor_mode
        self.reference = reference
        super().__init__(app, asset, model_dir, reference, **kwargs)
        if executor_mode == "free":
            self._create_free_camera()
            self._settle_parked_robot()
        elif executor_mode != "arm":
            raise ValueError("Unknown executor mode")
        self.camera_parameters = self.rep.AnnotatorRegistry.get_annotator("CameraParams")
        self.camera_parameters.attach(self.render_product_path)

    def _create_free_camera(self):
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from pxr import Gf, UsdGeom

        # Detach the wrist RGB annotators; the physical wrist camera and its geometry remain in the cell.
        self.reference_annotator.detach(self.render_product_path)
        self.sensor.detach_annotators("rgb")
        # Keep the first sensor/texture alive until shutdown. In this experimental runtime,
        # destroying it during the switch can invalidate the replacement's camera/scene rendering.
        self.parked_wrist_sensor = self.sensor
        self.CAMERA_PATH = "/World/FreeInspectionCamera"
        prim = UsdGeom.Camera.Define(self.stage, self.CAMERA_PATH)
        prim.MakeMatrixXform().Set(Gf.Matrix4d(1))
        camera = RtxCamera(self.CAMERA_PATH, tick_rate=0.0, reset_xform_op_properties=False)
        self.free_authoring = camera
        camera.camera.set_focal_lengths(24.0)
        camera.camera.set_apertures(horizontal_apertures=36.0, vertical_apertures=27.0)
        camera.camera.set_clipping_ranges(0.01,100.0)
        self.sensor = CameraSensor(camera, resolution=(480,640), annotators=["rgb"])
        self.render_product_path = str(self.sensor.render_product.GetPath())
        self.reference_annotator = self.rep.AnnotatorRegistry.get_annotator("ReferenceTime")
        self.reference_annotator.attach(self.render_product_path)
        self.render_camera_paths = [str(p) for p in self.sensor.render_product.GetCameraRel().GetTargets()]
        if self.render_camera_paths != [self.CAMERA_PATH]:
            raise RuntimeError("Free RGB render product camera mismatch")
        self.camera_settings = {**self.camera_settings, "prim_path":self.CAMERA_PATH, "parent_flange_path":None,
                                "T_flange_camera":None, "render_product_path":self.render_product_path,
                                "render_camera_paths":self.render_camera_paths,
                                "capture":"same held-physics RGB protocol; one joint-hold physics step after each free-camera move advances sensor time",
                                "robot_state":"same robot/cell present; joint targets held at initial parking pose; actual geometry read at each capture"}

    def _settle_parked_robot(self):
        self.parking_joints = np.asarray(self.read_state()["joints_rad"],dtype=float)
        self.camera_settings["parking_joint_targets_rad"] = self.parking_joints.tolist()
        settled = 0.0
        for _ in range(int(3/self.dt)):
            self.command(self.parking_joints)
            self.step()
            state = self.read_state()
            if not self.is_running() or np.max(np.abs(np.asarray(state["joints_rad"])-self.parking_joints)) > .01:
                raise RuntimeError("Robot could not maintain the parking joint targets")
            settled = settled+self.dt if np.max(np.abs(state["velocities_rad_s"])) <= .02 else 0.0
            if settled >= .25:
                return
        raise RuntimeError("Parking joint targets did not settle within 3 simulation seconds")

    def snapshot(self):
        from pxr import Usd, UsdGeom

        result = super().snapshot()
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        paths = [self.reference["base_prim_path"], *(j["child_path"] for j in self.reference["joints"])]
        result[3]["robot_body_world_poses"] = {
            path:rigid_transform(np.asarray(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path)),dtype=float).T).tolist()
            for path in paths}
        physical = self.read_state()
        if (result[3]["physics_step_index"] != physical["physics_step_index"]
                or result[3]["physics_time_seconds"] != physical["simulation_time_s"]):
            raise RuntimeError("Physics advanced while reading body transforms")
        assert_pose_close(result[3]["robot_body_world_poses"][self.reference["base_prim_path"]], physical["T_world_base"], "USD/physical base mismatch")
        return result

    def prepare_candidates(self, candidates, current):
        state = self.read_state()
        if (state["physics_step_index"] != current[3]["physics_step_index"]
                or state["simulation_time_s"] != current[3]["physics_time_seconds"]):
            raise ValueError("Candidate prediction requires the current physical snapshot")
        if self.executor_mode == "free":
            scene = predict_robot_scene(current[0],self.reference,state["T_world_base"],
                                        dict(zip(self.names,self.parking_joints)),current[3]["robot_body_world_poses"])
            records = [{"view_id":v.view_id, "status":"feasible", "reason":"free_camera_pose_change",
                        "prediction_source":"validated USD-chain FK at unchanged robot parking joint targets",
                        "prediction_scene_sha256":scene_fingerprint(scene), "motion_executed":False}
                       for v in candidates]
            return tuple(candidates), records, {v.view_id:scene for v in candidates}
        feasible, records = screen_candidates(self, self.world_part, self.mount, candidates)
        scenes = {}
        for record in records:
            if record["status"] != "feasible":
                continue
            q = dict(zip(self.names, record["ik"]["joints_rad"]))
            scene = predict_robot_scene(current[0], self.reference, state["T_world_base"], q,
                                        current[3]["robot_body_world_poses"])
            scenes[record["view_id"]] = scene
            record.update(prediction_source="validated USD-chain FK at candidate IK joints; no simulator mutation",
                          prediction_scene_sha256=scene_fingerprint(scene), prediction_joints_rad=q,
                          actual_source_scene_sha256=current[3]["full_scene_sha256"])
        if self.read_state() != state:
            raise RuntimeError("Candidate prediction must not advance physics")
        return feasible, records, scenes

    def move_to_part_pose(self, pose):
        from pxr import Gf
        from isaacsim.core.experimental.utils.backend import use_backend

        if self.executor_mode != "free":
            raise ValueError("Direct camera motion is available only in free mode")
        target = self.world_part@rigid_transform(pose)
        start = self.world_transform(self.CAMERA_PATH)
        a = Gf.Matrix4d(start.T.tolist()).ExtractRotationQuat()
        b = Gf.Matrix4d(target.T.tolist()).ExtractRotationQuat()
        started, duration = time.monotonic(), 0.05 if self.headless else 2.0
        while True:
            u = min((time.monotonic()-started)/duration,1.0)
            blend = u*u*(3-2*u)
            intermediate = np.asarray(Gf.Matrix4d(1).SetRotate(Gf.Slerp(blend,a,b)),dtype=float).T.copy()
            intermediate[:3,3] = (1-blend)*start[:3,3]+blend*target[:3,3]
            actual = target if u == 1 else intermediate
            quaternion = Gf.Matrix4d(actual.T.tolist()).ExtractRotationQuat()
            orientation = [quaternion.GetReal(),*quaternion.GetImaginary()]
            # Physics initialized the Fabric hierarchy before this free camera was created.
            # Keep both USD evidence and the renderer's USDRT world pose in sync via the supported API.
            for backend in ("usd","usdrt"):
                with use_backend(backend):
                    self.free_authoring.set_world_poses(positions=actual[:3,3],orientations=orientation)
            if not self.is_running() or self.timeline.is_playing():
                raise RuntimeError("Free-camera movement interrupted or timeline started")
            self.app.update()
            if u == 1:
                # RTX ReferenceTime follows the physical frame, even with a stopped timeline.
                self.command(self.parking_joints)
                self.step()
                return
            if not self.headless:
                time.sleep(1/120)

    def capture(self, image, movement=None):
        if self.executor_mode == "arm":
            rgb, record = super().capture(image,movement)
            record["renderer_camera"] = self._renderer_evidence()
            return rgb, record
        from pxr import Gf

        if self.timeline.is_playing():
            raise RuntimeError("Free capture requires held physics")
        if [str(p) for p in self.sensor.render_product.GetCameraRel().GetTargets()] != [self.CAMERA_PATH]:
            raise RuntimeError("Free RGB render camera changed")
        before, camera = self.read_state(), self.world_transform(self.CAMERA_PATH)
        assert_pose_close(self.world_transform(self.PART_PATH),self.world_part,"Part moved")
        self.timeline.set_current_time(before["simulation_time_s"])
        for _ in range(10):
            if not self.is_running():
                raise RuntimeError("Window closed before free RGB capture")
            self.rep.orchestrator.step(rt_subframes=4,delta_time=0.0,wait_for_render=True)
            self.render_steps += 1
            data, reference = self.sensor.get_data("rgb")[0], self.reference_annotator.get_data()
            if data is None or not reference:
                continue
            clock = reference_time(reference)
            if self.last_reference is not None and clock <= self.last_reference:
                continue
            rgb = data.numpy().copy()
            if rgb.shape != (480,640,3) or rgb.dtype != np.uint8 or np.ptp(rgb) == 0:
                continue
            after = self.read_state()
            if before != after or self.timeline.is_playing():
                raise RuntimeError("Physics moved during free RGB capture")
            if abs(float(clock)-after["simulation_time_s"]) > 1e-7:
                raise RuntimeError("Free RGB reference time does not match held physics time")
            if np.max(np.abs(np.asarray(after["joints_rad"])-self.parking_joints)) > .01:
                raise RuntimeError("Robot moved away from parking joint targets")
            assert_pose_close(self.world_transform(self.CAMERA_PATH),camera,"Free camera moved during capture")
            assert_pose_close(self.world_transform(self.PART_PATH),self.world_part,"Part moved")
            quat = Gf.Matrix4d(camera.T.tolist()).ExtractRotationQuat()
            record = capture_record(image=image,world_part=self.world_part,world_camera=camera,
                                     quaternion_wxyz=[quat.GetReal(),*quat.GetImaginary()],reference=reference,
                                     timeline_seconds=self.timeline.get_current_time(),app_update_count=0,
                                     readback_utc=datetime.now(timezone.utc).isoformat())
            record.pop("app_update_count")
            record.update(executor="free_camera", physics_time_seconds=after["simulation_time_s"],
                          physics_step_index=after["physics_step_index"], physics_held_during_capture=True,
                          render_step_index=self.render_steps,joints_rad=after["joints_rad"],velocities_rad_s=after["velocities_rad_s"],
                          parking_joint_targets_rad=self.parking_joints.tolist(),
                          reference_clock_convention="renderer reference equals held physics time; parking joint targets unchanged")
            self.last_reference = clock
            record["renderer_camera"] = self._renderer_evidence()
            return rgb, record
        raise RuntimeError("No fresh free RGB frame after 10 synchronized renders")

    def _renderer_evidence(self):
        from pxr import UsdGeom
        from visibility import CameraModel

        camera = UsdGeom.Camera(self.stage.GetPrimAtPath(self.CAMERA_PATH))
        model = CameraModel(self.world_transform(self.CAMERA_PATH), camera.GetFocalLengthAttr().Get(),
                            camera.GetHorizontalApertureAttr().Get(), camera.GetVerticalApertureAttr().Get(),
                            tuple(camera.GetClippingRangeAttr().Get()), tuple(self.sensor.resolution))
        return validate_render_camera(self.camera_parameters.get_data(), model)

    def close(self):
        if getattr(self, "camera_parameters", None) is not None:
            self.camera_parameters.detach(self.render_product_path)
        super().close()
