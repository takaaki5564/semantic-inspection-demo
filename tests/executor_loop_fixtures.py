"""Numerical camera executor fixtures; never used by the simulator runner."""

import numpy as np

from arm_inspection_geometry import scene_fingerprint
from arm_kinematics import pose_error
from geometry_fixtures import matrix, scene_of_boxes
from loop_fixtures import FakeAdapter
from part_geometry import SceneGeometry


class FakeCell(FakeAdapter):
    def __init__(self,*args,blocked=(),**kwargs):
        super().__init__(*args,**kwargs)
        self.blocked = set(blocked)
        self.tick, self.prepare_count, self.move_count = 0, 0, 0

    def snapshot(self):
        return self.scene,self.regions,self.camera,{
            "full_scene_sha256":scene_fingerprint(self.scene),
            "static_scene_sha256":scene_fingerprint(self.scene,exclude_robot=True),
            "physics_step_index":self.tick,"physics_time_seconds":self.tick/240}

    def prepare_candidates(self,views,current):
        self.prepare_count += 1
        eligible = tuple(v for v in views if v.view_id not in self.blocked)
        records = [{"view_id":v.view_id,"status":"blocked" if v.view_id in self.blocked else "feasible",
                    "reason":"test_fixture_ik_failed" if v.view_id in self.blocked else "test_fixture_pose"} for v in views]
        return eligible,records,{v.view_id:current[0] for v in eligible}

    def move_to_part_pose(self,pose):
        self.move_count += 1
        self.tick += 240
        if self.fault == "actual_arm_occlusion" and self.move_count > 1:
            extra = scene_of_boxes([("/World/UR10e/link/visuals/slab",matrix((0,0,.4),(4,4,.2)))])
            self.scene = SceneGeometry(np.concatenate([self.scene.triangles_world_m,extra.triangles_world_m]),
                                       self.scene.prim_paths+extra.prim_paths,self.scene.face_ids+extra.face_ids,self.scene.T_world_part)
        super().move_to_part_pose(pose)

    def capture(self,image):
        rgb,record = super().capture(image)
        record.update(physics_step_index=self.tick,physics_time_seconds=self.tick/240,physics_held_during_capture=True)
        return rgb,record


class FakeExecutor:
    def __init__(self,adapter):
        self.adapter = adapter

    def move_to_part_pose(self,pose,*,goal_id,emit):
        if self.adapter.fault == "blocked_move" and self.adapter.move_count:
            return {"status":"blocked","reason":"fixture_motion_timeout","goal_id":goal_id}
        self.adapter.move_to_part_pose(pose)
        error = pose_error(self.adapter.camera.T_world_camera,self.adapter.scene.T_world_part@pose)
        return {"status":"reached","reason":"fixture_measured_pose","goal_id":goal_id,
                "camera_pose_error":error,"T_part_camera_target":np.asarray(pose).tolist()}

    def capture(self,image):
        return self.adapter.capture(image)
