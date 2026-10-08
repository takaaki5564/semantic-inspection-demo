"""Predict robot visual geometry at an IK solution without modifying the simulator."""

import numpy as np

from arm_kinematics import axis_rotation
from arm_inspection_geometry import ROBOT_ROOT, scene_fingerprint
from capture_metadata import rigid_transform
from part_geometry import SceneGeometry, transform_points


def body_poses(reference, world_base, joints_by_name):
    joints = reference["joints"]
    if set(joints_by_name) != {j["name"] for j in joints}:
        raise ValueError("Prediction requires every validated USD joint")
    transform = rigid_transform(world_base).copy()
    result = {reference["base_prim_path"]: transform}
    parent = reference["base_prim_path"]
    for joint in joints:
        q = joints_by_name[joint["name"]]
        if (joint["parent_path"] != parent or not np.isfinite(q)
                or not joint["lower_limit_rad"] <= q <= joint["upper_limit_rad"]):
            raise ValueError("Invalid prediction joint chain or joint limits")
        transform = (transform @ rigid_transform(joint["T_parent_joint"]) @ axis_rotation(joint["axis"], q)
                     @ np.linalg.inv(rigid_transform(joint["T_child_joint"])))
        result[joint["child_path"]] = transform
        parent = joint["child_path"]
    return result


def predict_robot_scene(scene, reference, world_base, joints_by_name, actual_body_poses):
    """Rebase actual visible triangles onto FK bodies for a candidate joint vector.

    Actual body poses come from the same USD snapshot as the triangles. Fixed mesh,
    flange, bracket and camera offsets therefore remain attached to their bodies.
    This is a kinematic visibility prediction, not collision-aware motion planning.
    """
    predicted = body_poses(reference, world_base, joints_by_name)
    if set(actual_body_poses) != set(predicted):
        raise ValueError("Actual snapshot must contain every robot body pose")
    transforms = {path: predicted[path]@np.linalg.inv(rigid_transform(actual_body_poses[path])) for path in predicted}
    triangles = scene.triangles_world_m.copy()
    groups = {path: [] for path in predicted}
    for index, path in enumerate(scene.prim_paths):
        if path == ROBOT_ROOT or path.startswith(ROBOT_ROOT+"/"):
            owners = [body for body in predicted if path == body or path.startswith(body+"/")]
            if not owners:
                raise ValueError(f"Robot geometry has no validated body owner: {path}")
            groups[max(owners, key=len)].append(index)
    for path, indices in groups.items():
        if indices:
            values = triangles[indices]
            triangles[indices] = transform_points(transforms[path], values.reshape(-1,3)).reshape(values.shape)
    result = SceneGeometry(triangles, scene.prim_paths, scene.face_ids, scene.T_world_part, scene.part_root)
    if (result.geometry_version != scene.geometry_version
            or scene_fingerprint(result, exclude_robot=True) != scene_fingerprint(scene, exclude_robot=True)):
        raise ValueError("Robot prediction changed static part/cell geometry")
    return result
