"""USD-joint forward kinematics and model comparison; no Isaac Sim imports."""

import math

import numpy as np

from capture_metadata import rigid_transform


def axis_rotation(axis, angle_rad):
    axis = np.asarray(axis, dtype=float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or not np.isfinite(angle_rad) or np.linalg.norm(axis) < 1e-12:
        raise ValueError("Expected a finite rotation angle and nonzero 3-vector axis")
    x, y, z = axis / np.linalg.norm(axis)
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    result = np.eye(4)
    result[:3, :3] += math.sin(angle_rad) * skew + (1 - math.cos(angle_rad)) * (skew @ skew)
    return result


def pose_error(actual, expected):
    actual, expected = rigid_transform(actual), rigid_transform(expected)
    relative = expected[:3, :3].T @ actual[:3, :3]
    sine = np.linalg.norm([relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0],
                           relative[1, 0] - relative[0, 1]]) / 2
    cosine = np.clip((np.trace(relative) - 1) / 2, -1, 1)
    return {"position_error_m": float(np.linalg.norm(actual[:3, 3] - expected[:3, 3])),
            "orientation_error_rad": float(math.atan2(sine, cosine))}


def usd_joint_fk(reference, joint_positions_rad):
    """Compose body0 * local0 * rotation(q) * inverse(local1), then flange offset."""
    joints = reference["joints"]
    if not joints or len({j["name"] for j in joints}) != len(joints) or set(joint_positions_rad) != {j["name"] for j in joints}:
        raise ValueError("Joint positions must identify every USD-chain joint exactly once")
    transform = np.eye(4)
    for joint in joints:
        angle = joint_positions_rad[joint["name"]]
        if not np.isfinite(angle) or not joint["lower_limit_rad"] <= angle <= joint["upper_limit_rad"]:
            raise ValueError(f"Joint position outside USD limits: {joint['name']}")
        transform = (transform @ rigid_transform(joint["T_parent_joint"])
                     @ axis_rotation(joint["axis"], angle) @ np.linalg.inv(rigid_transform(joint["T_child_joint"])))
    return rigid_transform(transform @ rigid_transform(reference["T_leaf_flange"]))


def fk_cases(joint_names):
    """Fixed numerical probes, independent of part/specification/camera viewpoints."""
    names = list(joint_names)
    cases = [{"case_id": "zero", "joint_positions_rad": dict.fromkeys(names, 0.0)}]
    for name in names:
        for sign, label in ((1, "positive"), (-1, "negative")):
            positions = dict.fromkeys(names, 0.0)
            positions[name] = sign * 0.35
            cases.append({"case_id": f"{name}_{label}", "joint_positions_rad": positions})
    for label, values in (("mixed_01", np.linspace(-0.6, 0.6, len(names))),
                          ("mixed_02", np.linspace(0.5, -0.4, len(names)))):
        cases.append({"case_id": label, "joint_positions_rad": dict(zip(names, values.tolist()))})
    return cases


def compare_model(reference, *, model_joint_names, model_limits_rad, model_pose,
                  position_tolerance_m=1e-5, orientation_tolerance_rad=1e-5, on_case=None):
    """Compare an independent model FK against raw USD joint frames and authored zero pose."""
    if not all(np.isfinite(v) and v > 0 for v in (position_tolerance_m, orientation_tolerance_rad)):
        raise ValueError("FK tolerances must be finite and positive")
    names = list(model_joint_names)
    usd_names = [j["name"] for j in reference["joints"]]
    if not usd_names or len(names) != len(usd_names) or len(set(names)) != len(names) or set(names) != set(usd_names):
        raise ValueError("Model joint names do not match the USD chain")
    limits_match = set(model_limits_rad) == set(names) and all(
        np.allclose(model_limits_rad[j["name"]], [j["lower_limit_rad"], j["upper_limit_rad"]], rtol=0, atol=1e-7)
        for j in reference["joints"])

    def within(error):
        return error["position_error_m"] <= position_tolerance_m and error["orientation_error_rad"] <= orientation_tolerance_rad

    zero_pose = usd_joint_fk(reference, dict.fromkeys(usd_names, 0.0))
    zero_error = pose_error(zero_pose, reference["T_base_flange_authored"])
    results = []
    for case in fk_cases(usd_names):
        positions = case["joint_positions_rad"]
        expected = usd_joint_fk(reference, positions)
        # cuMotion/physics orders need not equal USD path sorting: map by name.
        actual = rigid_transform(model_pose(np.array([positions[name] for name in names])))
        error = pose_error(actual, expected)
        record = {**case, "model_joint_vector_rad": [positions[name] for name in names],
                  "T_base_flange_usd_joint_fk": expected.tolist(), "T_base_flange_model_fk": actual.tolist(),
                  **error, "passed": within(error)}
        results.append(record)
        if on_case:
            on_case(record)
    return {"model_joint_order": names, "usd_joint_order": usd_names,
            "position_tolerance_m": position_tolerance_m, "orientation_tolerance_rad": orientation_tolerance_rad,
            "authored_zero_pose_error": zero_error, "cases": results,
            "max_position_error_m": max(r["position_error_m"] for r in results),
            "max_orientation_error_rad": max(r["orientation_error_rad"] for r in results),
            "checks": {"joint_names_match": True, "joint_limits_match": bool(limits_match),
                       "authored_zero_pose_matches_joint_frames": within(zero_error),
                       "all_fk_cases_within_tolerance": all(r["passed"] for r in results)}}
