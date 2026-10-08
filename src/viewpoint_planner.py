"""One deterministic planning decision. Predictions never update observation state."""

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path

import numpy as np

from capture_metadata import camera_look_at, rigid_transform
from visibility import evaluate_visibility


@dataclass(frozen=True)
class CandidateView:
    view_id: str
    T_part_camera: np.ndarray

    def __post_init__(self):
        if not isinstance(self.view_id, str) or not self.view_id:
            raise ValueError("Candidate view requires an ID")
        pose = rigid_transform(self.T_part_camera).copy()
        pose.setflags(write=False)
        object.__setattr__(self, "T_part_camera", pose)


def load_viewpoints(path):
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if (document["schema_version"] != 1 or document["coordinate_frame"] != "part"
            or document["length_unit"] != "meter" or not document["source"]
            or document["approval_status"] != "prototype_not_approved"):
        raise ValueError("Expected prototype viewpoints in the part frame, in meters")
    candidates = tuple(CandidateView(value["view_id"], camera_look_at(value["eye_part_m"], value["target_part_m"]))
                       for value in document["views"])
    if len({view.view_id for view in candidates}) != len(candidates):
        raise ValueError("Candidate view IDs must be unique")
    weight = document["rotation_cost_m_per_rad"]
    if not np.isfinite(weight) or weight < 0:
        raise ValueError("Rotation cost weight must be finite and nonnegative")
    return candidates, weight, document


def motion_cost(current_pose, candidate_pose, rotation_cost_m_per_rad):
    translation = float(np.linalg.norm(current_pose[:3, 3] - candidate_pose[:3, 3]))
    relative = current_pose[:3, :3].T @ candidate_pose[:3, :3]
    angle = math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
    return {"translation_distance_m": translation, "rotation_distance_rad": angle,
            "motion_cost_m_equivalent": translation + rotation_cost_m_per_rad * angle}


def plan_next_view(scene, regions, camera, state, knowledge, spec_id, candidates, available_capabilities,
                   *, rotation_cost_m_per_rad=0.1, max_incidence_angle_deg=75.0, surface_tolerance_m=1e-5,
                   candidate_scenes=None):
    spec = knowledge.specification(spec_id)
    expected = {region: frozenset(ids) for region, ids in spec.required_points(regions).items()}
    if (state.part_geometry_version != scene.geometry_version or state.inspection_spec_version != spec.version
            or state.required != expected):
        raise ValueError("Part/specification context changed; reset observation state before planning")
    if any(regions[key].region_id != key for key in spec.required_region_ids):
        raise ValueError("Surface region IDs do not match region keys")
    if not np.isfinite(rotation_cost_m_per_rad) or rotation_cost_m_per_rad < 0:
        raise ValueError("Rotation cost weight must be finite and nonnegative")
    candidates = tuple(candidates)
    if len({view.view_id for view in candidates}) != len(candidates):
        raise ValueError("Candidate view IDs must be unique")
    if candidate_scenes is not None:
        if set(candidate_scenes) != {view.view_id for view in candidates}:
            raise ValueError("Prediction scenes must identify every candidate exactly once")
        for predicted in candidate_scenes.values():
            if (predicted.geometry_version != scene.geometry_version
                    or not np.allclose(predicted.T_world_part, scene.T_world_part, rtol=0, atol=1e-9)):
                raise ValueError("Candidate prediction changed part geometry or placement")
    missing = state.missing_evidence()
    cause_counts = {}
    for points in missing.values():
        for evidence in points.values():
            cause = evidence["reason"]
            cause_counts[cause] = cause_counts.get(cause, 0) + 1
    result = {
        "status": "blocked", "reason": None, "action": None,
        "inspection_spec_id": spec.spec_id, "inspection_spec_version": spec.version,
        "required_region_ids": list(spec.required_region_ids), "part_geometry_version": scene.geometry_version,
        "knowledge": knowledge.provenance(), "available_capability_ids": sorted(set(available_capabilities)),
        "observation_state": state.summary(), "missing_evidence": missing,
        "missing_reason_counts": dict(sorted(cause_counts.items())), "method_lookup": [], "candidate_evaluations": [],
        "prediction_source": "predicted_from_simulator_geometry", "execution_status": "not_executed",
        "defect_decision": "not_evaluated",
        "visibility_parameters": {"max_incidence_angle_deg": max_incidence_angle_deg,
                                  "surface_tolerance_m": surface_tolerance_m},
        "coordinate_conventions": {"length_unit": "meter", "camera_optical_axes": "+X right, +Y up, -Z forward",
                                   "transform_equation": "T_world_camera = T_world_part @ T_part_camera",
                                   "matrix_convention": "row-major serialization, column vectors"},
        "ranking": {"order": ["new_required_point_count_desc", "motion_cost_asc", "view_id_asc"],
                    "cost_rounding_decimals": 12, "rotation_cost_m_per_rad": rotation_cost_m_per_rad},
    }
    if candidate_scenes is not None:
        result["prediction_geometry_policy"] = "candidate_specific_scene; predictions never update observation state"
    if state.summary()["inspection_observation_satisfied"]:
        result.update(status="inspection_observation_satisfied", reason="required_samples_already_observed")
        return result
    if not state.evaluated_capture_ids:
        result["reason"] = "observation_evidence_required"
        return result
    methods = knowledge.lookup_methods(cause_counts, result["available_capability_ids"])
    result["method_lookup"] = methods
    if not methods:
        result["reason"] = "no_method_for_missing_observation_causes"
        return result
    executable = [method for method in methods if not method["missing_capability_ids"]]
    if not executable:
        result["reason"] = "required_capability_unavailable"
        return result
    if not any(method["method_id"] == "change_viewpoint" for method in executable):
        result["reason"] = "method_not_implemented"
        return result
    if not candidates:
        result["reason"] = "no_candidate_views"
        return result
    for candidate in sorted(candidates, key=lambda view: view.view_id):
        world_pose = scene.T_world_part @ candidate.T_part_camera
        predicted_camera = replace(camera, T_world_camera=world_pose)
        predicted_scene = scene if candidate_scenes is None else candidate_scenes[candidate.view_id]
        report = evaluate_visibility(predicted_scene, regions, predicted_camera, required_region_ids=spec.required_region_ids,
                                     inspection_spec_version=spec.version,
                                     max_incidence_angle_deg=max_incidence_angle_deg,
                                     surface_tolerance_m=surface_tolerance_m)
        new_ids = {region: sorted(point["point_id"] for point in report["regions"][region]["points"]
                                  if point["visible"] and point["point_id"] in missing[region])
                   for region in spec.required_region_ids}
        result["candidate_evaluations"].append({
            "view_id": candidate.view_id, "T_part_camera": candidate.T_part_camera.tolist(),
            "T_world_camera": world_pose.tolist(), "predicted_new_point_ids": new_ids,
            "predicted_new_point_count": sum(len(ids) for ids in new_ids.values()),
            **motion_cost(camera.T_world_camera, world_pose, rotation_cost_m_per_rad)})
    best = min(result["candidate_evaluations"], key=lambda value: (
        -value["predicted_new_point_count"], round(value["motion_cost_m_equivalent"], 12), value["view_id"]))
    if best["predicted_new_point_count"] <= 0:
        result["reason"] = "no_candidate_improves_required_observation"
        return result
    result.update(status="ready", reason="eligible_method_and_capability_matched")
    result["action"] = {"method_id": "change_viewpoint", "execution_status": "not_executed", **best}
    return result
