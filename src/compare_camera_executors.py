"""Replay saved RGB/geometry evidence, then compare free and arm camera runs offline."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

from arm_inspection import observe_capture
from arm_kinematics import pose_error
from arm_inspection_geometry import scene_fingerprint
from arm_inspection_check import ROOT, write_json
from capture_metadata import rigid_transform
from camera_executors import camera_error_ok
from inspection_knowledge import InspectionKnowledge
from inspection_state import ObservationState
from part_geometry import SceneGeometry, SurfaceRegion
from viewpoint_planner import CandidateView, plan_next_view
from visibility import CameraModel
from render_camera_evidence import validate_render_camera


def read_scene(path):
    data = json.loads(path.read_bytes())
    return data, SceneGeometry.from_dict(data["scene"]), {k:SurfaceRegion(**v) for k,v in data["surface_regions"].items()}, CameraModel(**data["camera"])


def verify_run(path):
    path = Path(path).resolve()
    report = json.loads(path.read_bytes())
    if report.get("demo_step") != "camera_executor_closed_loop" or report.get("run_status") not in ("passed","blocked"):
        raise ValueError("Expected a completed or explicitly blocked camera loop report")
    context, session = report["comparison_context"], report["session"]
    if len(session["captures"]) != 1+session["actions_executed"]:
        raise ValueError("Capture count disagrees with executed additional actions")
    knowledge = InspectionKnowledge(context["knowledge_snapshot"])
    spec = knowledge.specification(context["inspection_spec_id"])
    state, last_clock, last_rgb = None, None, None
    snapshots = {}
    goals = {v["view_id"]:v["T_part_camera"] for v in context["candidate_views"]}
    if "T_part_camera_initial" in context:
        goals[context["initial_view_id"]] = context["T_part_camera_initial"]
    expected_views = [context["initial_view_id"],*session["selected_view_ids"]]
    for index,record in enumerate(session["captures"]):
        data, scene, regions, camera = read_scene(path.parent/record["scene_file"])
        metadata = data["snapshot"]
        if (scene_fingerprint(scene) != metadata["full_scene_sha256"]
                or scene_fingerprint(scene, exclude_robot=True) != context["static_scene_sha256"]
                or scene.geometry_version != context["part_geometry_version"]
                or not np.allclose(scene.T_world_part, rigid_transform(context["T_world_part"]), rtol=0,atol=1e-9)):
            raise ValueError("Saved geometry hash or inspection context mismatch")
        if state is None:
            state = ObservationState(scene.geometry_version,spec.version,spec.required_points(regions))
        capture = record["capture"]
        if validate_render_camera(capture["renderer_camera"]["parameters"], camera) != capture["renderer_camera"]:
            raise ValueError("Saved renderer camera evidence mismatch")
        if (record["view_id"] != expected_views[index]
                or not camera_error_ok(pose_error(camera.T_world_camera,scene.T_world_part@rigid_transform(goals[record["view_id"]])))):
            raise ValueError("Captured camera does not match the planned camera goal")
        rgb = np.asarray(Image.open(path.parent/capture["image"]).convert("RGB"))
        if hashlib.sha256(rgb.tobytes()).hexdigest() != capture["rgb_array_sha256"]:
            raise ValueError("Saved PNG pixels differ from captured RGB evidence")
        snapshot = (scene,regions,camera,metadata)
        if not capture["physics_held_during_capture"] or state.summary() != record["state_before_capture"]:
            raise ValueError("Capture ordering or held-physics evidence mismatch")
        visibility, clock = observe_capture(snapshot,snapshot,rgb,capture,state,spec,record["capture_id"],
                                             previous_reference=last_clock,previous_rgb=last_rgb)
        if visibility != record["visibility"] or state.summary() != record["state_after_visibility"]:
            raise ValueError("Replayed actual visibility/state differs from saved report")
        snapshots[record["capture_id"]] = snapshot
        last_clock, last_rgb = clock, rgb
    if state is None:
        # Initial IK failure is a valid blocked run, but cannot be compared as an inspected initial view.
        raise ValueError("Comparison requires at least the initial captured view")
    replay = ObservationState(state.part_geometry_version,state.inspection_spec_version,state.required)
    current, screening = None, None
    planned_views = []
    planned_count = 0
    for event in session["events"]:
        if event["event_type"] == "observation_applied":
            capture_id = event["capture_id"]
            replay.record_capture(capture_id)
            replay.apply_visibility(capture_id,event["visibility"])
            current = snapshots[capture_id]
        elif event["event_type"] == "candidates_screened":
            screening = event
        elif event["event_type"] == "plan_computed":
            saved = event["plan"]
            if saved["candidate_evaluations"]:
                ids = [v["view_id"] for v in saved["candidate_evaluations"]]
                available = {v["view_id"]:CandidateView(v["view_id"],v["T_part_camera"]) for v in context["candidate_views"]}
                views, scenes = tuple(available[name] for name in ids), {}
                if screening is None or set(screening["eligible_view_ids"]) != set(ids):
                    raise ValueError("Plan candidates do not match preselection reachability screening")
                for name in ids:
                    entry = next(p for p in report["predictions"] if p["action_index"] == planned_count and p["view_id"] == name)
                    data, scene, _, _ = read_scene(path.parent/entry["scene_file"])
                    if scene_fingerprint(scene) != entry["scene_sha256"]:
                        raise ValueError("Prediction geometry hash mismatch")
                    scenes[name] = scene
                actual_plan = plan_next_view(*current[:3],replay,knowledge,spec.spec_id,views,context["available_capability_ids"],
                                             rotation_cost_m_per_rad=context["rotation_cost_m_per_rad"],candidate_scenes=scenes)
                if actual_plan != saved:
                    raise ValueError("Replayed candidate-specific plan differs from saved decision")
                if saved["action"]:
                    planned_views.append(saved["action"]["view_id"])
                    planned_count += 1
            else:
                actual_plan = plan_next_view(*current[:3],replay,knowledge,spec.spec_id,(),context["available_capability_ids"],
                                             rotation_cost_m_per_rad=context["rotation_cost_m_per_rad"])
                if actual_plan != saved:
                    raise ValueError("Replayed stopping decision differs from saved decision")
    if replay.summary() != session["final_state"] or state.summary() != session["final_state"]:
        raise ValueError("Final state differs from replayed actual captures")
    if session["actions_executed"] != len(session["selected_view_ids"]) or session["selected_view_ids"] != planned_views[:session["actions_executed"]]:
        raise ValueError("Executed views differ from the planned view sequence")
    satisfied = state.summary()["inspection_observation_satisfied"]
    if (report["run_status"] == "passed") != satisfied or (session["status"] == "inspection_observation_satisfied") != satisfied:
        raise ValueError("Run success status disagrees with actual observations")
    initial = session["captures"][0]["state_after_visibility"]["regions"]
    final = session["final_state"]["regions"]
    summary = {"executor":report["executor"], "status":session["status"], "stop_reason":session["stop_reason"],
               "capture_count":len(session["captures"]), "additional_action_count":session["actions_executed"],
               "selected_view_ids":session["selected_view_ids"],
               "initial_required_observed_count":sum(v["observed_count"] for v in initial.values()),
               "final_required_observed_count":sum(v["observed_count"] for v in final.values()),
               "required_point_count":sum(v["total_count"] for v in final.values()),
               "regions":{k:{"initial_observed_count":initial[k]["observed_count"],"final_observed_count":v["observed_count"],
                             "total_count":v["total_count"]} for k,v in final.items()}}
    return report, summary


def compare_runs(arm_path,free_path):
    arm, arm_summary = verify_run(arm_path)
    free, free_summary = verify_run(free_path)
    if arm["executor"] != "arm" or free["executor"] != "free":
        raise ValueError("Provide one arm report and one free report")
    if (arm["comparison_context"] != free["comparison_context"] or arm["model_manifest"] != free["model_manifest"]
            or arm["runtime"]["isaacsim_package"] != free["runtime"]["isaacsim_package"]):
        raise ValueError("Part/spec/cell/optics/candidates/capability/budget/model differ; comparison rejected")
    return {"schema_version":1, "demo_step":"camera_executor_comparison", "evidence_source":"replayed_saved_simulator_RGB_and_geometry",
            "comparison_context":arm["comparison_context"], "arm":arm_summary, "free":free_summary,
            "sources":{"arm":str(Path(arm_path).resolve()),"free":str(Path(free_path).resolve())},
            "limitations":["robot moves in arm mode and remains parked in free mode; dynamic geometry and shadows can differ",
                           "same camera goals do not imply bit-identical RGB or exactly equal measured poses",
                           "visibility uses known simulator geometry; no defect accuracy or collision-safety claim"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-report",type=Path,required=True)
    parser.add_argument("--free-report",type=Path,required=True)
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/camera_comparison_01.json")
    args = parser.parse_args(argv)
    try:
        comparison = compare_runs(args.arm_report,args.free_report)
        write_json(args.output,comparison)
        for name in ("arm","free"):
            value = comparison[name]
            print(f"{name} | captures={value['capture_count']} | additional_actions={value['additional_action_count']} | "
                  f"observed={value['initial_required_observed_count']}->{value['final_required_observed_count']}/{value['required_point_count']} | {value['status']}")
        print(f"COMPARISON | {args.output.resolve()}")
        return 0
    except (OSError,ValueError,KeyError,TypeError,StopIteration) as error:
        print(f"COMPARISON FAILED: {error}",file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
