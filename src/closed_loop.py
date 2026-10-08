"""Bounded plan/execute/observe loop. Core uses an adapter and never imports Isaac Sim."""

import hashlib
import json

import numpy as np

from capture_metadata import reference_time, rigid_transform
from inspection_state import ObservationState
from viewpoint_planner import plan_next_view
from visibility import evaluate_visibility


def require_pose(actual, expected, label):
    if not np.allclose(rigid_transform(actual), rigid_transform(expected), rtol=0, atol=1e-7):
        raise RuntimeError(f"{label} does not match expected pose")


def snapshot(adapter):
    scene, regions = adapter.geometry_snapshot()
    return scene, regions, adapter.camera_model()


def require_same_scene(before, after):
    old_scene, old_regions, _ = before
    new_scene, new_regions, _ = after
    if (old_scene.prim_paths != new_scene.prim_paths or old_scene.face_ids != new_scene.face_ids
            or old_scene.geometry_version != new_scene.geometry_version
            or old_scene.triangles_world_m.shape != new_scene.triangles_world_m.shape
            or not np.allclose(old_scene.triangles_world_m, new_scene.triangles_world_m, rtol=0, atol=1e-9)
            or {key: value.to_dict() for key, value in old_regions.items()}
            != {key: value.to_dict() for key, value in new_regions.items()}):
        raise RuntimeError("Scene or surface samples changed during execution/capture; start a new session")
    require_pose(new_scene.T_world_part, old_scene.T_world_part, "Part")


def validate_frame(rgb, capture, expected, actual, previous_reference, previous_rgb, expected_image):
    """Validate synchronized evidence before allowing it to affect observation state."""
    require_same_scene(expected, actual)
    scene, _, camera = actual
    require_pose(camera.T_world_camera, expected[2].T_world_camera, "Camera during capture")
    before_optics, after_optics = expected[2].to_dict(), camera.to_dict()
    before_optics.pop("T_world_camera")
    after_optics.pop("T_world_camera")
    if before_optics != after_optics:
        raise RuntimeError("Camera optics changed during capture")
    if capture["status"] != "captured" or capture["evidence_source"] != "simulator_rgb" or capture["image"] != expected_image:
        raise RuntimeError("Unexpected RGB capture evidence or image ID")
    require_pose(capture["T_world_camera"], camera.T_world_camera, "Captured camera")
    require_pose(capture["T_world_part"], scene.T_world_part, "Captured part")
    require_pose(scene.T_world_part @ rigid_transform(capture["T_part_camera"]), camera.T_world_camera, "Composed camera")
    clock = reference_time(capture["render_reference_time"])
    if previous_reference is not None and clock <= previous_reference:
        raise RuntimeError("Stale RGB frame: render reference time did not advance")
    if rgb.shape != (*camera.resolution_hw, 3) or rgb.dtype != np.uint8 or np.ptp(rgb) == 0:
        raise RuntimeError("Expected nonuniform uint8 RGB matching camera resolution")
    if previous_rgb is not None and np.array_equal(rgb, previous_rgb):
        raise RuntimeError("RGB unchanged after camera movement; fresh observation cannot be confirmed")
    return clock


def run_session(adapter, knowledge, spec_id, candidates, *, session_id="inspection", max_actions=3,
                available_capabilities=("camera_pose_change",), rotation_cost_m_per_rad=0.1,
                previous_reference=None, on_event=None, on_capture=None):
    if not isinstance(max_actions, int) or isinstance(max_actions, bool) or max_actions < 0:
        raise ValueError("max_actions must be a nonnegative integer")
    candidates, capabilities = tuple(candidates), tuple(available_capabilities)
    spec = knowledge.specification(spec_id)
    current = snapshot(adapter)
    state = ObservationState(current[0].geometry_version, spec.version, spec.required_points(current[1]))
    events, attempted, actions_executed = [], set(), 0
    last_clock, last_rgb = previous_reference, None

    def emit(event_type, **data):
        # Freeze the payload before giving it to storage/console callbacks.
        event = json.loads(json.dumps({"sequence": len(events), "session_id": session_id,
                                      "event_type": event_type, **data}, allow_nan=False))
        events.append(event)
        if on_event is not None:
            on_event(json.loads(json.dumps(event)))

    def stop(status, reason):
        emit("session_stopped", status=status, reason=reason, actions_executed=actions_executed,
             final_state=state.summary())
        return {"session_id": session_id, "inspection_spec_id": spec_id,
                "inspection_spec_version": spec.version, "part_geometry_version": state.part_geometry_version,
                "status": status, "stop_reason": reason, "actions_executed": actions_executed,
                "max_actions": max_actions, "final_state": state.summary(), "events": events}

    def observe(capture_index, action_id):
        nonlocal current, last_clock, last_rgb
        capture_id = f"{session_id}_capture_{capture_index:02d}"
        image = f"images/{capture_id}.png"
        emit("capture_started", capture_id=capture_id, action_id=action_id)
        rgb, capture = adapter.capture(image)
        actual = snapshot(adapter)
        clock = validate_frame(rgb, capture, current, actual, last_clock, last_rgb, image)
        capture = {**capture, "rgb_array_sha256": hashlib.sha256(rgb.tobytes()).hexdigest()}
        before_capture = state.summary()
        state.record_capture(capture_id)
        emit("capture_completed", capture_id=capture_id, action_id=action_id, capture=capture,
             scene=actual[0].to_dict(), surface_regions={key: value.to_dict() for key, value in actual[1].items()},
             camera=actual[2].to_dict(), state_before_capture=before_capture, state_after_capture=state.summary(),
             freshness={"render_reference_advanced": True if last_clock is not None else None,
                        "captured_pose_matches_readback": True,
                        "synchronized_capture": "Replicator step, rt_subframes=4, wait_for_render=True"})
        report = evaluate_visibility(actual[0], actual[1], actual[2], required_region_ids=spec.required_region_ids,
                                     inspection_spec_version=spec.version)
        missing_before = {key: set(value["missing_point_ids"]) for key, value in before_capture["regions"].items()}
        if on_capture is not None:
            on_capture(rgb, capture, report)
        state.apply_visibility(capture_id, report)
        new_ids = {key: sorted(missing_before[key] - set(value["missing_point_ids"]))
                   for key, value in state.summary()["regions"].items()}
        gain = sum(len(ids) for ids in new_ids.values())
        emit("observation_applied", capture_id=capture_id, visibility=report, state_after_visibility=state.summary(),
             actual_new_point_ids=new_ids, actual_new_point_count=gain)
        current, last_clock, last_rgb = actual, clock, rgb.copy()
        return gain

    try:
        emit("session_started", inspection_spec_id=spec_id, inspection_spec_version=spec.version,
             required_region_ids=list(spec.required_region_ids), max_actions=max_actions,
             available_capability_ids=sorted(set(capabilities)), state=state.summary())
        observe(0, None)
        while True:
            available_views = tuple(view for view in candidates if view.view_id not in attempted)
            plan = plan_next_view(*current, state, knowledge, spec_id, available_views, capabilities,
                                  rotation_cost_m_per_rad=rotation_cost_m_per_rad)
            emit("plan_computed", available_view_ids=[view.view_id for view in available_views], plan=plan)
            if plan["status"] == "inspection_observation_satisfied":
                return stop(plan["status"], plan["reason"])
            if plan["status"] == "blocked":
                return stop("blocked", plan["reason"])
            if actions_executed >= max_actions:
                return stop("blocked", "max_actions_reached")
            action = plan["action"]
            action_id = f"{session_id}_action_{actions_executed + 1:02d}"
            emit("action_started", action_id=action_id, action=action)
            adapter.move_to_part_pose(action["T_part_camera"])
            after_move = snapshot(adapter)
            require_same_scene(current, after_move)
            require_pose(after_move[2].T_world_camera, action["T_world_camera"], "Executed camera")
            actions_executed += 1
            attempted.add(action["view_id"])
            current = after_move
            emit("action_completed", action_id=action_id, view_id=action["view_id"],
                 execution_status="executed_pose_verified", T_world_part=current[0].T_world_part.tolist(),
                 T_world_camera=current[2].T_world_camera.tolist(),
                 T_part_camera=(np.linalg.inv(current[0].T_world_part) @ current[2].T_world_camera).tolist())
            if observe(actions_executed, action_id) == 0:
                return stop("blocked", "executed_view_did_not_improve_observation")
    except Exception as error:
        emit("session_failed", reason=str(error), actions_executed=actions_executed, state=state.summary())
        raise
