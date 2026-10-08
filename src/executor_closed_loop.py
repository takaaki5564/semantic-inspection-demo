"""Bounded camera-pose planning with measured capture scenes; no Isaac Sim imports."""

import hashlib
import json

from arm_kinematics import pose_error
from arm_inspection import observe_capture, require_static_scene
from camera_executors import camera_error_ok
from closed_loop import require_pose, require_same_scene
from inspection_state import ObservationState
from viewpoint_planner import plan_next_view


def require_context(initial, current):
    require_static_scene(initial, current)
    if {k:v.to_dict() for k,v in initial[1].items()} != {k:v.to_dict() for k,v in current[1].items()}:
        raise ValueError("Surface samples changed; reset the inspection session")
    optics = [snap[2].to_dict() for snap in (initial, current)]
    for item in optics:
        item.pop("T_world_camera")
    if optics[0] != optics[1]:
        raise ValueError("Camera optics changed; reset the inspection session")


def run_camera_session(adapter, executor, knowledge, spec_id, candidates, initial_view, *,
                       session_id="inspection", max_actions=3, available_capabilities=("camera_pose_change",),
                       rotation_cost_m_per_rad=0.1, on_event=None, on_capture=None, on_prediction=None,
                       on_motion=lambda event: None):
    if not isinstance(max_actions, int) or isinstance(max_actions, bool) or max_actions < 0:
        raise ValueError("max_actions must be a nonnegative integer")
    candidates = tuple(candidates)
    if len({v.view_id for v in candidates}) != len(candidates):
        raise ValueError("Candidate IDs must be unique")
    capabilities = tuple(available_capabilities)
    spec = knowledge.specification(spec_id)
    initial = current = adapter.snapshot()
    state = ObservationState(initial[0].geometry_version, spec.version, spec.required_points(initial[1]))
    events, movements, captures, selected = [], [], [], []
    attempted = {initial_view.view_id}
    last_clock, last_rgb = None, None
    actions = 0

    def emit(kind, **data):
        event = json.loads(json.dumps({"sequence":len(events), "session_id":session_id, "event_type":kind, **data}, allow_nan=False))
        events.append(event)
        if on_event:
            on_event(json.loads(json.dumps(event)))

    def stop(status, reason):
        emit("session_stopped", status=status, reason=reason, actions_executed=actions, final_state=state.summary())
        return {"session_id":session_id, "status":status, "stop_reason":reason, "actions_executed":actions,
                "initial_positioning_count":sum(m["phase"] == "initial" for m in movements),
                "max_actions":max_actions, "inspection_spec_id":spec_id, "inspection_spec_version":spec.version,
                "part_geometry_version":state.part_geometry_version, "final_state":state.summary(),
                "movements":movements, "captures":captures, "selected_view_ids":selected, "events":events}

    def move(view, phase):
        nonlocal current
        emit("movement_started", phase=phase, view_id=view.view_id, T_part_camera_target=view.T_part_camera.tolist())
        result = executor.move_to_part_pose(view.T_part_camera, goal_id=view.view_id, emit=on_motion)
        movements.append({"phase":phase, **result})
        if result["status"] != "reached":
            emit("movement_blocked", phase=phase, view_id=view.view_id, reason=result["reason"])
            return result["reason"]
        after = adapter.snapshot()
        require_context(initial, after)
        error = pose_error(after[2].T_world_camera, after[0].T_world_part@view.T_part_camera)
        if not camera_error_ok(error):
            return "measured_camera_pose_error"
        current = after
        emit("movement_completed", phase=phase, view_id=view.view_id, camera_pose_error=error,
             T_world_camera_actual=current[2].T_world_camera.tolist())
        return None

    def observe(view_id):
        nonlocal current, last_clock, last_rgb
        before = adapter.snapshot()
        require_context(initial, before)
        capture_id = f"capture_{len(captures):02d}"
        image = f"images/{capture_id}.png"
        emit("capture_started", capture_id=capture_id, view_id=view_id)
        rgb, capture = executor.capture(image)
        after = adapter.snapshot()
        capture["rgb_array_sha256"] = hashlib.sha256(rgb.tobytes()).hexdigest()
        previous = state.summary()
        visibility, clock = observe_capture(before, after, rgb, capture, state, spec, capture_id,
                                             previous_reference=last_clock, previous_rgb=last_rgb)
        missing_before = {k:set(v["missing_point_ids"]) for k,v in previous["regions"].items()}
        new_ids = {k:sorted(missing_before[k]-set(v["missing_point_ids"])) for k,v in state.summary()["regions"].items()}
        record = {"capture_id":capture_id, "view_id":view_id, "capture":capture, "visibility":visibility,
                  "state_before_capture":previous, "state_after_visibility":state.summary(),
                  "actual_new_point_ids":new_ids, "actual_new_point_count":sum(map(len,new_ids.values()))}
        if on_capture:
            on_capture(rgb, record, after)
        captures.append(record)
        emit("observation_applied", **record)
        current, last_clock, last_rgb = after, clock, rgb.copy()
        return record["actual_new_point_count"]

    try:
        emit("session_started", inspection_spec_id=spec_id, required_region_ids=list(spec.required_region_ids),
             available_capability_ids=list(capabilities), max_actions=max_actions, initial_view_id=initial_view.view_id,
             state=state.summary())
        eligible, screening, _ = adapter.prepare_candidates((initial_view,), current)
        emit("initial_view_screened", screening=screening)
        if not eligible:
            return stop("blocked", "initial_view_infeasible")
        reason = move(initial_view, "initial")
        if reason:
            return stop("blocked", reason)
        observe(initial_view.view_id)
        while True:
            require_context(initial, current)
            # Eligibility and satisfied-state checks do not need robot IK/prediction.
            gate = plan_next_view(*current[:3], state, knowledge, spec_id, (), capabilities,
                                  rotation_cost_m_per_rad=rotation_cost_m_per_rad)
            if gate["status"] == "inspection_observation_satisfied" or gate["reason"] != "no_candidate_views":
                emit("plan_computed", plan=gate)
                return stop(gate["status"], gate["reason"])
            if actions >= max_actions:
                return stop("blocked", "max_actions_reached")
            available = tuple(view for view in candidates if view.view_id not in attempted)
            if not available:
                emit("plan_computed", plan=gate)
                return stop("blocked", "no_candidate_views")
            eligible, screening, scenes = adapter.prepare_candidates(available, current)
            emit("candidates_screened", screening=screening, eligible_view_ids=[v.view_id for v in eligible])
            if not eligible:
                return stop("blocked", "no_reachable_candidate_views")
            if on_prediction:
                for view in eligible:
                    on_prediction(actions, view, scenes[view.view_id], screening, current)
            plan = plan_next_view(*current[:3], state, knowledge, spec_id, eligible, capabilities,
                                  rotation_cost_m_per_rad=rotation_cost_m_per_rad, candidate_scenes=scenes)
            emit("plan_computed", plan=plan)
            if plan["status"] != "ready":
                return stop(plan["status"], plan["reason"])
            action = plan["action"]
            view = next(v for v in eligible if v.view_id == action["view_id"])
            # Planning is read-only, and the captured scene must still be the current scene.
            latest = adapter.snapshot()
            require_same_scene(current[:3], latest[:3])
            require_pose(current[2].T_world_camera, latest[2].T_world_camera, "Camera during planning")
            require_context(initial, latest)
            attempted.add(view.view_id)
            reason = move(view, "planned")
            if reason:
                return stop("blocked", reason)
            actions += 1
            selected.append(view.view_id)
            if observe(view.view_id) == 0:
                return stop("blocked", "executed_view_did_not_improve_observation")
    except Exception as error:
        emit("session_failed", reason=str(error), state=state.summary())
        raise
