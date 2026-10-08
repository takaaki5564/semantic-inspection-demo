"""Incremental measured camera inspection; CLI and GUI share one controller."""

from copy import deepcopy
import hashlib
import json

from arm_kinematics import pose_error
from arm_inspection import observe_capture, require_static_scene
from camera_executors import camera_error_ok
from closed_loop import require_pose, require_same_scene
from inspection_region_state import load_region_metadata, region_snapshot
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


def frozen(value):
    return json.loads(json.dumps(value, allow_nan=False))


class CameraInspectionSession:
    """One step = one measured/saved observation, followed by its next plan.

    Calls are synchronous; an Isaac UI driver schedules them outside button
    callbacks. Stop requests take effect at an observation boundary. Reset means
    creating a new instance/session ID at that boundary, never reusing evidence.
    """

    def __init__(self, adapter, executor, knowledge, spec_id, candidates, initial_view, *,
                 session_id="inspection", max_actions=3, available_capabilities=("camera_pose_change",),
                 rotation_cost_m_per_rad=0.1, on_event=None, on_capture=None, on_prediction=None,
                 on_motion=lambda event: None, region_metadata=None):
        if not isinstance(max_actions, int) or isinstance(max_actions, bool) or max_actions < 0:
            raise ValueError("max_actions must be a nonnegative integer")
        self.candidates = deepcopy(tuple(candidates))
        if len({v.view_id for v in self.candidates}) != len(self.candidates):
            raise ValueError("Candidate IDs must be unique")
        self.adapter, self.executor = adapter, executor
        self.knowledge = deepcopy(knowledge)
        self.spec_id, self.spec = spec_id, self.knowledge.specification(spec_id)
        self.initial_view = deepcopy(initial_view)
        self.session_id, self.max_actions = session_id, max_actions
        self.capabilities = tuple(available_capabilities)
        self.weight = rotation_cost_m_per_rad
        self.on_event, self.on_capture = on_event, on_capture
        self.on_prediction, self.on_motion = on_prediction, on_motion
        self.initial = self.current = adapter.snapshot()
        self.state = ObservationState(self.initial[0].geometry_version, self.spec.version,
                                      self.spec.required_points(self.initial[1]))
        self.metadata = deepcopy(region_metadata if region_metadata is not None else load_region_metadata())
        region_snapshot(self.state, self.initial[1], self.metadata)
        self.events, self.movements, self.captures, self.selected = [], [], [], []
        self.attempted = {initial_view.view_id}
        self.last_clock, self.last_rgb = None, None
        self.actions, self.steps_completed = 0, 0
        self.status, self.reason = "initialized", None
        self.pending_plan, self.pending_view = None, None
        self._started, self._busy, self._stop_requested = False, False, False
        self.phase = "initialized"

    @property
    def done(self):
        return self.status not in ("initialized", "running", "paused")

    def snapshot(self):
        return frozen({"session_id":self.session_id, "status":self.status, "phase":self.phase,
                       "stop_reason":self.reason, "steps_completed":self.steps_completed,
                       "actions_executed":self.actions, "capture_count":len(self.captures),
                       "observation_state":self.state.summary(), "next_plan":self.pending_plan,
                       "region_states":region_snapshot(self.state, self.initial[1], self.metadata, self.pending_plan)})

    def report(self):
        return frozen({"session_id":self.session_id, "status":self.status, "stop_reason":self.reason,
                       "phase":self.phase, "steps_completed":self.steps_completed, "next_plan":self.pending_plan,
                       "actions_executed":self.actions,
                       "initial_positioning_count":sum(m["phase"] == "initial" for m in self.movements),
                       "max_actions":self.max_actions, "inspection_spec_id":self.spec_id,
                       "inspection_spec_version":self.spec.version, "part_geometry_version":self.state.part_geometry_version,
                       "final_state":self.state.summary(), "region_states":self.snapshot()["region_states"],
                       "movements":self.movements, "captures":self.captures,
                       "selected_view_ids":self.selected, "events":self.events})

    def _emit(self, kind, **data):
        event = frozen({"sequence":len(self.events), "session_id":self.session_id, "event_type":kind, **data})
        self.events.append(event)
        if self.on_event:
            self.on_event(frozen(event))

    def _stop(self, status, reason):
        self.status, self.reason, self.phase = status, reason, "stopped"
        self.pending_plan = self.pending_view = None
        self._emit("session_stopped", status=status, reason=reason, actions_executed=self.actions,
                   final_state=self.state.summary(), region_states=self.snapshot()["region_states"])

    def request_stop(self):
        """Safe cycle boundary stop; not a physical emergency stop."""
        self._stop_requested = True

    def _require_held_context(self):
        latest = self.adapter.snapshot()
        require_same_scene(self.current[:3], latest[:3])
        require_pose(self.current[2].T_world_camera, latest[2].T_world_camera, "Camera while session paused/planning")
        require_context(self.initial, latest)

    def _move(self, view, phase):
        self.phase = "moving"
        self._emit("movement_started", phase=phase, view_id=view.view_id, T_part_camera_target=view.T_part_camera.tolist())
        result = self.executor.move_to_part_pose(view.T_part_camera, goal_id=view.view_id, emit=self.on_motion)
        self.movements.append({"phase":phase, **result})
        if result["status"] != "reached":
            self._emit("movement_blocked", phase=phase, view_id=view.view_id, reason=result["reason"])
            return result["reason"]
        after = self.adapter.snapshot()
        require_context(self.initial, after)
        error = pose_error(after[2].T_world_camera, after[0].T_world_part@view.T_part_camera)
        if not camera_error_ok(error):
            return "measured_camera_pose_error"
        self.current = after
        self._emit("movement_completed", phase=phase, view_id=view.view_id, camera_pose_error=error,
                   T_world_camera_actual=after[2].T_world_camera.tolist())
        return None

    def _observe(self, view_id):
        before = self.adapter.snapshot()
        require_context(self.initial, before)
        capture_id = f"capture_{len(self.captures):02d}"
        self.phase = "capturing"
        self._emit("capture_started", capture_id=capture_id, view_id=view_id)
        rgb, capture = self.executor.capture(f"images/{capture_id}.png")
        after = self.adapter.snapshot()
        capture["rgb_array_sha256"] = hashlib.sha256(rgb.tobytes()).hexdigest()
        previous = self.state.summary()
        # Save/evaluate transaction: failures cannot commit coverage or freshness.
        staged = deepcopy(self.state)
        visibility, clock = observe_capture(before, after, rgb, capture, staged, self.spec, capture_id,
                                             previous_reference=self.last_clock, previous_rgb=self.last_rgb)
        missing_before = {k:set(v["missing_point_ids"]) for k,v in previous["regions"].items()}
        new_ids = {k:sorted(missing_before[k]-set(v["missing_point_ids"])) for k,v in staged.summary()["regions"].items()}
        record = {"capture_id":capture_id, "view_id":view_id, "capture":capture, "visibility":visibility,
                  "state_before_capture":previous, "state_after_visibility":staged.summary(),
                  "actual_new_point_ids":new_ids, "actual_new_point_count":sum(map(len,new_ids.values()))}
        self.phase = "capture_validated"
        self._emit("capture_validated", capture_id=capture_id, view_id=view_id)
        if self.on_capture:
            self.phase = "saving"
            self.on_capture(rgb, record, after)
            self.phase = "saved"
            self._emit("capture_saved", capture_id=capture_id, image=capture["image"])
        self.state = staged
        self.captures.append(frozen(record))
        self.current, self.last_clock, self.last_rgb = after, clock, rgb.copy()
        self.phase = "observed"
        self._emit("observation_applied", **record)
        return record["actual_new_point_count"]

    def _plan(self):
        self.phase = "planning"
        require_context(self.initial, self.current)
        gate = plan_next_view(*self.current[:3], self.state, self.knowledge, self.spec_id, (), self.capabilities,
                              rotation_cost_m_per_rad=self.weight)
        if gate["status"] == "inspection_observation_satisfied" or gate["reason"] != "no_candidate_views":
            self._emit("plan_computed", plan=gate)
            self._stop(gate["status"], gate["reason"])
            return
        if self.actions >= self.max_actions:
            self._stop("blocked", "max_actions_reached")
            return
        available = tuple(v for v in self.candidates if v.view_id not in self.attempted)
        if not available:
            self._emit("plan_computed", plan=gate)
            self._stop("blocked", "no_candidate_views")
            return
        eligible, screening, scenes = self.adapter.prepare_candidates(available, self.current)
        self._emit("candidates_screened", screening=screening, eligible_view_ids=[v.view_id for v in eligible])
        if not eligible:
            self._stop("blocked", "no_reachable_candidate_views")
            return
        if self.on_prediction:
            for view in eligible:
                self.on_prediction(self.actions, view, scenes[view.view_id], screening, self.current)
        plan = plan_next_view(*self.current[:3], self.state, self.knowledge, self.spec_id, eligible, self.capabilities,
                              rotation_cost_m_per_rad=self.weight, candidate_scenes=scenes)
        self._emit("plan_computed", plan=plan)
        if plan["status"] != "ready":
            self._stop(plan["status"], plan["reason"])
            return
        self._require_held_context()
        self.pending_plan = plan
        self.pending_view = next(v for v in eligible if v.view_id == plan["action"]["view_id"])
        self.status, self.phase = "paused", "awaiting_step"

    def step(self):
        if self._busy:
            raise RuntimeError("Inspection step already running")
        if self.done:
            return self.snapshot()
        self._busy = True
        try:
            self.status = "running"
            if not self._started:
                self._started = True
                self._emit("session_started", inspection_spec_id=self.spec_id,
                           required_region_ids=list(self.spec.required_region_ids),
                           available_capability_ids=list(self.capabilities), max_actions=self.max_actions,
                           initial_view_id=self.initial_view.view_id, state=self.state.summary())
            if self._stop_requested:
                self._stop("stopped", "stop_requested")
                return self.snapshot()
            self._require_held_context()
            if not self.captures:
                view, phase = self.initial_view, "initial"
                eligible, screening, _ = self.adapter.prepare_candidates((view,), self.current)
                self._emit("initial_view_screened", screening=screening)
                if not eligible:
                    self._stop("blocked", "initial_view_infeasible")
                    return self.snapshot()
            else:
                view, phase = self.pending_view, "planned"
                self.pending_plan = self.pending_view = None
                self.attempted.add(view.view_id)
            reason = self._move(view, phase)
            if reason:
                self._stop("blocked", reason)
                return self.snapshot()
            if phase == "planned":
                self.actions += 1
                self.selected.append(view.view_id)
            gain = self._observe(view.view_id)
            self.steps_completed += 1
            if phase == "planned" and gain == 0:
                self._stop("blocked", "executed_view_did_not_improve_observation")
            elif self._stop_requested:
                self._stop("stopped", "stop_requested")
            else:
                self._plan()
            self._emit("step_completed", snapshot=self.snapshot())
            return self.snapshot()
        except (Exception, KeyboardInterrupt) as error:
            failure_phase = self.phase
            reason = str(error) or type(error).__name__
            self.status, self.reason, self.phase = "failed", reason, "failed"
            self.pending_plan = self.pending_view = None
            self._emit("session_failed", reason=reason, failure_phase=failure_phase, state=self.state.summary(),
                       region_states=self.snapshot()["region_states"])
            raise
        finally:
            self._busy = False

    def run(self):
        while not self.done:
            self.step()
        return self.report()


def run_camera_session(adapter, executor, knowledge, spec_id, candidates, initial_view, **kwargs):
    """Existing continuous CLI entrypoint; uses exactly the same step implementation."""
    return CameraInspectionSession(adapter, executor, knowledge, spec_id, candidates, initial_view, **kwargs).run()
