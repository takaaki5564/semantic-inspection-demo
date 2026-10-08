"""Replay executed Day 4 steps from saved images, geometry and event history, without Isaac."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

from capture_metadata import camera_look_at
from closed_loop import require_pose, run_session
from inspection_knowledge import InspectionKnowledge
from part_geometry import SceneGeometry, SurfaceRegion
from viewpoint_planner import CandidateView
from visibility import CameraModel


class ReplayAdapter:
    def __init__(self, captures, output_dir):
        self.captures, self.output_dir = captures, Path(output_dir).resolve()
        self.current, self.next_capture = 0, 0

    def geometry_snapshot(self):
        event = self.captures[self.current]
        return SceneGeometry.from_dict(event["scene"]), {
            key: SurfaceRegion(**value) for key, value in event["surface_regions"].items()}

    def camera_model(self):
        return CameraModel(**self.captures[self.current]["camera"])

    def capture(self, image):
        if self.current != self.next_capture:
            raise ValueError("Unexpected replay capture ordering")
        capture = self.captures[self.current]["capture"]
        if image != capture["image"]:
            raise ValueError("Capture filename does not match event history")
        path = (self.output_dir / image).resolve()
        if not path.is_relative_to(self.output_dir):
            raise ValueError("Capture must be inside the run directory")
        with Image.open(path) as saved:
            if saved.mode != "RGB":
                raise ValueError("Expected RGB PNG")
            rgb = np.asarray(saved).copy()
        self.next_capture += 1
        return rgb, capture

    def move_to_part_pose(self, pose):
        if self.next_capture >= len(self.captures):
            raise ValueError("Executed action has no subsequent capture evidence")
        self.current = self.next_capture
        scene, _ = self.geometry_snapshot()
        camera = self.camera_model()
        require_pose(np.linalg.inv(scene.T_world_part) @ camera.T_world_camera, pose, "Replayed executed camera")


def verify_document(document, output_dir):
    if (document["run_status"] != "completed" or document["demo_step"] != "day4_closed_loop"
            or document["approval_status"] != "prototype_not_approved"
            or document["defect_decision"] != "not_evaluated"):
        raise ValueError("Run is incomplete or makes an unexpected approval/defect claim")
    session = document["session"]
    output_dir = Path(output_dir).resolve()
    with (output_dir / "events.jsonl").open(encoding="utf-8") as stream:
        stored_events = [json.loads(line) for line in stream if line.strip()]
    if stored_events != session["events"]:
        raise ValueError("JSONL event history differs from report")
    captures = [event for event in session["events"] if event["event_type"] == "capture_completed"]
    if not captures:
        raise ValueError("No actual capture evidence")
    configuration = document["viewpoint_configuration"]
    if (configuration["coordinate_frame"] != "part" or configuration["length_unit"] != "meter"
            or configuration["approval_status"] != "prototype_not_approved"):
        raise ValueError("Expected prototype viewpoints in the part frame, in meters")
    candidates = tuple(CandidateView(value["view_id"], camera_look_at(value["eye_part_m"], value["target_part_m"]))
                       for value in configuration["views"])
    replay = run_session(ReplayAdapter(captures, output_dir), InspectionKnowledge(document["knowledge_snapshot"]),
                         session["inspection_spec_id"], candidates, session_id=session["session_id"],
                         max_actions=session["max_actions"], available_capabilities=document["available_capability_ids"],
                         rotation_cost_m_per_rad=configuration["rotation_cost_m_per_rad"])
    if replay != session:
        raise ValueError("Planning, executed poses, RGB hashes, visibility or observation state did not replay")
    actions = [event for event in session["events"] if event["event_type"] == "action_completed"]
    observations = [event for event in session["events"] if event["event_type"] == "observation_applied"]
    satisfied = session["final_state"]["inspection_observation_satisfied"]
    checks = {
        "event_history_and_state_replayed": True,
        "capture_count_matches_executed_actions": len(captures) == len(actions) + 1 == len(observations),
        "action_count_matches_pose_receipts": len(actions) == session["actions_executed"],
        "max_actions_respected": len(actions) <= session["max_actions"],
        "success_requires_actual_observation": (session["status"] == "inspection_observation_satisfied") == satisfied,
        "blocked_is_explicit": session["status"] != "blocked" or bool(session["stop_reason"]),
        "every_action_improves_or_stops_blocked": all(event["actual_new_point_count"] > 0 for event in observations[1:])
            or (session["status"] == "blocked" and session["stop_reason"] == "executed_view_did_not_improve_observation"),
        "defect_decision_not_evaluated": session["final_state"]["defect_decision"] == "not_evaluated",
    }
    if not all(checks.values()) or ("acceptance_checks" in document and checks != document["acceptance_checks"]):
        raise ValueError(f"Day 4 acceptance failed: {checks}")
    return checks


def verify(output_dir):
    output_dir = Path(output_dir).resolve()
    return verify_document(json.loads((output_dir / "closed_loop_report.json").read_text(encoding="utf-8")), output_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, nargs="?", default=Path(__file__).resolve().parents[1] / "outputs/day4_run_01")
    args = parser.parse_args()
    try:
        checks = verify(args.output_dir)
        print(f"PASS: Day 4 planning, executed poses, fresh RGB, geometry, visibility and state replay | {checks}")
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
