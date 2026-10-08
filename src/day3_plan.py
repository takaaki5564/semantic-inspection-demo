"""Compare specifications and planning using saved Day 2 evidence; no simulator launch."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from capture_metadata import CONVENTIONS
from inspection_knowledge import InspectionKnowledge
from inspection_state import ObservationState
from part_geometry import SceneGeometry, SurfaceRegion
from verify_day2 import verify
from viewpoint_planner import CandidateView, load_viewpoints, plan_next_view
from visibility import CameraModel, evaluate_visibility


ROOT = Path(__file__).resolve().parents[1]


def evaluate_case(source_case, spec_id, knowledge, candidates, capabilities, rotation_weight):
    scene = SceneGeometry.from_dict(source_case["scene"])
    regions = {key: SurfaceRegion(**value) for key, value in source_case["surface_regions"].items()}
    camera = CameraModel(**source_case["camera"])
    spec = knowledge.specification(spec_id)
    # Every part/spec context starts fresh; re-evaluate the existing initial capture only.
    state = ObservationState(scene.geometry_version, spec.version, spec.required_points(regions))
    before_capture = state.summary()
    state.record_capture(source_case["case_id"])
    after_capture = state.summary()
    tolerances = {key: source_case["visibility"][key]
                  for key in ("max_incidence_angle_deg", "surface_tolerance_m")}
    baseline = evaluate_visibility(scene, regions, camera, required_region_ids=spec.required_region_ids,
                                   inspection_spec_version=spec.version, **tolerances)
    state.apply_visibility(source_case["case_id"], baseline)
    before_planning, missing_before = state.summary(), state.missing_evidence()
    plan = plan_next_view(scene, regions, camera, state, knowledge, spec_id, candidates, capabilities,
                          rotation_cost_m_per_rad=rotation_weight, **tolerances)
    after_planning = state.summary()
    if before_planning != after_planning or missing_before != state.missing_evidence():
        raise ValueError("Planning mutated actual observation evidence")
    return {"source_case_id": source_case["case_id"], "source_capture": source_case["capture"],
            "state_before_capture": before_capture, "state_after_capture": after_capture,
            "baseline_visibility": baseline, "state_before_planning": before_planning,
            "plan": plan, "state_after_planning": after_planning}


def run_comparison(document, knowledge, candidates, rotation_weight):
    initial = {case["case_id"]: case for case in document["cases"] if case["case_id"] in ("A_initial", "B_initial")}
    if set(initial) != {"A_initial", "B_initial"}:
        raise ValueError("Day 3 comparison requires the Day 2 A/B initial captures")
    results = {}
    for part in ("A", "B"):
        for spec_id in ("X", "Y"):
            results[f"{part}_{spec_id}"] = evaluate_case(initial[f"{part}_initial"], spec_id, knowledge,
                                                        candidates, {"camera_pose_change"}, rotation_weight)
    b = initial["B_initial"]
    results["B_Y_motion_disabled"] = evaluate_case(b, "Y", knowledge, candidates, set(), rotation_weight)
    current_pose = np.linalg.inv(np.asarray(b["scene"]["T_world_part"])) @ np.asarray(b["camera"]["T_world_camera"])
    results["B_Y_current_view_only"] = evaluate_case(b, "Y", knowledge, (CandidateView("current_view", current_pose),),
                                                    {"camera_pose_change"}, rotation_weight)
    plans = {key: case["plan"] for key, case in results.items()}
    checks = {
        "spec_X_requires_only_R1": plans["B_X"]["required_region_ids"] == ["R1"],
        "spec_Y_requires_R1_and_R2": plans["B_Y"]["required_region_ids"] == ["R1", "R2"],
        "A_needs_no_additional_view": all(plans[f"A_{spec}"]["status"] == "inspection_observation_satisfied"
                                           for spec in ("X", "Y")),
        "B_X_needs_no_additional_view": plans["B_X"]["status"] == "inspection_observation_satisfied",
        "B_Y_selects_improving_view": plans["B_Y"]["status"] == "ready" and plans["B_Y"]["action"]["predicted_new_point_count"] > 0,
        "no_camera_motion_is_blocked": plans["B_Y_motion_disabled"]["status"] == "blocked"
                                      and plans["B_Y_motion_disabled"]["reason"] == "required_capability_unavailable",
        "no_improving_view_is_blocked": plans["B_Y_current_view_only"]["status"] == "blocked"
                                       and plans["B_Y_current_view_only"]["reason"] == "no_candidate_improves_required_observation",
        "part_and_spec_start_with_empty_observations": all(case["state_before_capture"]["status"] == "not_captured"
                                                            for case in results.values()),
        "planning_does_not_change_observations": all(case["state_before_planning"] == case["state_after_planning"]
                                                      for case in results.values()),
        "no_predicted_motion_claimed_as_execution": all(plan["execution_status"] == "not_executed" for plan in plans.values()),
        "no_defect_decision_claimed": all(plan["defect_decision"] == "not_evaluated" for plan in plans.values()),
    }
    if not all(checks.values()):
        raise ValueError(f"Day 3 acceptance failed: {checks}")
    return {"cases": results, "acceptance_checks": checks}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True, help="Existing verified Day 2 output directory")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/day3_run_01")
    parser.add_argument("--knowledge", type=Path, default=ROOT / "knowledge/inspection_knowledge.json")
    parser.add_argument("--viewpoints", type=Path, default=ROOT / "config/camera_views.json")
    args = parser.parse_args(argv)
    try:
        if args.output_dir.exists():
            raise FileExistsError(f"Output directory already exists: {args.output_dir}; choose a new directory")
        verify(args.input_dir)
        input_path = (args.input_dir / "visibility_report.json").resolve()
        input_bytes = input_path.read_bytes()
        knowledge = InspectionKnowledge.load(args.knowledge)
        candidates, weight, view_document = load_viewpoints(args.viewpoints)
        comparison = run_comparison(json.loads(input_bytes), knowledge, candidates, weight)
        report = {"schema_version": 1, "mode": "offline_planning_from_saved_simulator_evidence",
                  "execution_status": "not_executed", "approval_status": "prototype_not_approved",
                  "defect_decision": "not_evaluated", "coordinate_conventions": CONVENTIONS,
                  "input_evidence": {"path": str(input_path), "sha256": hashlib.sha256(input_bytes).hexdigest(),
                                     "day2_replay_verification": "passed", "additional_captures": 0},
                  "knowledge_snapshot": knowledge.document, "viewpoint_configuration": view_document, **comparison}
        output_dir = args.output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=False)
        (output_dir / "plan_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        for case_id, case in comparison["cases"].items():
            plan = case["plan"]
            action = plan["action"]
            detail = f"view={action['view_id']}, predicted new points={action['predicted_new_point_count']}" if action else plan["reason"]
            print(f"{case_id}: {plan['status']} | {detail}")
        print(f"DAY3 COMPLETE: {sum(comparison['acceptance_checks'].values())} checks PASS | {output_dir / 'plan_report.json'}")
        print("Offline prediction only; camera not moved, no new images, observations unchanged.")
        return 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"DAY3 FAILED: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
