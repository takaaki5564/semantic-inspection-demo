"""Day 4.5b: independent UR10e A/B inspection captures and moving-arm visibility."""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import traceback

import numpy as np

from arm_camera_check import load_motion_evidence
from arm_inspection import observe_capture, require_static_scene, screen_candidates
from camera_executors import ArmMountedCameraExecutor
from capture_metadata import CONVENTIONS, prepare_output_directory, rigid_transform, validate_capture_pair
from inspection_knowledge import InspectionKnowledge
from inspection_state import ObservationState
from part_geometry import PARTS
from viewpoint_planner import load_viewpoints


ROOT = Path(__file__).resolve().parents[1]


def read_camera_evidence(path, version, motion):
    if path is None:
        candidates = [ROOT/"outputs/arm_camera_01/arm_camera_report.json", ROOT/"outputs/arm_camera_codex_gui_02/arm_camera_report.json"]
        path = next((p for p in candidates if p.is_file()), candidates[0])
    path = path.expanduser().resolve()
    raw = path.read_bytes()
    evidence = json.loads(raw)
    checks = evidence.get("checks", {})
    if (evidence.get("demo_step") != "arm_camera_two_rgb_views" or evidence.get("run_status") != "passed"
            or not checks or not all(checks.values()) or len(evidence.get("captures", [])) != 2):
        raise ValueError("Wrist-camera verification must pass before A/B integration")
    if (evidence["runtime"]["isaacsim_package"] != version or evidence["model_manifest"] != motion["model_manifest"]
            or evidence["runtime"]["cumotion_version"] != motion["runtime"]["cumotion_version"]):
        raise ValueError("Wrist-camera model/runtime evidence changed")
    return {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest()}


def write_json(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--part", choices=PARTS, default="B")
    parser.add_argument("--spec", choices=("X", "Y"), default="Y")
    parser.add_argument("--step3-report", type=Path)
    parser.add_argument("--camera-report", type=Path)
    parser.add_argument("--cell", type=Path, default=ROOT/"config/arm_inspection_cell.json")
    parser.add_argument("--viewpoints", type=Path, default=ROOT/"config/camera_views.json")
    parser.add_argument("--views", nargs=2, default=("front", "rear"), metavar=("FIRST", "SECOND"))
    parser.add_argument("--output-dir", type=Path, default=ROOT/"outputs/arm_inspection_01")
    parser.add_argument("--max-motion-seconds", type=float, default=20.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--exit-after-capture", action="store_true")
    args = parser.parse_args(argv)
    try:
        version = importlib.metadata.version("isaacsim")
        motion, _, motion_provenance = load_motion_evidence(args.step3_report, version)
        camera_provenance = read_camera_evidence(args.camera_report, version, motion)
        asset, model_dir = Path(motion["asset_path"]), Path(motion["model_directory"])
        if not asset.is_file() or not model_dir.is_dir():
            raise FileNotFoundError("Verified UR10e asset/model not found")
        cell_path, views_path = args.cell.expanduser().resolve(), args.viewpoints.expanduser().resolve()
        cell = json.loads(cell_path.read_bytes())
        if (cell["schema_version"] != 1 or cell["length_unit"] != "meter" or not cell["source"]
                or cell["approval_status"] != "prototype_not_approved"):
            raise ValueError("Expected a prototype cell configuration in meters")
        world_part = rigid_transform(cell["T_world_part"])
        candidates, _, views_document = load_viewpoints(views_path)
        by_id = {v.view_id: v for v in candidates}
        if len(set(args.views)) != 2 or any(name not in by_id for name in args.views):
            raise ValueError("Choose two distinct IDs present in the viewpoint configuration")
        if not math.isfinite(args.max_motion_seconds) or args.max_motion_seconds <= 0:
            raise ValueError("Motion timeout must be finite and positive")
        knowledge = InspectionKnowledge.load(ROOT/"knowledge/inspection_knowledge.json")
        spec = knowledge.specification(args.spec)
        output = prepare_output_directory(args.output_dir)
        (output/"geometry").mkdir()
    except (OSError, ValueError, KeyError, TypeError, importlib.metadata.PackageNotFoundError) as error:
        print(f"ARM INSPECTION FAILED: {error}", file=sys.stderr, flush=True)
        return 1

    report = {"schema_version": 1, "demo_step": "arm_inspection_scene_probe", "run_status": "running",
              "created_utc": datetime.now(timezone.utc).isoformat(), "part": args.part, "spec": args.spec,
              "required_region_ids": list(spec.required_region_ids), "inspection_spec_version": spec.version,
              "step3_report": motion_provenance, "camera_report": camera_provenance,
              "runtime": {"python_executable": sys.executable, "isaacsim_package": version, "headless": args.headless},
              "cell": {"path": str(cell_path), "sha256": hashlib.sha256(cell_path.read_bytes()).hexdigest(), **cell},
              "viewpoints": {"path": str(views_path), "sha256": hashlib.sha256(views_path.read_bytes()).hexdigest(), **views_document},
              "probe_view_ids": list(args.views), "knowledge": knowledge.provenance(),
              "coordinate_conventions": {**CONVENTIONS, "part": "same local part coordinates as Day 2-4; fixed world pose from cell config",
                                         "composition": "T_world_camera = T_world_flange @ T_flange_camera",
                                         "geometry_capture_binding": "full scene hash and physics step/time checked before/after held RGB"},
              "planner_executed": False, "viewpoint_selection": "two explicit diagnostic views after IK screening",
              "collision_safety_validated": False, "part_and_support_collision_enabled": False,
              "defect_decision": "not_evaluated", "movements": [], "captures": [], "reachability": [],
              "checks": {}, "events_file": "arm_inspection_events.jsonl",
              "scope": "A/B scene, candidate IK screening, two actual wrist RGB captures and observation updates including current arm geometry; closed-loop selection/comparison deferred"}
    report["coordinate_conventions"].pop("app_update_count")
    app, adapter, exit_code = None, None, 0
    images, records = [], []
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": args.headless, "renderer": "RaytracedLighting",
                             "extra_args": ["--enable", "isaacsim.robot_motion.cumotion", "--enable", "isaacsim.sensors.experimental.rtx"]})
        import cumotion
        from PIL import Image
        from arm_camera_evidence import EVIDENCE_TOLERANCES
        from arm_inspection_adapter import ArmInspectionAdapter
        from arm_model_adapter import read_usd_reference, validate_manifest
        from arm_motion import TOLERANCES
        from day2_visibility import save_overlay

        _, reference = read_usd_reference(asset)
        manifest = validate_manifest(model_dir, reference, version)
        if manifest != motion["model_manifest"] or cumotion.__version__ != motion["runtime"]["cumotion_version"]:
            raise ValueError("Verified model/runtime changed")
        adapter = ArmInspectionAdapter(app, asset, model_dir, reference, headless=args.headless,
                                       definition=PARTS[args.part], world_part=world_part, cell=cell)
        executor = ArmMountedCameraExecutor(adapter, adapter.world_part, adapter.mount, max_motion_seconds=args.max_motion_seconds)
        report.update(controller=adapter.settings, camera=adapter.camera_settings, model_manifest=manifest,
                      evidence_tolerances=EVIDENCE_TOLERANCES, motion_tolerances=TOLERANCES)
        report["runtime"]["cumotion_version"] = cumotion.__version__
        initial = adapter.snapshot()
        state = ObservationState(initial[0].geometry_version, spec.version, spec.required_points(initial[1]))
        feasible, report["reachability"] = screen_candidates(adapter, adapter.world_part, adapter.mount, candidates)
        feasible_ids = {v.view_id for v in feasible}
        for candidate in report["reachability"]:
            print(f"CANDIDATE | {candidate['view_id']} | {candidate['status']} | {candidate['reason']}", flush=True)
        last_clock, last_rgb = None, None
        with (output/report["events_file"]).open("x", encoding="utf-8") as events:
            sequence = 0
            def emit(event):
                nonlocal sequence
                sequence += 1
                events.write(json.dumps({"sequence": sequence, "recorded_utc": datetime.now(timezone.utc).isoformat(), **event}, allow_nan=False)+"\n")
                if event["event"] != "physics_sample":
                    events.flush()

            emit({"event": "candidate_screening", "reachability": report["reachability"]})
            if any(name not in feasible_ids for name in args.views):
                report.update(run_status="blocked", blocked_reason="requested_probe_view_infeasible")
                exit_code = 2
            else:
                for name in args.views:
                    print(f"ARM INSPECTION MOVE | {args.part}/{args.spec}/{name}", flush=True)
                    movement = executor.move_to_part_pose(by_id[name].T_part_camera, goal_id=name, emit=emit)
                    report["movements"].append(movement)
                    if movement["status"] != "reached":
                        report.update(run_status="blocked", blocked_reason=movement["reason"])
                        exit_code = 2
                        break
                    before = adapter.snapshot()
                    require_static_scene(initial, before)
                    image = f"images/{name}.png"
                    rgb, capture = executor.capture(image)
                    after = adapter.snapshot()
                    capture["rgb_array_sha256"] = hashlib.sha256(rgb.tobytes()).hexdigest()
                    before_state = state.summary()
                    visibility, clock = observe_capture(before, after, rgb, capture, state, spec, name,
                                                         previous_reference=last_clock, previous_rgb=last_rgb)
                    Image.fromarray(rgb).save(output/image)
                    overlay = f"images/{name}_visibility.png"
                    save_overlay(rgb, visibility, f"UR10e {args.part}/{args.spec}/{name}", output/overlay)
                    scene_file = f"geometry/{name}.json"
                    write_json(output/scene_file, {"scene": after[0].to_dict(),
                                                  "surface_regions": {k: v.to_dict() for k, v in after[1].items()},
                                                  "camera": after[2].to_dict(), "snapshot": after[3]})
                    record = {"view_id": name, "capture": capture, "visibility": visibility, "overlay_image": overlay,
                              "scene_file": scene_file, "state_before_capture": before_state, "state_after_visibility": state.summary()}
                    report["captures"].append(record)
                    emit({"event": "inspection_observation", **record})
                    images.append(rgb)
                    records.append(capture)
                    last_clock, last_rgb = clock, rgb.copy()
                    counts = ", ".join(f"{k}={v['visible_count']}/{v['total_count']}" for k,v in visibility["regions"].items())
                    arm_counts = {k: len(v) for k,v in visibility["robot_occluded_point_ids"].items()}
                    print(f"ARM INSPECTION CAPTURED | {name} | {counts} | arm_occlusion={arm_counts}", flush=True)
            report["final_state"] = state.summary()
            if exit_code == 0:
                report["image_comparison"] = validate_capture_pair(records, images)
            report["checks"] = {
                "two_controlled_camera_goals_reached": len(report["movements"]) == 2 and all(m["status"] == "reached" for m in report["movements"]),
                "two_fresh_distinct_images": len(records) == 2 and "image_comparison" in report,
                "two_observations_from_capture_scene": len(state.evaluated_capture_ids) == 2,
                "robot_instance_meshes_in_both_snapshots": len(records) == 2 and all(
                    any(m["instance_proxy"] for m in c["visibility"]["geometry_snapshot"]["visible_meshes"]) for c in report["captures"]),
                "arm_geometry_changes_between_views": len(records) == 2 and report["captures"][0]["visibility"]["geometry_snapshot"]["full_scene_sha256"]
                    != report["captures"][1]["visibility"]["geometry_snapshot"]["full_scene_sha256"]}
            if exit_code == 0:
                if not all(report["checks"].values()):
                    raise RuntimeError(f"Integration checks failed: {report['checks']}")
                report["run_status"] = "passed"
            emit({"event": "probe_finished", "run_status": report["run_status"], "final_state": report["final_state"]})
    except (Exception, KeyboardInterrupt) as error:
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        report.update(run_status="failed", error=str(error))
        print(f"ARM INSPECTION FAILED: {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
    finally:
        try:
            write_json(output/"arm_inspection_report.json", report)
            print(f"REPORT | {output/'arm_inspection_report.json'}", flush=True)
            if exit_code == 0:
                print(f"ARM INSPECTION PASS | captures=2 | observation={report['final_state']['status']}", flush=True)
            elif exit_code == 2:
                print(f"ARM INSPECTION BLOCKED | {report['blocked_reason']}", flush=True)
            if app is not None and not args.headless and not args.exit_after_capture:
                print("Review the external arm view and saved wrist images, then close the window.", flush=True)
                while app.is_running():
                    app.update()
        except (Exception, KeyboardInterrupt) as error:
            exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
            print(f"ARM INSPECTION FAILED: report/display: {error}", file=sys.stderr, flush=True)
        finally:
            try:
                if adapter is not None:
                    adapter.close()
            except Exception:
                exit_code = 1
                traceback.print_exc()
            finally:
                if app is not None:
                    app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
