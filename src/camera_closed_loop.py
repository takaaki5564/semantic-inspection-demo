"""Day 4.5c: closed-loop inspection with free or UR10e eye-in-hand camera execution."""

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time
import traceback

from arm_camera_check import load_motion_evidence
from arm_inspection_check import ROOT, read_camera_evidence, write_json
from camera_executors import ArmMountedCameraExecutor, FreeCameraExecutor
from capture_metadata import CONVENTIONS, prepare_output_directory, rigid_transform
from executor_closed_loop import CameraInspectionSession, run_camera_session
from inspection_knowledge import InspectionKnowledge
from part_geometry import PARTS
from viewpoint_planner import load_viewpoints


def read_inspection_evidence(path, version, motion):
    if path is None:
        paths = [ROOT/"outputs/arm_inspection_B_Y_01/arm_inspection_report.json",
                 ROOT/"outputs/arm_inspection_codex_B_Y_02/arm_inspection_report.json"]
        path = next((p for p in paths if p.is_file()), paths[0])
    path = path.expanduser().resolve()
    raw = path.read_bytes()
    report = json.loads(raw)
    checks = report.get("checks", {})
    if (report.get("demo_step") != "arm_inspection_scene_probe" or report.get("run_status") != "passed"
            or not checks or not all(checks.values()) or len(report.get("captures", [])) != 2):
        raise ValueError("A/B arm scene verification must pass before closed-loop integration")
    if (report["runtime"]["isaacsim_package"] != version or report["model_manifest"] != motion["model_manifest"]
            or report["runtime"]["cumotion_version"] != motion["runtime"]["cumotion_version"]):
        raise ValueError("A/B arm scene model/runtime evidence changed")
    for source in (report["cell"], report["viewpoints"]):
        if hashlib.sha256(Path(source["path"]).read_bytes()).hexdigest() != source["sha256"]:
            raise ValueError("A/B arm scene source configuration changed; repeat scene verification")
    return {"path":str(path), "sha256":hashlib.sha256(raw).hexdigest()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executor", choices=("arm","free"), default="arm")
    parser.add_argument("--part", choices=PARTS, default="B")
    parser.add_argument("--spec", choices=("X","Y"), default="Y")
    parser.add_argument("--initial-view", default="front")
    parser.add_argument("--candidate-views", nargs="*", default=("front","rear"))
    parser.add_argument("--max-actions", type=int, default=3)
    parser.add_argument("--max-motion-seconds", type=float, default=20.0)
    parser.add_argument("--disable-camera-motion", action="store_true", help="Disable additional views after initial positioning/capture")
    parser.add_argument("--step3-report", type=Path)
    parser.add_argument("--camera-report", type=Path)
    parser.add_argument("--inspection-report", type=Path)
    parser.add_argument("--cell", type=Path, default=ROOT/"config/arm_inspection_cell.json")
    parser.add_argument("--viewpoints", type=Path, default=ROOT/"config/camera_views.json")
    parser.add_argument("--knowledge", type=Path, default=ROOT/"knowledge/inspection_knowledge.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT/"outputs/camera_loop_01")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--exit-after-run", action="store_true")
    parser.add_argument("--single-step", action="store_true", help="Run only the initial observation/next-plan cycle for review")
    args = parser.parse_args(argv)
    try:
        version = importlib.metadata.version("isaacsim")
        motion, _, motion_source = load_motion_evidence(args.step3_report, version)
        camera_source = read_camera_evidence(args.camera_report, version, motion)
        inspection_source = read_inspection_evidence(args.inspection_report, version, motion)
        asset, model = Path(motion["asset_path"]), Path(motion["model_directory"])
        if not asset.is_file() or not model.is_dir():
            raise FileNotFoundError("Verified robot asset/model not found")
        cell_path, views_path = args.cell.expanduser().resolve(), args.viewpoints.expanduser().resolve()
        cell = json.loads(cell_path.read_bytes())
        if (cell["schema_version"] != 1 or cell["length_unit"] != "meter" or not cell["source"]
                or cell["approval_status"] != "prototype_not_approved"):
            raise ValueError("Expected a prototype cell in meters")
        part_pose = rigid_transform(cell["T_world_part"])
        all_views, weight, view_document = load_viewpoints(views_path)
        by_id = {v.view_id:v for v in all_views}
        if (args.initial_view not in by_id or len(set(args.candidate_views)) != len(args.candidate_views)
                or any(v not in by_id for v in args.candidate_views)):
            raise ValueError("Initial/candidate IDs must be present and candidate IDs unique")
        initial, candidates = by_id[args.initial_view], tuple(by_id[v] for v in args.candidate_views)
        if args.max_actions < 0 or not math.isfinite(args.max_motion_seconds) or args.max_motion_seconds <= 0:
            raise ValueError("Action budget must be nonnegative and motion time finite and positive")
        knowledge = InspectionKnowledge.load(args.knowledge)
        spec = knowledge.specification(args.spec)
        output = prepare_output_directory(args.output_dir)
        (output/"geometry").mkdir()
    except (OSError, ValueError, KeyError, TypeError, importlib.metadata.PackageNotFoundError) as error:
        print(f"CAMERA LOOP FAILED: {error}",file=sys.stderr,flush=True)
        return 1

    capabilities = () if args.disable_camera_motion else ("camera_pose_change",)
    context = {"part":args.part, "inspection_spec_id":args.spec, "inspection_spec_version":spec.version,
               "required_region_ids":list(spec.required_region_ids), "knowledge_snapshot":knowledge.document,
               "cell_sha256":hashlib.sha256(cell_path.read_bytes()).hexdigest(),
               "viewpoints_sha256":hashlib.sha256(views_path.read_bytes()).hexdigest(),
               "T_world_part":part_pose.tolist(), "initial_view_id":initial.view_id,
               "T_part_camera_initial":initial.T_part_camera.tolist(),
               "candidate_views":[{"view_id":v.view_id,"T_part_camera":v.T_part_camera.tolist()} for v in sorted(candidates,key=lambda v:v.view_id)],
               "max_actions":args.max_actions, "available_capability_ids":list(capabilities),
               "rotation_cost_m_per_rad":weight}
    report = {"schema_version":1, "demo_step":"camera_executor_closed_loop", "run_status":"running",
              "control_mode":"single_step" if args.single_step else "continuous",
              "created_utc":datetime.now(timezone.utc).isoformat(), "executor":args.executor,
              "comparison_context":context, "cell_configuration":cell, "viewpoint_configuration":view_document,
              "step3_report":motion_source, "camera_report":camera_source, "inspection_report":inspection_source,
              "runtime":{"python_executable":sys.executable,"isaacsim_package":version,"headless":args.headless},
              "max_motion_seconds":args.max_motion_seconds,
              "coordinate_conventions":{**CONVENTIONS,"capture_binding":"measured poses and held geometry at each captured physics step",
                                        "free_clock":"renderer reference equals held physics time; one parking-joint physics step after each camera move",
                                        "arm_clock":"renderer reference clock equals held physics time"},
              "collision_safety_validated":False, "defect_decision":"not_evaluated",
              "events_file":"events.jsonl", "motion_events_file":"motion_events.jsonl", "predictions":[]}
    report["coordinate_conventions"].pop("app_update_count")
    app, adapter, exit_code = None, None, 0
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless":args.headless,"renderer":"RaytracedLighting",
                             "extra_args":["--enable","isaacsim.robot_motion.cumotion","--enable","isaacsim.sensors.experimental.rtx"]})
        import cumotion
        from PIL import Image
        from arm_inspection_geometry import scene_fingerprint
        from arm_model_adapter import read_usd_reference, validate_manifest
        from day2_visibility import save_overlay
        from inspection_loop_adapter import InspectionLoopAdapter

        _, reference = read_usd_reference(asset)
        manifest = validate_manifest(model, reference, version)
        if manifest != motion["model_manifest"] or cumotion.__version__ != motion["runtime"]["cumotion_version"]:
            raise ValueError("Verified runtime/model changed")
        adapter = InspectionLoopAdapter(app,asset,model,reference,headless=args.headless,executor_mode=args.executor,
                                        definition=PARTS[args.part],world_part=part_pose,cell=cell)
        executor = (ArmMountedCameraExecutor(adapter,part_pose,adapter.mount,max_motion_seconds=args.max_motion_seconds)
                    if args.executor == "arm" else FreeCameraExecutor(adapter,part_pose))
        first = adapter.snapshot()
        optics = first[2].to_dict()
        optics.pop("T_world_camera")
        context.update(camera_optics=optics, static_scene_sha256=first[3]["static_scene_sha256"], part_geometry_version=first[0].geometry_version)
        report.update(camera=adapter.camera_settings,controller=adapter.settings,model_manifest=manifest)
        report["runtime"]["cumotion_version"] = cumotion.__version__

        def save_capture(rgb, record, snapshot):
            capture_id = record["capture_id"]
            Image.fromarray(rgb).save(output/record["capture"]["image"])
            record["overlay_image"] = f"images/{capture_id}_visibility.png"
            save_overlay(rgb,record["visibility"],f"{args.executor}/{args.part}/{args.spec}/{record['view_id']}",output/record["overlay_image"])
            record["scene_file"] = f"geometry/{capture_id}.json"
            write_json(output/record["scene_file"],{"scene":snapshot[0].to_dict(),"camera":snapshot[2].to_dict(),
                       "surface_regions":{k:v.to_dict() for k,v in snapshot[1].items()},"snapshot":snapshot[3]})

        def save_prediction(action_index, view, scene, screening, current):
            name = f"geometry/prediction_{len(report['predictions']):02d}.json"
            camera = replace(current[2],T_world_camera=scene.T_world_part@view.T_part_camera)
            evidence = next(r for r in screening if r["view_id"] == view.view_id)
            write_json(output/name,{"evidence_source":"predicted_from_simulator_geometry","view_id":view.view_id,
                       "scene":scene.to_dict(),"camera":camera.to_dict(),"surface_regions":{k:v.to_dict() for k,v in current[1].items()},
                       "screening":evidence,"actual_source_snapshot":current[3]})
            report["predictions"].append({"action_index":action_index,"view_id":view.view_id,"scene_file":name,
                                          "scene_sha256":scene_fingerprint(scene)})

        with (output/"events.jsonl").open("x",encoding="utf-8") as events, (output/"motion_events.jsonl").open("x",encoding="utf-8") as motions:
            def save_event(event):
                events.write(json.dumps(event,allow_nan=False)+"\n")
                events.flush()
                kind = event["event_type"]
                if kind == "observation_applied":
                    counts = ", ".join(f"{k}={v['observed_count']}/{v['total_count']}" for k,v in event["state_after_visibility"]["regions"].items())
                    print(f"OBSERVED | {event['capture_id']} | {counts} | new={event['actual_new_point_count']}",flush=True)
                elif kind == "plan_computed":
                    plan = event["plan"]
                    detail = f"view={plan['action']['view_id']} predicted_new={plan['action']['predicted_new_point_count']}" if plan["action"] else plan["reason"]
                    print(f"PLAN | {plan['status']} | {detail}",flush=True)
                elif kind in ("initial_view_screened","candidates_screened"):
                    for item in event["screening"]:
                        print(f"CANDIDATE | {item['view_id']} | {item['status']} | {item['reason']}",flush=True)
                elif kind in ("movement_started","movement_blocked","session_stopped"):
                    print(f"{kind.upper()} | {event.get('view_id','')} | {event.get('reason','')}",flush=True)
                elif kind == "step_completed":
                    snapshot = event["snapshot"]
                    write_json(output/f"step_{snapshot['steps_completed']:02d}.json", snapshot)
                    counts = ", ".join(f"{r['region_id']}={r['state']}" for r in snapshot["region_states"])
                    print(f"STEP | {snapshot['steps_completed']} | {snapshot['status']} | {counts}",flush=True)
            motion_sequence = 0
            def save_motion(event):
                nonlocal motion_sequence
                motion_sequence += 1
                motions.write(json.dumps({"sequence":motion_sequence,"recorded_utc":datetime.now(timezone.utc).isoformat(),**event},allow_nan=False)+"\n")
                if event["event"] != "physics_sample":
                    motions.flush()
            started = time.perf_counter()
            run_session = run_camera_session
            if args.single_step:
                def run_session(*positional, **options):
                    controller = CameraInspectionSession(*positional, **options)
                    controller.step()
                    return controller.report()
            session = run_session(adapter,executor,knowledge,args.spec,candidates,initial,
                                          session_id=f"{args.executor}_{args.part}_{args.spec}",max_actions=args.max_actions,
                                          available_capabilities=capabilities,rotation_cost_m_per_rad=weight,
                                          on_event=save_event,on_motion=save_motion,on_capture=save_capture,on_prediction=save_prediction)
            report["elapsed_loop_seconds"] = time.perf_counter()-started
        report["session"] = session
        report["run_status"] = ("passed" if session["status"] == "inspection_observation_satisfied"
                                else "paused" if session["status"] == "paused" else "blocked")
        exit_code = 0 if report["run_status"] in ("passed", "paused") else 2
    except (Exception,KeyboardInterrupt) as error:
        exit_code = 130 if isinstance(error,KeyboardInterrupt) else 1
        report.update(run_status="failed",error=str(error))
        print(f"CAMERA LOOP FAILED: {error}",file=sys.stderr,flush=True)
        traceback.print_exc()
    finally:
        try:
            write_json(output/"camera_loop_report.json",report)
            print(f"REPORT | {output/'camera_loop_report.json'}",flush=True)
            if report["run_status"] in ("passed","blocked","paused"):
                session = report["session"]
                print(f"CAMERA LOOP {report['run_status'].upper()} | executor={args.executor} | captures={len(session['captures'])} | additional_actions={session['actions_executed']}",flush=True)
            if app is not None and not args.headless and not args.exit_after_run:
                print("Review the arm and saved images, then close the window.",flush=True)
                while app.is_running():
                    app.update()
        except (Exception,KeyboardInterrupt) as error:
            exit_code = 130 if isinstance(error,KeyboardInterrupt) else 1
            print(f"CAMERA LOOP FAILED: report/display: {error}",file=sys.stderr,flush=True)
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
