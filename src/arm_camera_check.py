"""UR10e wrist-camera substep: two camera-pose goals, held-pose RGB and evidence."""

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

from camera_executors import ArmMountedCameraExecutor
from arm_camera_evidence import EVIDENCE_TOLERANCES
from arm_motion import TOLERANCES
from capture_metadata import CONVENTIONS, prepare_output_directory, validate_capture_pair


ROOT = Path(__file__).resolve().parents[1]


def load_motion_evidence(path, version):
    if path is None:
        # Prefer the user's normal run; the earlier verified development GUI run is an explicit fallback.
        candidates = [ROOT/"outputs/arm_step3_01/arm_motion_report.json", ROOT/"outputs/arm_step3_codex_gui_01/arm_motion_report.json"]
        path = next((p for p in candidates if p.is_file()), candidates[0])
    path = path.expanduser().resolve()
    raw = path.read_bytes()
    report = json.loads(raw)
    checks = report.get("checks",{})
    if report.get("demo_step") != "arm_step3_controlled_motion" or report.get("run_status") != "passed" or not checks or not all(checks.values()):
        raise ValueError("Step 3 must pass before wrist-camera capture")
    if len(report.get("goals",[])) != 2 or any(g["status"] != "reached" for g in report["goals"]):
        raise ValueError("Step 3 must contain two reached goals")
    if report["runtime"]["isaacsim_package"] != version:
        raise ValueError("Isaac Sim version changed; repeat preflight")
    source = Path(report["goals_source"]["path"])
    goals_bytes = source.read_bytes()
    if hashlib.sha256(goals_bytes).hexdigest() != report["goals_source"]["sha256"]:
        raise ValueError("Step 3 goal source changed")
    return report, json.loads(goals_bytes)["goals"], {"path":str(path),"sha256":hashlib.sha256(raw).hexdigest()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step3-report",type=Path,help="Successful Step 3 report; prefer arm_step3_01, otherwise development GUI report")
    parser.add_argument("--output-dir",type=Path,default=ROOT/"outputs/arm_camera_01")
    parser.add_argument("--headless",action="store_true")
    parser.add_argument("--exit-after-capture",action="store_true")
    parser.add_argument("--max-motion-seconds",type=float,default=12.0)
    args = parser.parse_args(argv)
    try:
        version = importlib.metadata.version("isaacsim")
        evidence, goals, provenance = load_motion_evidence(args.step3_report, version)
        asset, model_dir = Path(evidence["asset_path"]), Path(evidence["model_directory"])
        if not asset.is_file() or not model_dir.is_dir():
            raise FileNotFoundError("Step 3 asset/model not found")
        if not math.isfinite(args.max_motion_seconds) or args.max_motion_seconds <= 0:
            raise ValueError("Motion timeout must be finite and positive")
        output = prepare_output_directory(args.output_dir)
    except (OSError,ValueError,KeyError,TypeError,importlib.metadata.PackageNotFoundError) as error:
        print(f"WRIST CAMERA FAILED: {error}",file=sys.stderr,flush=True)
        return 1
    print(f"STEP 3 EVIDENCE | {provenance['path']}",flush=True)
    report = {"schema_version":1,"demo_step":"arm_camera_two_rgb_views","run_status":"running",
              "created_utc":datetime.now(timezone.utc).isoformat(),"step3_report":provenance,
              "verification_scope":"fixed flange camera, camera-pose conversion, actual controlled motion and two fresh RGB captures; inspection not evaluated",
              "runtime":{"python_executable":sys.executable,"isaacsim_package":version,"headless":args.headless},
              "movements":[],"captures":[],"events_file":"arm_camera_events.jsonl","camera_metadata_file":"camera_poses.json",
              "scene":{"geometry_version":"wrist_capture_board_v1","geometry_source":"simulator_ground_truth",
                       "inspection_evaluated":False,"board_collision_enabled":False,"collision_safety_validated":False}}
    app,adapter,exit_code = None,None,0
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless":args.headless,"renderer":"RaytracedLighting",
                             "extra_args":["--enable","isaacsim.robot_motion.cumotion","--enable","isaacsim.sensors.experimental.rtx"]})
        import cumotion
        from PIL import Image
        from arm_model_adapter import read_usd_reference,validate_manifest
        from arm_camera_adapter import ArmCameraAdapter

        _,reference = read_usd_reference(asset)
        manifest = validate_manifest(model_dir,reference,version)
        if manifest != evidence["model_manifest"] or cumotion.__version__ != evidence["runtime"]["cumotion_version"]:
            raise ValueError("Step 3 model/runtime evidence changed")
        adapter = ArmCameraAdapter(app,asset,model_dir,reference,headless=args.headless)
        executor = ArmMountedCameraExecutor(adapter,adapter.world_part,adapter.mount,max_motion_seconds=args.max_motion_seconds)
        report.update(controller=adapter.settings,camera=adapter.camera_settings,model_manifest=manifest,
                      evidence_tolerances=EVIDENCE_TOLERANCES,motion_tolerances=TOLERANCES)
        report["runtime"]["cumotion_version"] = cumotion.__version__
        report["coordinate_conventions"] = {**CONVENTIONS,"part":"calibration-board center at top surface; +Z up",
                                              "composition":"T_world_camera = T_world_flange @ T_flange_camera",
                                              "reference_time":"renderer clock explicitly set to held physics snapshot; zero delta_time capture",
                                              "render_step_index":"count of synchronous Replicator render requests in this adapter"}
        # app_update_count is not recorded in this adapter; physics and render steps are explicit instead.
        report["coordinate_conventions"].pop("app_update_count")
        world_base = np.array(adapter.read_state()["T_world_base"])
        images,records = [],[]
        with (output/report["events_file"]).open("x",encoding="utf-8") as events:
            sequence = 0
            def emit(event):
                nonlocal sequence
                sequence += 1
                events.write(json.dumps({"sequence":sequence,"recorded_utc":datetime.now(timezone.utc).isoformat(),**event},allow_nan=False)+"\n")
                if event["event"] != "physics_sample":
                    events.flush()

            emit({"event":"camera_mounted","camera":adapter.camera_settings,"state":adapter.read_state()})
            for i,goal in enumerate(goals,1):
                # Independent probe goals derived from already verified flange poses, not inspection-planner decisions.
                world_camera_target = world_base @ np.array(goal["T_base_flange_target"]) @ adapter.mount
                part_camera_target = np.linalg.inv(adapter.world_part) @ world_camera_target
                print(f"WRIST MOVE | view_{i:02d}",flush=True)
                movement = executor.move_to_part_pose(part_camera_target,goal_id=f"view_{i:02d}",emit=emit)
                report["movements"].append(movement)
                if movement["status"] != "reached":
                    report.update(run_status="blocked",blocked_reason=movement["reason"])
                    exit_code = 2
                    print(f"WRIST BLOCKED | {movement['reason']}",flush=True)
                    break
                name = f"images/view_{i:02d}.png"
                rgb,record = executor.capture(name)
                Image.fromarray(rgb).save(output/name)
                records.append(record)
                images.append(rgb)
                report["captures"].append({"image":name,"goal_id":record["goal_id"],"camera_pose_error":record["camera_pose_error"]})
                emit({"event":"wrist_rgb_captured",**record})
                print(f"WRIST CAPTURED | view_{i:02d} | position_m={record['world_position_m']} | physics_s={record['physics_time_seconds']:.6f}",flush=True)
            if exit_code == 0:
                report["image_comparison"] = validate_capture_pair(records,images)
                report["run_status"] = "passed"
            metadata = {"schema_version":1,"demo_step":"arm_camera_two_rgb_views","run_status":report["run_status"],
                        "coordinate_conventions":report["coordinate_conventions"],"camera":adapter.camera_settings,
                        "evidence_tolerances":EVIDENCE_TOLERANCES,"motion_tolerances":TOLERANCES,
                        "scene":report["scene"],"captures":records}
            with (output/report["camera_metadata_file"]).open("x",encoding="utf-8") as stream:
                json.dump(metadata,stream,indent=2,allow_nan=False)
                stream.write("\n")
        report["checks"] = {"two_controlled_camera_goals_reached":len(report["movements"])==2 and all(m["status"]=="reached" for m in report["movements"]),
                            "two_fresh_distinct_images":len(records)==2 and "image_comparison" in report,
                            "fixed_mount_and_measured_camera_pose":len(records)==2,
                            "physics_held_during_both_captures":len(records)==2 and all(r["physics_held_during_capture"] for r in records)}
    except (Exception,KeyboardInterrupt) as error:
        exit_code = 130 if isinstance(error,KeyboardInterrupt) else 1
        report.update(run_status="failed",error=str(error))
        print(f"WRIST CAMERA FAILED: {error}",file=sys.stderr,flush=True)
        traceback.print_exc()
    finally:
        try:
            with (output/"arm_camera_report.json").open("x",encoding="utf-8") as stream:
                json.dump(report,stream,indent=2,allow_nan=False)
                stream.write("\n")
            print(f"REPORT | {output/'arm_camera_report.json'}",flush=True)
            if exit_code == 0:
                print("WRIST CAMERA PASS | captures=2",flush=True)
            if app is not None and not args.headless and not args.exit_after_capture:
                print("Close the window after reviewing the arm and saved wrist images.",flush=True)
                while app.is_running():
                    app.update()
        except (Exception,KeyboardInterrupt) as error:
            exit_code = 130 if isinstance(error,KeyboardInterrupt) else 1
            print(f"WRIST CAMERA FAILED: report/display: {error}",file=sys.stderr,flush=True)
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
