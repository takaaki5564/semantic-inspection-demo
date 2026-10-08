"""UR10e Step 3: IK and two bounded, physically measured arm movements."""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import traceback

from arm_motion import IK_TOLERANCES, TOLERANCES, execute_goal
from capture_metadata import rigid_transform


ROOT = Path(__file__).resolve().parents[1]


def read_step2(path, version):
    raw = path.read_bytes()
    report = json.loads(raw)
    checks = report.get("comparison", {}).get("checks", {})
    if report.get("demo_step") != "arm_step2_model_check" or report.get("run_status") != "passed" or not checks or not all(checks.values()):
        raise ValueError("Step 2 must pass before controlled motion")
    if report["runtime"]["isaacsim_package"] != version:
        raise ValueError("Isaac Sim version changed; repeat Step 1 and Step 2")
    return report, hashlib.sha256(raw).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step2-report", type=Path, default=ROOT / "outputs/arm_step2_01/arm_model_report.json")
    parser.add_argument("--goals", type=Path, default=ROOT / "config/robots/ur10e/motion_probe_goals.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/arm_step3_01")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--exit-after-run", action="store_true", help="Close GUI after saving the report")
    parser.add_argument("--max-motion-seconds", type=float, default=12.0, help="Per-goal simulation-time limit including settling")
    args = parser.parse_args(argv)
    try:
        version = importlib.metadata.version("isaacsim")
        step2_path = args.step2_report.expanduser().resolve()
        step2, digest = read_step2(step2_path, version)
        asset, model_dir = Path(step2["asset_path"]), Path(step2["model_directory"])
        if not asset.is_file() or not model_dir.is_dir():
            raise FileNotFoundError("Step 2 asset or model directory not found")
        goals_path = args.goals.expanduser().resolve()
        goals_bytes = goals_path.read_bytes()
        goals_config = json.loads(goals_bytes)
        if goals_config["base_frame"] != "base_link" or goals_config["tool_frame"] != "flange" or len(goals_config["goals"]) != 2:
            raise ValueError("Expected two base_link-relative flange goals")
        if len({g["goal_id"] for g in goals_config["goals"]}) != 2:
            raise ValueError("Goal IDs must differ")
        for goal in goals_config["goals"]:
            rigid_transform(goal["T_base_flange_target"])
        if not math.isfinite(args.max_motion_seconds) or args.max_motion_seconds <= 0:
            raise ValueError("Motion timeout must be positive and finite")
        output_dir = args.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=False)
    except (OSError, ValueError, KeyError, TypeError, importlib.metadata.PackageNotFoundError) as error:
        print(f"UR10E MOTION FAILED: {error}", file=sys.stderr, flush=True)
        return 1

    report = {"schema_version": 1, "demo_step": "arm_step3_controlled_motion", "run_status": "running",
              "created_utc": datetime.now(timezone.utc).isoformat(),
              "verification_scope": "isolated simulated UR10e IK, joint target motion and measured flange settling; no wrist camera or inspection integration",
              "runtime": {"python_executable": sys.executable, "isaacsim_package": version, "headless": args.headless},
              "step2_report": {"path": str(step2_path), "sha256": digest}, "asset_path": str(asset),
              "model_directory": str(model_dir), "goals_source": {"path": str(goals_path), "sha256": hashlib.sha256(goals_bytes).hexdigest(),
                                                                     "description": goals_config["description"]},
              "coordinate_conventions": {"units": "meters", "angles": "radians", "world": "right-handed +Z up",
                                         "matrix": "column vectors; T_world_flange = T_world_base @ T_base_flange",
                                         "quaternion_order": "wxyz when reading PhysX via RigidPrim"},
              "tolerances": TOLERANCES, "ik_tolerances": IK_TOLERANCES, "max_motion_seconds": args.max_motion_seconds,
              "goals": [], "events_file": "arm_motion_events.jsonl", "collision_safety_validated": False}
    app, exit_code = None, 0
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": args.headless, "renderer": "RaytracedLighting",
                             "extra_args": ["--enable", "isaacsim.robot_motion.cumotion"]})
        import cumotion
        from arm_model_adapter import read_usd_reference, validate_manifest
        from arm_motion_adapter import ArmMotionAdapter

        _, reference = read_usd_reference(asset)
        manifest = validate_manifest(model_dir, reference, version)
        if manifest != step2["model_manifest"] or reference["source_layer_sha256"] != step2["usd_reference"]["source_layer_sha256"]:
            raise ValueError("Step 2 evidence no longer matches current model/source")
        report["model_manifest"] = manifest
        report["runtime"]["cumotion_version"] = cumotion.__version__
        if cumotion.__version__ != step2["runtime"]["cumotion_version"]:
            raise ValueError("cuMotion version changed; repeat Step 2")
        adapter = ArmMotionAdapter(app, asset, model_dir, reference, headless=args.headless)
        report["controller"] = adapter.settings
        report["motion_limits"] = {k: v.tolist() for k, v in adapter.limits.items()}
        with (output_dir / report["events_file"]).open("x", encoding="utf-8") as events:
            sequence = 0

            def emit(event):
                nonlocal sequence
                sequence += 1
                events.write(json.dumps({"sequence": sequence, "recorded_utc": datetime.now(timezone.utc).isoformat(), **event}, allow_nan=False)+"\n")
                if event["event"] != "physics_sample":
                    events.flush()

            emit({"event": "controller_initialized", "settings": adapter.settings, "state": adapter.read_state()})
            for goal in goals_config["goals"]:
                print(f"MOTION START | {goal['goal_id']}", flush=True)
                result = execute_goal(adapter, goal, adapter.limits, emit, max_motion_seconds=args.max_motion_seconds)
                report["goals"].append(result)
                if result["status"] != "reached":
                    report.update(run_status="blocked", blocked_reason=result["reason"])
                    exit_code = 2
                    print(f"MOTION BLOCKED | {goal['goal_id']} | {result['reason']}", flush=True)
                    break
                error = result["final_error"]
                print(f"MOTION REACHED | {goal['goal_id']} | position_m={error['position_error_m']:.6f} | angle_rad={error['orientation_error_rad']:.6f} | settled_s={result['settled_seconds']:.3f}", flush=True)
        if exit_code == 0:
            report["run_status"] = "passed"
        report["checks"] = {"validated_step2_model": True, "two_goals_reached": len(report["goals"]) == 2 and all(g["status"] == "reached" for g in report["goals"]),
                            "both_goals_show_physical_motion": len(report["goals"]) == 2 and all(g["max_actual_joint_displacement_rad"] >= 0.03 and g["physics_steps"] > 0 for g in report["goals"])}
    except (Exception, KeyboardInterrupt) as error:
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        report.update(run_status="failed", error=str(error))
        print(f"UR10E MOTION FAILED: {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
    finally:
        try:
            report_path = output_dir / "arm_motion_report.json"
            with report_path.open("x", encoding="utf-8") as stream:
                json.dump(report, stream, indent=2, allow_nan=False)
                stream.write("\n")
            print(f"REPORT | {report_path}", flush=True)
            if exit_code == 0:
                print("UR10E MOTION PASS | goals=2", flush=True)
            if app is not None and not args.headless and not args.exit_after_run:
                print("GUI remains open at the final pose. Close the window to finish.", flush=True)
                while app.is_running():
                    app.update()  # Manual physics stepping has ended; final pose remains fixed.
        except (Exception, KeyboardInterrupt) as error:
            exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
            print(f"UR10E MOTION FAILED: saving report / final display: {error}", file=sys.stderr, flush=True)
            traceback.print_exc()
        finally:
            if app is not None:
                app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
