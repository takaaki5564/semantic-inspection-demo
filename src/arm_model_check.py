"""UR10e Step 2: verify cuMotion forward kinematics against the Step 1 USD."""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import traceback

from arm_kinematics import compare_model


ROOT = Path(__file__).resolve().parents[1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step1-report", type=Path, default=ROOT / "outputs/arm_step1_01/arm_preflight_report.json")
    parser.add_argument("--model-dir", type=Path, default=ROOT / "config/robots/ur10e")
    parser.add_argument("--export-model", action="store_true", help="Export a fresh model under the NEW output directory instead of using --model-dir")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/arm_step2_01")
    args = parser.parse_args(argv)
    try:
        step1_path = args.step1_report.expanduser().resolve()
        step1_bytes = step1_path.read_bytes()
        step1 = json.loads(step1_bytes)
        if step1.get("demo_step") != "arm_step1_load" or step1.get("run_status") != "passed" or not step1.get("robot", {}).get("checks") or not all(step1["robot"]["checks"].values()):
            raise ValueError("Step 1 must pass before model checking")
        version = importlib.metadata.version("isaacsim")
        if version != step1["runtime"]["isaacsim_package"]:
            raise ValueError("Isaac Sim version changed; repeat Step 1 first")
        asset = Path(step1["asset"]["path"])
        if not asset.is_file():
            raise FileNotFoundError(f"Step 1 asset not found: {asset}")
        output_dir = args.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=False)
    except (OSError, ValueError, KeyError, importlib.metadata.PackageNotFoundError) as error:
        print(f"UR10E MODEL FAILED: {error}", file=sys.stderr, flush=True)
        return 1

    model_dir = output_dir / "model" if args.export_model else args.model_dir.expanduser().resolve()
    report = {"schema_version": 1, "demo_step": "arm_step2_model_check", "run_status": "running",
              "created_utc": datetime.now(timezone.utc).isoformat(),
              "verification_scope": "numerical FK: authored USD zero pose and USD joint frames vs cuMotion URDF/XRDF; no physical motion or IK tested",
              "runtime": {"python_executable": sys.executable, "isaacsim_package": version},
              "step1_report": {"path": str(step1_path), "sha256": hashlib.sha256(step1_bytes).hexdigest()},
              "asset_path": str(asset), "model_directory": str(model_dir),
              "coordinate_conventions": {"units": "meters", "angles": "radians",
                                         "matrix": "column vectors: p_base = T_base_flange @ p_flange"}}
    app, exit_code = None, 0
    try:
        from isaacsim import SimulationApp

        extensions = ["--enable", "isaacsim.robot_motion.cumotion"]
        if args.export_model:
            extensions += ["--enable", "isaacsim.asset.exporter.urdf"]
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "extra_args": extensions})
        import omni.timeline
        import cumotion
        from isaacsim.robot_motion.cumotion import load_cumotion_robot
        from arm_model_adapter import read_usd_reference, export_model, validate_manifest

        timeline = omni.timeline.get_timeline_interface()
        start_time = timeline.get_current_time()
        if timeline.is_playing():
            raise RuntimeError("Step 2 requires a stopped timeline")
        stage, reference = read_usd_reference(asset)
        if args.export_model:
            export_model(stage, reference, model_dir, version)
        report["model_manifest"] = validate_manifest(model_dir, reference, version)
        robot = load_cumotion_robot(model_dir)
        kin = robot.kinematics
        if kin.base_frame_name() != reference["base_frame"] or reference["tool_frame"] not in robot.robot_description.tool_frame_names():
            raise ValueError("Loaded model has unexpected base or tool frame")
        names = robot.controlled_joint_names
        limits = {name: [kin.cspace_coord_limits(i).lower, kin.cspace_coord_limits(i).upper] for i, name in enumerate(names)}
        report["runtime"]["cumotion_version"] = cumotion.__version__
        report["usd_reference"] = reference

        def show_case(case):
            status = "PASS" if case["passed"] else "FAIL"
            print(f"FK {status} | {case['case_id']} | position_m={case['position_error_m']:.3e} | angle_rad={case['orientation_error_rad']:.3e}", flush=True)

        report["comparison"] = compare_model(reference, model_joint_names=names, model_limits_rad=limits,
                                             model_pose=lambda q: kin.pose(q, reference["tool_frame"]).matrix(), on_case=show_case)
        if timeline.is_playing() or timeline.get_current_time() != start_time:
            raise RuntimeError("Timeline unexpectedly advanced during numerical model checking")
        report["timeline_seconds"] = float(timeline.get_current_time())
        if not all(report["comparison"]["checks"].values()):
            raise ValueError(f"Model consistency checks failed: {report['comparison']['checks']}")
        report["run_status"] = "passed"
    except (Exception, KeyboardInterrupt) as error:
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        report.update(run_status="failed", error=str(error))
        print(f"UR10E MODEL FAILED: {error}", file=sys.stderr, flush=True)
        traceback.print_exc()
    finally:
        try:
            report_path = output_dir / "arm_model_report.json"
            with report_path.open("x", encoding="utf-8") as stream:
                json.dump(report, stream, indent=2, allow_nan=False)
                stream.write("\n")
            print(f"REPORT | {report_path}", flush=True)
            if exit_code == 0:
                print(f"UR10E MODEL PASS | cases={len(report['comparison']['cases'])} | base={reference['base_frame']} | tool={reference['tool_frame']}", flush=True)
        except Exception as error:
            exit_code = 1
            print(f"UR10E MODEL FAILED: could not save report: {error}", file=sys.stderr, flush=True)
            traceback.print_exc()
        finally:
            if app is not None:
                app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
