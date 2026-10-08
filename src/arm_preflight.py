"""UR10e Step 1: display a local USD and report joints/flange, without arm motion."""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[1]
TEST_ASSET = Path("exts/isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--exit-after-load", action="store_true")
    parser.add_argument("--robot-usd", type=Path, help="Local UR10e USD; defaults to the installed bundled test asset")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/arm_step1_01")
    args = parser.parse_args(argv)
    try:
        distribution = importlib.metadata.distribution("isaacsim")
        asset_path = (args.robot_usd or Path(distribution.locate_file("isaacsim")) / TEST_ASSET).expanduser().resolve()
        if not asset_path.is_file():
            raise FileNotFoundError(f"Local UR10e USD not found: {asset_path}; use --robot-usd PATH")
        output_dir = args.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=False)
    except (OSError, importlib.metadata.PackageNotFoundError) as error:
        print(f"UR10E LOAD FAILED: {error}", file=sys.stderr, flush=True)
        return 1

    report_path = output_dir / "arm_preflight_report.json"
    document = {"schema_version": 1, "demo_step": "arm_step1_load", "run_status": "running",
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "verification_scope": "static USD composition and display; IK, physical motion and camera mounting not evaluated",
                "runtime": {"python_executable": sys.executable, "python_version": sys.version,
                            "isaacsim_package": distribution.version, "headless": args.headless},
                "asset": {"path": str(asset_path), "source": "explicit_local_file" if args.robot_usd else "isaacsim_bundled_test_asset"},
                "coordinate_conventions": {"units": "meters", "world": "right-handed, +Z up",
                                           "matrix": "row-major JSON arrays; column vectors: p_world = T_world_frame @ p_frame",
                                           "joint_limits": "authored USD limits in degrees; not measured joint positions"}}
    app, exit_code = None, 0
    try:
        print(f"UR10E LOAD START | Isaac Sim {distribution.version} | {asset_path}", flush=True)
        # Preserve the working startup order: SimulationApp before omni/pxr imports.
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720,
                             "renderer": "RaytracedLighting"})
        from arm_preflight_adapter import load_scene

        document["robot"] = load_scene(app, asset_path, headless=args.headless)
        document["run_status"] = "passed"
        with report_path.open("x", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, allow_nan=False)
            stream.write("\n")
        robot = document["robot"]
        print(f"UR10E LOAD PASS | joints={robot['joint_count']} | meshes={robot['mesh_count']}", flush=True)
        print(f"FLANGE | {robot['flange_prim_path']}", flush=True)
        print(f"REPORT | {report_path}", flush=True)
        if not (args.headless or args.exit_after_load):
            print("Step 1: static display only. Close the window to exit; leave the timeline stopped.", flush=True)
            while app.is_running():
                app.update()
                time.sleep(1 / 120)
    except (Exception, KeyboardInterrupt) as error:
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        print("UR10E LOAD INTERRUPTED" if exit_code == 130 else "UR10E LOAD FAILED", file=sys.stderr, flush=True)
        traceback.print_exc()
        if not report_path.exists():
            document.update(run_status="failed", error=str(error))
            with report_path.open("x", encoding="utf-8") as stream:
                json.dump(document, stream, indent=2, allow_nan=False)
                stream.write("\n")
    finally:
        if app is not None:
            app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
