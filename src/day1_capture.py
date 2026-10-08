"""Capture two RGB views using the existing Isaac Sim Python environment."""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import sys
import time
import traceback

from capture_metadata import CONVENTIONS, prepare_output_directory, validate_capture_pair


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Capture without a GUI")
    parser.add_argument("--exit-after-capture", action="store_true", help="Close after saving evidence")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[1] / "outputs",
                        help="New output directory; existing directories are never overwritten")
    args = parser.parse_args()
    try:
        output_dir = prepare_output_directory(args.output_dir)
    except FileExistsError:
        parser.error("Output directory already exists. Choose a new directory with --output-dir.")

    # Preserve the proven startup order: SimulationApp before any omni/pxr imports.
    from isaacsim import SimulationApp
    app = SimulationApp({
        "headless": args.headless, "width": 1280, "height": 720,
        "renderer": "RaytracedLighting",
        "extra_args": ["--enable", "isaacsim.sensors.experimental.rtx"],
    })
    adapter = None
    exit_code = 0
    try:
        from PIL import Image
        from sim_adapter import CaptureAdapter

        adapter = CaptureAdapter(app, headless=args.headless)
        target = (0.0, 0.0, 0.08)
        first_eye = (0.85, -1.05, 0.75)
        second_eye = (0.85, 1.05, 0.75)
        adapter.set_part_relative_view(first_eye, target)
        print("DAY1 SCENE READY | R1=green, R2=orange | simulator markers", flush=True)
        adapter.hold(0.25 if args.headless else 2.0)
        images, records = [], []

        for index in (1, 2):
            if index == 2:
                print("MOVING CAMERA TO VIEW 02", flush=True)
                adapter.move_view(first_eye, second_eye, target, seconds=0.1 if args.headless else 3.0)
                adapter.hold(0.25 if args.headless else 1.0)
            name = f"images/view_{index:02d}.png"
            rgb, record = adapter.capture(name)
            Image.fromarray(rgb).save(output_dir / name)
            images.append(rgb)
            records.append(record)
            print(f"CAPTURED view_{index:02d} | position_m={record['world_position_m']}", flush=True)
            if index == 1 and not args.headless:
                adapter.hold(2.0)

        checks = validate_capture_pair(records, images)
        metadata = {
            "schema_version": 1, "demo_step": "day1_two_rgb_views",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "runtime": {"python": sys.version, "python_executable": sys.executable,
                        "isaacsim_package": importlib.metadata.version("isaacsim"),
                        "renderer": "RaytracedLighting"},
            "coordinate_conventions": CONVENTIONS,
            "scene": {"geometry_version": "day1_plate_low_rib_v1",
                      "geometry_source": "simulator_ground_truth",
                      "region_markers": {"R1": "green", "R2": "orange"},
                      "inspection_evaluated": False},
            "camera": {"prim_path": adapter.CAMERA_PATH, "resolution_hw": [480, 640],
                       "focal_length_mm": 24.0},
            "captures": records, "checks": checks,
        }
        with (output_dir / "camera_poses.json").open("x", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2, allow_nan=False)
            stream.write("\n")
        print(f"DAY1 CAPTURE COMPLETE | {output_dir}", flush=True)
        print(f"Pixel difference: {checks['mean_absolute_pixel_difference']:.3f} | captured; inspection not evaluated", flush=True)
        if not (args.headless or args.exit_after_capture):
            print("Close the Isaac Sim window to exit.", flush=True)
            while app.is_running():
                app.update()
                time.sleep(1 / 120)
        return 0
    except Exception:
        exit_code = 1
        print("DAY1 CAPTURE FAILED", file=sys.stderr, flush=True)
        traceback.print_exc()
        return exit_code
    except KeyboardInterrupt:
        exit_code = 130
        print("DAY1 CAPTURE INTERRUPTED", file=sys.stderr, flush=True)
        return exit_code
    finally:
        try:
            if adapter is not None:
                adapter.close()
        except Exception:
            exit_code = 1
            traceback.print_exc()
        finally:
            # Installed SimulationApp uses fast shutdown, which otherwise masks failures with exit 0.
            app.close(exit_code=exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
