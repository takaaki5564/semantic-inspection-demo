"""Day 2 manual A/B visibility comparison; viewpoint planning is deferred to Day 3."""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np
from PIL import Image, ImageDraw

from capture_metadata import CONVENTIONS, prepare_output_directory
from inspection_state import ObservationState
from part_geometry import PARTS
from verify_day2 import acceptance_checks
from visibility import evaluate_visibility


SPECIFICATION = {"version": "day2_all_surface_samples_v1", "required_region_ids": ["R1", "R2"],
                 "satisfaction_rule": "all predefined required surface samples observed across captured views",
                 "evidence_source": "simulator_ground_truth_surface_definition"}
INITIAL_EYE = (0.85, -1.05, 0.75)
ALTERNATE_EYE = (0.85, 1.05, 0.75)
TARGET = (0, 0, 0.08)


def save_overlay(rgb, report, case_id, path):
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    for region in report["regions"].values():
        for point in region["points"]:
            if point["pixel_uv"] is None:
                continue
            x, y = point["pixel_uv"]
            if 0 <= x < image.width and 0 <= y < image.height:
                color = (0, 255, 80) if point["visible"] else (255, 60, 60)
                draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=color)
    counts = " | ".join(f"{key}: {value['visible_count']}/{value['total_count']}" for key, value in report["regions"].items())
    draw.rectangle((0, 0, image.width, 52), fill=(15, 20, 25))
    draw.text((8, 4), "SIM GEOMETRY VISIBILITY | not image recognition", fill=(255, 255, 255))
    draw.text((8, 20), f"{case_id} | {counts}", fill=(255, 255, 255))
    draw.text((8, 36), "green = visible | red = missing | defect decision not evaluated", fill=(255, 255, 255))
    image.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--exit-after-capture", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parents[1] / "outputs/day2_run_01")
    args = parser.parse_args()
    try:
        output_dir = prepare_output_directory(args.output_dir)
    except FileExistsError:
        parser.error("Output directory exists. Use --output-dir to choose a new directory.")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720,
                         "renderer": "RaytracedLighting",
                         "extra_args": ["--enable", "isaacsim.sensors.experimental.rtx"]})
    adapter, exit_code = None, 0
    try:
        from day2_adapter import VisibilityAdapter
        adapter = VisibilityAdapter(app, headless=args.headless)
        cases, state, previous_eye = [], None, None
        for case_id, definition, eye in (("A_initial", PARTS["A"], INITIAL_EYE),
                                         ("B_initial", PARTS["B"], INITIAL_EYE),
                                         ("B_alternate", PARTS["B"], ALTERNATE_EYE)):
            print(f"DAY2 CASE {case_id} | rib_height_m={definition.rib_height_m}", flush=True)
            adapter.set_part(definition)
            if previous_eye is not None and previous_eye != eye:
                print("MOVING CAMERA | manual predefined viewpoint", flush=True)
                adapter.move_view(previous_eye, eye, TARGET, seconds=0.1 if args.headless else 3)
            else:
                adapter.set_part_relative_view(eye, TARGET)
            adapter.hold(0.25 if args.headless else 2)
            previous_eye = eye
            scene, regions = adapter.geometry_snapshot()
            required = SPECIFICATION["required_region_ids"]
            if state is None or state.part_geometry_version != scene.geometry_version:
                state = ObservationState(scene.geometry_version, SPECIFICATION["version"],
                                         {key: regions[key].point_ids for key in required})
                print("OBSERVATION STATE RESET | new part geometry", flush=True)
            before_capture = state.summary()
            image_name = f"images/{case_id}.png"
            rgb, capture = adapter.capture(image_name)
            # Read actual geometry and optics after capture, while the scene is still fixed.
            captured_scene, regions = adapter.geometry_snapshot()
            camera = adapter.camera_model()
            if (scene.geometry_version != captured_scene.geometry_version
                    or not np.allclose(capture["T_world_part"], captured_scene.T_world_part)
                    or not np.allclose(capture["T_world_camera"], camera.T_world_camera)):
                raise RuntimeError("Geometry or camera changed between capture and visibility snapshot")
            scene = captured_scene
            state.record_capture(case_id)
            after_capture = state.summary()
            start = time.perf_counter()
            report = evaluate_visibility(scene, regions, camera, required_region_ids=required,
                                         inspection_spec_version=SPECIFICATION["version"])
            evaluation_ms = (time.perf_counter() - start) * 1000
            state.apply_visibility(case_id, report)
            Image.fromarray(rgb).save(output_dir / image_name)
            overlay = f"images/{case_id}_visibility.png"
            save_overlay(rgb, report, case_id, output_dir / overlay)
            cases.append({"case_id": case_id, "part_label": definition.label,
                          "nominal_rib_height_m": definition.rib_height_m,
                          "capture": capture, "camera": camera.to_dict(), "scene": scene.to_dict(),
                          "surface_regions": {key: value.to_dict() for key, value in regions.items()},
                          "visibility": report, "evaluation_ms": evaluation_ms, "overlay_image": overlay,
                          "state_before_capture": before_capture, "state_after_capture": after_capture,
                          "state_after_visibility": state.summary()})
            for region_id, region in report["regions"].items():
                print(f"{case_id} | {region_id}: {region['visible_count']}/{region['total_count']} visible "
                      f"| reasons={region['reason_counts']}", flush=True)
            print(f"OBSERVATION STATUS: {state.summary()['status']} | defect decision not evaluated", flush=True)
            if not args.headless:
                adapter.hold(2)

        checks = acceptance_checks(cases)
        document = {"schema_version": 1, "demo_step": "day2_parts_and_visibility",
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                    "evidence_source": "derived_from_simulator_geometry", "defect_decision": "not_evaluated",
                    "planner_implemented": False, "viewpoint_source": "manual_predefined",
                    "runtime": {"python_executable": sys.executable,
                                "isaacsim_package": importlib.metadata.version("isaacsim")},
                    "coordinate_conventions": CONVENTIONS, "inspection_specification": SPECIFICATION,
                    "cases": cases, "acceptance_checks": checks}
        with (output_dir / "visibility_report.json").open("x", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, allow_nan=False)
            stream.write("\n")
        if not all(checks.values()):
            raise RuntimeError(f"Day 2 acceptance failed: {checks}")
        print(f"DAY2 COMPLETE | {output_dir}", flush=True)
        if not (args.headless or args.exit_after_capture):
            print("Close the Isaac Sim window to exit.", flush=True)
            while app.is_running():
                app.update()
                time.sleep(1 / 120)
    except Exception:
        exit_code = 1
        print("DAY2 FAILED", file=sys.stderr, flush=True)
        traceback.print_exc()
    except KeyboardInterrupt:
        exit_code = 130
        print("DAY2 INTERRUPTED", file=sys.stderr, flush=True)
    finally:
        try:
            if adapter is not None:
                adapter.close()
        except Exception:
            exit_code = 1
            traceback.print_exc()
        finally:
            app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
