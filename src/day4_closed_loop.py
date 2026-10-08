"""Day 4: execute planned camera views, capture fresh RGB, update observations and replan."""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import sys
import time
import traceback

from PIL import Image

from capture_metadata import CONVENTIONS, prepare_output_directory
from closed_loop import run_session
from day2_visibility import save_overlay
from inspection_knowledge import InspectionKnowledge
from part_geometry import PARTS
from verify_day4 import verify_document
from viewpoint_planner import load_viewpoints


ROOT = Path(__file__).resolve().parents[1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--part", choices=tuple(PARTS), default="B")
    parser.add_argument("--spec", choices=("X", "Y"), default="Y")
    parser.add_argument("--max-actions", type=int, default=3)
    parser.add_argument("--disable-camera-motion", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--exit-after-run", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/day4_run_01")
    parser.add_argument("--knowledge", type=Path, default=ROOT / "knowledge/inspection_knowledge.json")
    parser.add_argument("--viewpoints", type=Path, default=ROOT / "config/camera_views.json")
    args = parser.parse_args(argv)
    if args.max_actions < 0:
        parser.error("--max-actions must be nonnegative")
    try:
        knowledge = InspectionKnowledge.load(args.knowledge)
        knowledge.specification(args.spec)
        candidates, weight, view_document = load_viewpoints(args.viewpoints)
        initial = next((view for view in candidates if view.view_id == "front"), None)
        if initial is None:
            raise ValueError("Day 4 demo requires the shared 'front' initial viewpoint")
        output_dir = prepare_output_directory(args.output_dir)
    except (OSError, ValueError, KeyError) as error:
        print(f"DAY4 FAILED: {error}", file=sys.stderr)
        return 1

    capabilities = [] if args.disable_camera_motion else ["camera_pose_change"]
    document = {"schema_version": 1, "demo_step": "day4_closed_loop", "run_status": "running",
                "created_utc": datetime.now(timezone.utc).isoformat(), "approval_status": "prototype_not_approved",
                "observation_evidence_source": "derived_from_simulator_geometry", "defect_decision": "not_evaluated",
                "coordinate_conventions": CONVENTIONS, "knowledge_snapshot": knowledge.document,
                "viewpoint_configuration": view_document, "available_capability_ids": capabilities,
                "part": {"label": args.part, "nominal_rib_height_m": PARTS[args.part].rib_height_m},
                "runtime": {"python_executable": sys.executable, "isaacsim_package": importlib.metadata.version("isaacsim")}}
    app, adapter, exit_code = None, None, 0
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720,
                             "renderer": "RaytracedLighting",
                             "extra_args": ["--enable", "isaacsim.sensors.experimental.rtx"]})
        from day4_adapter import ClosedLoopAdapter
        adapter = ClosedLoopAdapter(app, headless=args.headless)
        adapter.set_part(PARTS[args.part])
        adapter.set_part_relative_pose(initial.T_part_camera)
        adapter.hold(0.25 if args.headless else 2)

        def save_capture(rgb, capture, visibility):
            Image.fromarray(rgb).save(output_dir / capture["image"])
            image_path = Path(capture["image"])
            save_overlay(rgb, visibility, image_path.stem,
                         output_dir / image_path.with_name(image_path.stem + "_visibility.png"))

        with (output_dir / "events.jsonl").open("x", encoding="utf-8") as stream:
            def save_event(event):
                stream.write(json.dumps(event, allow_nan=False) + "\n")
                stream.flush()
                kind = event["event_type"]
                if kind == "observation_applied":
                    counts = " | ".join(f"{key}={value['observed_count']}/{value['total_count']}"
                                        for key, value in event["state_after_visibility"]["regions"].items())
                    print(f"OBSERVED {event['capture_id']} | {counts} | actual new={event['actual_new_point_count']}", flush=True)
                    if not args.headless:
                        adapter.hold(2)
                elif kind == "plan_computed":
                    plan = event["plan"]
                    action = plan["action"]
                    detail = f"view={action['view_id']} predicted new={action['predicted_new_point_count']}" if action else plan["reason"]
                    print(f"PLAN {plan['status']} | {detail}", flush=True)
                elif kind in ("action_started", "action_completed", "session_stopped", "session_failed"):
                    print(f"{kind.upper()} | {event.get('action_id', '')} {event.get('reason', '')}", flush=True)

            session = run_session(adapter, knowledge, args.spec, candidates, session_id=f"{args.part}_{args.spec}",
                                  max_actions=args.max_actions, available_capabilities=capabilities,
                                  rotation_cost_m_per_rad=weight, on_event=save_event, on_capture=save_capture)
        document.update(run_status="completed", session=session)
        document["acceptance_checks"] = verify_document(document, output_dir)
        with (output_dir / "closed_loop_report.json").open("x", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, allow_nan=False)
            stream.write("\n")
        print(f"DAY4 COMPLETE | status={session['status']} | actions={session['actions_executed']} | {output_dir}", flush=True)
        if not (args.headless or args.exit_after_run):
            print("Close the Isaac Sim window to exit.", flush=True)
            while app.is_running():
                app.update()
                time.sleep(1 / 120)
    except (Exception, KeyboardInterrupt) as error:
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        print("DAY4 INTERRUPTED" if exit_code == 130 else "DAY4 FAILED", file=sys.stderr, flush=True)
        traceback.print_exc()
        document.update(run_status="failed", error=str(error))
        report_path = output_dir / "closed_loop_report.json"
        if not report_path.exists():
            report_path.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8")
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
