"""Replay saved geometry visibility and state transitions without starting Isaac Sim."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

from capture_metadata import reference_time, rigid_transform
from inspection_state import ObservationState
from part_geometry import SceneGeometry, SurfaceRegion
from visibility import CameraModel, evaluate_visibility


def acceptance_checks(cases):
    if [case["case_id"] for case in cases] != ["A_initial", "B_initial", "B_alternate"]:
        raise ValueError("Expected A initial, B initial, B alternate captures in that order")
    a, b, alternate = cases
    ar, br, cr = (case["visibility"]["regions"] for case in cases)
    return {
        "same_initial_camera_pose": bool(np.allclose(a["capture"]["T_world_camera"], b["capture"]["T_world_camera"])),
        "different_part_geometry": a["visibility"]["part_geometry_version"] != b["visibility"]["part_geometry_version"],
        "same_required_sample_points": all(ar[region]["points"][index]["point_part_m"] == br[region]["points"][index]["point_part_m"]
                                           for region in ar for index in range(len(ar[region]["points"]))),
        "A_initial_required_samples_visible": all(region["visible_count"] == region["total_count"] for region in ar.values()),
        "B_initial_R1_visible": br["R1"]["visible_count"] == br["R1"]["total_count"],
        "B_initial_R2_self_occluded": br["R2"]["reason_counts"].get("self_occlusion", 0) > 0,
        "B_initial_R2_less_visible": br["R2"]["visible_count"] < ar["R2"]["visible_count"],
        "B_alternate_R2_more_visible": cr["R2"]["visible_count"] > br["R2"]["visible_count"],
        "B_geometry_unchanged_between_views": b["visibility"]["part_geometry_version"] == alternate["visibility"]["part_geometry_version"],
        "B_viewpoints_differ": not bool(np.allclose(b["capture"]["T_world_camera"], alternate["capture"]["T_world_camera"])),
        "B_combined_observation_satisfied": alternate["state_after_visibility"]["inspection_observation_satisfied"],
        "part_switch_resets_observation": b["state_before_capture"]["status"] == "not_captured" and all(
            region["observed_count"] == 0 for region in b["state_before_capture"]["regions"].values()),
    }


def verify(output_dir):
    output_dir = Path(output_dir).resolve()
    document = json.loads((output_dir / "visibility_report.json").read_text(encoding="utf-8"))
    if document["evidence_source"] != "derived_from_simulator_geometry" or document["defect_decision"] != "not_evaluated":
        raise ValueError("Unexpected evidence source or defect claim")
    state, previous_clock = None, None
    for case in document["cases"]:
        scene = SceneGeometry.from_dict(case["scene"])
        regions = {key: SurfaceRegion(**value) for key, value in case["surface_regions"].items()}
        camera = CameraModel(**case["camera"])
        required = document["inspection_specification"]["required_region_ids"]
        spec_version = document["inspection_specification"]["version"]
        report = evaluate_visibility(scene, regions, camera, required_region_ids=required,
                                     inspection_spec_version=spec_version,
                                     max_incidence_angle_deg=case["visibility"]["max_incidence_angle_deg"],
                                     surface_tolerance_m=case["visibility"]["surface_tolerance_m"])
        if report != case["visibility"]:
            raise ValueError(f"Visibility replay differs from saved result: {case['case_id']}")
        capture = case["capture"]
        if capture["status"] != "captured" or capture["evidence_source"] != "simulator_rgb":
            raise ValueError("Missing simulator RGB capture evidence")
        if (not np.allclose(rigid_transform(capture["T_world_camera"]), camera.T_world_camera)
                or not np.allclose(rigid_transform(capture["T_world_part"]), scene.T_world_part)
                or not np.allclose(scene.T_world_part @ rigid_transform(capture["T_part_camera"]), camera.T_world_camera)):
            raise ValueError("Camera/part capture pose differs from visibility snapshot")
        clock = reference_time(capture["render_reference_time"])
        if previous_clock is not None and clock <= previous_clock:
            raise ValueError("Capture render reference time did not advance")
        previous_clock = clock
        for image_name in (capture["image"], case["overlay_image"]):
            path = (output_dir / image_name).resolve()
            if not path.is_relative_to(output_dir):
                raise ValueError("Image path must be inside output directory")
            with Image.open(path) as image:
                array = np.asarray(image)
                if image.mode != "RGB" or array.shape != (*camera.resolution_hw, 3) or np.ptp(array) == 0:
                    raise ValueError("Expected nonuniform RGB image matching camera resolution")
        if state is None or state.part_geometry_version != scene.geometry_version:
            state = ObservationState(scene.geometry_version, spec_version,
                                     {region: regions[region].point_ids for region in required})
        if state.summary() != case["state_before_capture"]:
            raise ValueError("State before capture does not replay")
        state.record_capture(case["case_id"])
        if state.summary() != case["state_after_capture"]:
            raise ValueError("Capture changed observation state unexpectedly")
        state.apply_visibility(case["case_id"], report)
        if state.summary() != case["state_after_visibility"]:
            raise ValueError("Observation state does not replay")
    checks = acceptance_checks(document["cases"])
    if checks != document["acceptance_checks"] or not all(checks.values()):
        raise ValueError(f"Day 2 acceptance failed: {checks}")
    return checks


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path, nargs="?", default=Path(__file__).resolve().parents[1] / "outputs/day2_run_01")
    args = parser.parse_args()
    checks = verify(args.output_dir)
    print(f"PASS: Day 2 geometry, visibility replay, capture evidence and state transitions | {checks}")
