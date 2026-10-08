from contextlib import redirect_stderr
from dataclasses import replace
from io import StringIO
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from arm_inspection_check import write_json
from compare_camera_executors import compare_runs,main,verify_run
from executor_closed_loop import run_camera_session
from executor_loop_fixtures import FakeCell,FakeExecutor
from inspection_knowledge import InspectionKnowledge
from viewpoint_planner import load_viewpoints
from render_camera_evidence import validate_render_camera
from render_camera_fixtures import renderer_parameters


def make_run(folder,mode,*,spec_id="Y",rib_height=.12,fault=None,**kwargs):
    folder = Path(folder)
    folder.mkdir()
    (folder/"images").mkdir()
    (folder/"geometry").mkdir()
    adapter = FakeCell(rib_height,fault=fault)
    knowledge = InspectionKnowledge.load(ROOT/"knowledge/inspection_knowledge.json")
    views,weight,_ = load_viewpoints(ROOT/"config/camera_views.json")
    views = views[:2]
    initial = adapter.snapshot()
    predictions = []
    def save_capture(rgb,record,snapshot):
        record["capture"]["renderer_camera"] = validate_render_camera(renderer_parameters(snapshot[2]), snapshot[2])
        Image.fromarray(rgb).save(folder/record["capture"]["image"])
        record["scene_file"] = f"geometry/{record['capture_id']}.json"
        write_json(folder/record["scene_file"],{"scene":snapshot[0].to_dict(),"surface_regions":{k:v.to_dict() for k,v in snapshot[1].items()},
                                               "camera":snapshot[2].to_dict(),"snapshot":snapshot[3]})
    def save_prediction(index,view,scene,screening,current):
        name = f"geometry/prediction_{len(predictions)}.json"
        camera = replace(current[2],T_world_camera=scene.T_world_part@view.T_part_camera)
        write_json(folder/name,{"scene":scene.to_dict(),"surface_regions":{k:v.to_dict() for k,v in current[1].items()},"camera":camera.to_dict()})
        from arm_inspection_geometry import scene_fingerprint
        predictions.append({"action_index":index,"view_id":view.view_id,"scene_file":name,"scene_sha256":scene_fingerprint(scene)})
    session = run_camera_session(adapter,FakeExecutor(adapter),knowledge,spec_id,views,views[0],
                                 rotation_cost_m_per_rad=weight,on_capture=save_capture,on_prediction=save_prediction,**kwargs)
    optics = initial[2].to_dict()
    optics.pop("T_world_camera")
    context = {"part":"fixture", "inspection_spec_id":spec_id,"inspection_spec_version":knowledge.specification(spec_id).version,
               "knowledge_snapshot":knowledge.document,"T_world_part":initial[0].T_world_part.tolist(),"initial_view_id":"front",
               "candidate_views":[{"view_id":v.view_id,"T_part_camera":v.T_part_camera.tolist()} for v in views],
               "available_capability_ids":["camera_pose_change"],"rotation_cost_m_per_rad":weight,"camera_optics":optics,
               "static_scene_sha256":initial[3]["static_scene_sha256"],"part_geometry_version":initial[0].geometry_version,
               "cell_sha256":"fixture_cell","max_actions":3}
    report = {"demo_step":"camera_executor_closed_loop","run_status":"passed" if session["status"] == "inspection_observation_satisfied" else "blocked",
              "executor":mode,"comparison_context":context,"model_manifest":{"source":"numeric_fixture"},
              "runtime":{"isaacsim_package":"fixture_no_simulator"},"session":session,"predictions":predictions}
    path = folder/"camera_loop_report.json"
    write_json(path,report)
    return path


class ComparisonTests(unittest.TestCase):
    def test_saved_captures_and_candidate_plans_replay_for_B_Y_A_Y_and_B_X(self):
        for height,spec,count,captures in ((.12,"Y",126,2),(.025,"Y",126,1),(.12,"X",63,1)):
            with tempfile.TemporaryDirectory() as folder:
                arm = make_run(Path(folder)/"arm","arm",rib_height=height,spec_id=spec)
                free = make_run(Path(folder)/"free","free",rib_height=height,spec_id=spec)
                compared = compare_runs(arm,free)
                self.assertEqual(compared["arm"]["final_required_observed_count"],count)
                self.assertEqual(compared["free"]["capture_count"],captures)

    def test_valid_blocked_arm_run_is_reported_honestly_against_successful_free_run(self):
        with tempfile.TemporaryDirectory() as folder:
            arm = make_run(Path(folder)/"arm","arm",fault="actual_arm_occlusion")
            free = make_run(Path(folder)/"free","free")
            comparison = compare_runs(arm,free)
            self.assertEqual(comparison["arm"]["status"],"blocked")
            self.assertEqual(comparison["arm"]["final_required_observed_count"],66)
            self.assertEqual(comparison["free"]["final_required_observed_count"],126)

    def test_cell_or_budget_mismatch_rejects_comparison(self):
        for key,value in (("cell_sha256","different"),("max_actions",0)):
            with tempfile.TemporaryDirectory() as folder:
                arm = make_run(Path(folder)/"arm","arm")
                free = make_run(Path(folder)/"free","free")
                data = json.loads(free.read_text())
                data["comparison_context"][key] = value
                free.write_text(json.dumps(data))
                with self.assertRaisesRegex(ValueError,"differ"):
                    compare_runs(arm,free)

    def test_tampered_png_visibility_prediction_or_execution_cannot_pass_replay(self):
        for fault in ("pixels","visibility","prediction","execution","renderer_pose"):
            with tempfile.TemporaryDirectory() as folder:
                path = make_run(Path(folder)/"arm","arm")
                report = json.loads(path.read_text())
                if fault == "pixels":
                    image = path.parent/report["session"]["captures"][0]["capture"]["image"]
                    rgb = np.asarray(Image.open(image)).copy()
                    rgb[0,0] = 255-rgb[0,0]
                    Image.fromarray(rgb).save(image)
                elif fault == "visibility":
                    report["session"]["captures"][1]["visibility"]["regions"]["R2"]["visible_count"] = 0
                elif fault == "prediction":
                    report["predictions"][0]["scene_sha256"] = "tampered"
                elif fault == "renderer_pose":
                    report["session"]["captures"][0]["capture"]["renderer_camera"]["parameters"]["cameraViewTransform"] = np.eye(4).ravel().tolist()
                else:
                    report["session"]["selected_view_ids"] = ["front"]
                path.write_text(json.dumps(report))
                with self.subTest(fault=fault),self.assertRaises(ValueError):
                    verify_run(path)

    def test_existing_comparison_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            arm = make_run(folder/"arm","arm")
            free = make_run(folder/"free","free")
            output = folder/"comparison.json"
            output.write_text("original")
            with redirect_stderr(StringIO()):
                self.assertEqual(main(["--arm-report",str(arm),"--free-report",str(free),"--output",str(output)]),1)
            self.assertEqual(output.read_text(),"original")


if __name__ == "__main__":
    unittest.main()
