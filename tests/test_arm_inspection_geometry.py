from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src"))
from arm_inspection_geometry import read_geometry, scene_fingerprint, triangle_mesh
from geometry_fixtures import demo_scene, matrix, overhead_camera, scene_of_boxes, target_region
from part_geometry import SceneGeometry
from visibility import evaluate_visibility

try:
    from pxr import Gf, Usd, UsdGeom
except ImportError:
    Usd = None


class MeshGeometryTests(unittest.TestCase):
    def test_triangle_indices_and_scaled_rotated_world_transform(self):
        transform = np.array([[0,-2,0,3],[3,0,0,4],[0,0,4,5],[0,0,0,1]], dtype=float)
        faces, ids = triangle_mesh([[0,0,0],[1,0,0],[0,1,0]], [3], [2,0,1], transform)
        np.testing.assert_allclose(faces, [[[1,4,5],[3,4,5],[3,7,5]]])
        self.assertEqual(ids, ("mesh_face:0",))

    def test_unsupported_or_invalid_mesh_fails_instead_of_ignoring_geometry(self):
        good = {"points": [[0,0,0],[1,0,0],[0,1,0]], "counts": [3], "indices": [0,1,2], "world_transform": np.eye(4)}
        for bad in ({"counts": [4]}, {"indices": [0,1,4]}, {"indices": [-1,1,2]},
                    {"subdivision": "catmullClark"}, {"holes": [0]}, {"indices": [0,0,2]},
                    {"points": [[0,0,0],[np.nan,0,0],[0,1,0]]}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                triangle_mesh(**{**good, **bad})

    def test_moving_arm_mesh_changes_occlusion_without_changing_part_identity(self):
        static = scene_of_boxes([("/World/Part/Target", np.eye(4))])
        triangle = np.array([[[-1,-1,1.5],[1,-1,1.5],[0,1,1.5]]])
        def with_arm(offset):
            return SceneGeometry(np.concatenate([static.triangles_world_m, triangle+[offset,0,0]]),
                                 static.prim_paths+("/World/UR10e/link/visuals/mesh",), static.face_ids+("mesh_face:0",), np.eye(4))
        before, after = with_arm(0), with_arm(3)
        args = {"required_region_ids": ["R1"], "inspection_spec_version": "test"}
        old = evaluate_visibility(before, {"R1": target_region()}, overhead_camera(), **args)
        new = evaluate_visibility(after, {"R1": target_region()}, overhead_camera(), **args)
        self.assertEqual(old["regions"]["R1"]["points"][0]["reason"], "scene_occlusion")
        self.assertEqual(new["regions"]["R1"]["visible_count"], 1)
        self.assertEqual(before.geometry_version, after.geometry_version)
        self.assertEqual(scene_fingerprint(before, exclude_robot=True), scene_fingerprint(after, exclude_robot=True))
        self.assertNotEqual(scene_fingerprint(before), scene_fingerprint(after))


@unittest.skipIf(Usd is None, "Optional USD reader tests require pxr, but not Isaac Sim startup")
class UsdSnapshotTests(unittest.TestCase):
    def fixture(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/Part")
        boxes = {"Plate": ((0,0,.03),(.9,.6,.06)), "Rib": ((0,0,.12),(.75,.045,.12)),
                 "R1Marker": ((-.08,-.17,.061),(.5,.15,.002)), "R2Marker": ((.08,.17,.061),(.5,.15,.002))}
        for name, (pos, scale) in boxes.items():
            cube = UsdGeom.Cube.Define(stage, "/World/Part/"+name)
            cube.CreateSizeAttr(1)
            cube.MakeMatrixXform().Set(Gf.Matrix4d(matrix(pos, scale).T.tolist()))
        UsdGeom.Xform.Define(stage, "/Library/Arm")
        for name, purpose in (("Visual", "default"), ("Collision", "guide")):
            mesh = UsdGeom.Mesh.Define(stage, "/Library/Arm/"+name)
            mesh.CreatePointsAttr([(-1,-1,1.5),(1,-1,1.5),(0,1,1.5)])
            mesh.CreateFaceVertexCountsAttr([3])
            mesh.CreateFaceVertexIndicesAttr([0,1,2])
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreatePurposeAttr(purpose)
        link = UsdGeom.Xform.Define(stage, "/World/UR10e/Link")
        link.GetPrim().GetReferences().AddInternalReference("/Library/Arm")
        link.GetPrim().SetInstanceable(True)
        translate = link.AddTranslateOp()
        return stage, translate

    def test_instance_meshes_guide_exclusion_and_fresh_transform_cache(self):
        stage, translate = self.fixture()
        scene, regions, meta = read_geometry(stage)
        self.assertEqual(meta["visible_meshes"], [{"prim_path": "/World/UR10e/Link/Visual", "triangle_count": 1, "instance_proxy": True}])
        self.assertEqual(meta["excluded_invisible_or_guide_proxy_prims"], ["/World/UR10e/Link/Collision"])
        expected, _ = demo_scene(.12)
        order = np.argsort(expected.prim_paths, kind="stable")
        expected = SceneGeometry(expected.triangles_world_m[order], tuple(expected.prim_paths[i] for i in order),
                                 tuple(expected.face_ids[i] for i in order), expected.T_world_part)
        self.assertEqual(scene.geometry_version, expected.geometry_version)
        self.assertEqual(len(regions["R2"].point_ids), 63)
        translate.Set(Gf.Vec3d(2,0,0))
        moved, _, new = read_geometry(stage)
        self.assertEqual(meta["static_scene_sha256"], new["static_scene_sha256"])
        self.assertNotEqual(meta["full_scene_sha256"], new["full_scene_sha256"])
        self.assertEqual(scene.geometry_version, moved.geometry_version)

    def test_invisible_mesh_excluded_and_unknown_visible_geometry_rejected(self):
        stage, _ = self.fixture()
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/UR10e/Link")).MakeInvisible()
        self.assertFalse(read_geometry(stage)[2]["visible_meshes"])
        UsdGeom.Sphere.Define(stage, "/World/Unsupported")
        with self.assertRaisesRegex(ValueError, "Unsupported visible geometry"):
            read_geometry(stage)


if __name__ == "__main__":
    unittest.main()
