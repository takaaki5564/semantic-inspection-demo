"""UR10e Step 1: compose and inspect a static USD scene. No motion control."""

import math

from capture_metadata import camera_look_at, rigid_transform


ROBOT_PATH = "/World/UR10e"
FLANGE_PATH = ROBOT_PATH + "/wrist_3_link/flange"
JOINT_NAMES = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
}


def inspect_robot(stage):
    """Read composed USD, including instance proxies; do not initialize physics."""
    from pxr import Usd, UsdGeom, UsdPhysics

    robot = stage.GetPrimAtPath(ROBOT_PATH)
    flange = stage.GetPrimAtPath(FLANGE_PATH)
    if not robot.IsValid() or not flange.IsValid():
        raise ValueError(f"UR10e robot or flange missing: {ROBOT_PATH}, {FLANGE_PATH}")
    prims = list(Usd.PrimRange(robot, Usd.TraverseInstanceProxies()))
    joints, meshes, roots = [], [], []
    for prim in prims:
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(str(prim.GetPath()))
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint = UsdPhysics.RevoluteJoint(prim)
            lower, upper = joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
            if lower is None or upper is None or not all(math.isfinite(v) for v in (lower, upper)) or lower > upper:
                raise ValueError(f"Invalid authored joint limits: {prim.GetPath()}")
            joints.append({"name": prim.GetName(), "prim_path": str(prim.GetPath()),
                           "lower_limit_deg": float(lower), "upper_limit_deg": float(upper)})
        if prim.IsA(UsdGeom.Mesh):
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            meshes.append({"prim_path": str(prim.GetPath()), "point_count": len(points) if points is not None else 0})
    checks = {
        "six_expected_revolute_joints": len(joints) == 6 and {j["name"] for j in joints} == JOINT_NAMES,
        "one_articulation_root": len(roots) == 1,
        "flange_is_xformable": bool(UsdGeom.Xformable(flange)),
        "meshes_have_points": bool(meshes) and all(m["point_count"] > 0 for m in meshes),
        "stage_is_meters_z_up": UsdGeom.GetStageMetersPerUnit(stage) == 1.0 and UsdGeom.GetStageUpAxis(stage) == "Z",
    }
    if not all(checks.values()):
        raise ValueError(f"UR10e USD checks failed: {checks}")

    def transform(prim):
        import numpy as np

        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return rigid_transform(np.asarray(matrix, dtype=float).T).tolist()

    return {"robot_prim_path": ROBOT_PATH, "flange_prim_path": FLANGE_PATH,
            "articulation_root_paths": roots, "joint_count": len(joints),
            "joints": sorted(joints, key=lambda joint: joint["name"]), "mesh_count": len(meshes),
            "meshes": meshes, "T_world_robot": transform(robot), "T_world_flange": transform(flange),
            "pose_source": "authored USD transforms; physics has not been initialized",
            "variant_selections": {name: robot.GetVariantSets().GetVariantSet(name).GetVariantSelection()
                                   for name in robot.GetVariantSets().GetNames()}, "checks": checks}


def load_scene(app, asset_path, *, headless=False):
    """Load the local robot reference and an external viewing camera, timeline stopped."""
    import omni.timeline
    import omni.usd
    from pxr import Gf, Usd, UsdGeom, UsdLux

    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("Step 1 requires a stopped timeline")
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage after SimulationApp startup")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    robot = stage.DefinePrim(ROBOT_PATH, "Xform")
    if not robot.GetReferences().AddReference(str(asset_path)):
        raise ValueError(f"Could not reference local UR10e USD: {asset_path}")
    for name, selection in {"Physics": "PhysX", "Gripper": "None", "Sensor": "None"}.items():
        variants = robot.GetVariantSets().GetVariantSet(name)
        if name in robot.GetVariantSets().GetNames():
            if selection not in variants.GetVariantNames() or not variants.SetVariantSelection(selection):
                raise ValueError(f"Could not select robot variant {name}={selection}")

    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.035))
    floor.AddScaleOp().Set(Gf.Vec3f(4, 4, 0.06))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.16, 0.18, 0.21)])
    UsdLux.DomeLight.Define(stage, "/World/FillLight").CreateIntensityAttr(500.0)
    light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    light.CreateIntensityAttr(1800.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(-35, -20, 15))

    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    center = bounds.ComputeWorldBound(robot).ComputeAlignedRange().GetMidpoint()
    camera = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    camera.CreateFocalLengthAttr(24.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    eye = (center[0] + 2.0, center[1] - 2.0, center[2] + 1.5)
    camera.MakeMatrixXform().Set(Gf.Matrix4d(camera_look_at(eye, center).T.tolist()))
    if not headless:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("No active viewport for UR10e display")
        viewport.camera_path = str(camera.GetPath())
    initial_time = timeline.get_current_time()
    for _ in range(30):
        if not app.is_running():
            raise RuntimeError("Isaac Sim closed before UR10e load verification")
        app.update()
    if timeline.is_playing() or timeline.get_current_time() != initial_time:
        raise RuntimeError("Timeline unexpectedly advanced during static Step 1")
    result = inspect_robot(stage)
    result["timeline_seconds"] = float(timeline.get_current_time())
    result["overview_camera_prim_path"] = str(camera.GetPath())
    return result
