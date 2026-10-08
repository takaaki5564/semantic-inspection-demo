"""Read UR10e joint frames and export a kinematic model using installed Isaac Sim APIs."""

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from arm_preflight_adapter import ROBOT_PATH, FLANGE_PATH, inspect_robot
from capture_metadata import rigid_transform


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_layers(stage, asset_path):
    folder = Path(asset_path).resolve().parent
    return {str(Path(layer.realPath).resolve().relative_to(folder)): file_hash(layer.realPath)
            for layer in stage.GetUsedLayers() if not layer.anonymous and layer.realPath}


def read_usd_reference(asset_path):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    robot = stage.DefinePrim(ROBOT_PATH, "Xform")
    if not robot.GetReferences().AddReference(str(asset_path)):
        raise ValueError("Could not compose the Step 1 USD asset")
    for name, value in {"Physics": "PhysX", "Gripper": "None", "Sensor": "None"}.items():
        variants = robot.GetVariantSets().GetVariantSet(name)
        if not variants.SetVariantSelection(value):
            raise ValueError(f"Could not select {name}={value}")
    inspection = inspect_robot(stage)

    def world(path):
        matrix = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return rigid_transform(np.asarray(matrix, dtype=float).T)

    def anchor(position, quaternion):
        quaternion = Gf.Quatd(quaternion).GetNormalized()
        matrix = Gf.Matrix4d(1.0)
        matrix.SetRotateOnly(quaternion)
        matrix.SetTranslateOnly(Gf.Vec3d(position))
        return rigid_transform(np.asarray(matrix, dtype=float).T).tolist()

    joints = []
    for item in inspection["joints"]:
        joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(item["prim_path"]))
        parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if len(parents) != 1 or len(children) != 1:
            raise ValueError("Each UR10e joint must connect two named bodies")
        axis = str(joint.GetAxisAttr().Get())
        if axis not in ("X", "Y", "Z"):
            raise ValueError(f"Unsupported joint axis: {axis}")
        joints.append({"name": item["name"], "parent_path": str(parents[0]), "child_path": str(children[0]),
                       "axis": {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}[axis],
                       "T_parent_joint": anchor(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get()),
                       "T_child_joint": anchor(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get()),
                       "lower_limit_rad": float(np.deg2rad(item["lower_limit_deg"])),
                       "upper_limit_rad": float(np.deg2rad(item["upper_limit_deg"]))})
    bases = {j["parent_path"] for j in joints} - {j["child_path"] for j in joints}
    by_parent = {j["parent_path"]: j for j in joints}
    if len(bases) != 1 or len(by_parent) != len(joints):
        raise ValueError("Expected one unbranched UR10e chain")
    base = next(iter(bases))
    chain, current = [], base
    while current in by_parent:
        joint = by_parent[current]
        if joint in chain:
            raise ValueError("Cycle in USD joint graph")
        chain.append(joint)
        current = joint["child_path"]
    if len(chain) != 6 or not FLANGE_PATH.startswith(current + "/"):
        raise ValueError("Six-joint chain must terminate at the flange parent")
    return stage, {"base_prim_path": base, "flange_prim_path": FLANGE_PATH,
                   "base_frame": base.rsplit("/", 1)[-1], "tool_frame": "flange", "joints": chain,
                   "T_world_base": world(base).tolist(),
                   "T_base_flange_authored": (np.linalg.inv(world(base)) @ world(FLANGE_PATH)).tolist(),
                   "T_leaf_flange": (np.linalg.inv(world(current)) @ world(FLANGE_PATH)).tolist(),
                   "source_layer_sha256": source_layers(stage, asset_path)}


def export_model(stage, reference, model_dir, isaacsim_version):
    """Write a NEW model directory; source USD and existing configs stay unchanged."""
    from isaacsim.asset.exporter.urdf import UsdToUrdfConverter

    model_dir = Path(model_dir)
    model_dir.mkdir(parents=True, exist_ok=False)

    def kinematic_only(xml):
        for link in xml.findall("link"):
            for element in list(link):
                if element.tag in ("visual", "collision"):
                    link.remove(element)
        for joint in xml.findall("joint"):
            limit = joint.find("limit")
            if limit is not None:
                # Explicit prototype limit: avoid the loader's 1000 rad/s fallback.
                limit.set("velocity", "0.5")

    UsdToUrdfConverter(stage, root_prim_path=ROBOT_PATH).convert(str(model_dir / "robot.urdf"), postprocess=kinematic_only)
    # Standard XML parsing removes exporter drive comments; kinematic values are retained.
    xml = ET.parse(model_dir / "robot.urdf")
    ET.indent(xml, space="  ")
    xml.write(model_dir / "robot.urdf", encoding="utf-8", xml_declaration=True)
    with (model_dir / "robot.urdf").open("ab") as stream:
        stream.write(b"\n")
    names = [joint["name"] for joint in reference["joints"]]
    xrdf = {"format": "xrdf", "format_version": 2.0, "default_joint_positions": dict.fromkeys(names, 0.0),
            "cspace": {"joint_names": names, "acceleration_limits": [1.0] * 6, "jerk_limits": [10.0] * 6},
            "tool_frames": [reference["tool_frame"]]}
    (model_dir / "robot.xrdf").write_text(json.dumps(xrdf, indent=2) + "\n", encoding="utf-8")
    manifest = {"schema_version": 1, "source": "installed UR10e USD, exported with isaacsim.asset.exporter.urdf",
                "isaacsim_version": isaacsim_version, "source_layer_sha256": reference["source_layer_sha256"],
                "model_file_sha256": {name: file_hash(model_dir / name) for name in ("robot.urdf", "robot.xrdf")},
                "base_frame": reference["base_frame"], "tool_frame": reference["tool_frame"],
                "scope": "kinematics only; visual/collision elements removed; no collision spheres in XRDF",
                "prototype_limits": {"velocity_rad_s": 0.5, "acceleration_rad_s2": 1.0, "jerk_rad_s3": 10.0,
                                     "source": "demo configuration choices, not manufacturer limits or validated motion settings"}}
    (model_dir / "model_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def validate_manifest(model_dir, reference, isaacsim_version):
    model_dir = Path(model_dir)
    manifest = json.loads((model_dir / "model_manifest.json").read_text(encoding="utf-8"))
    expected = manifest["model_file_sha256"]
    if set(expected) != {"robot.urdf", "robot.xrdf"} or any(file_hash(model_dir / name) != digest for name, digest in expected.items()):
        raise ValueError("Model files changed since export; regenerate into a new output directory")
    if manifest["source_layer_sha256"] != reference["source_layer_sha256"] or manifest["isaacsim_version"] != isaacsim_version:
        raise ValueError("USD source layers or Isaac Sim version changed since model export")
    if manifest["base_frame"] != reference["base_frame"] or manifest["tool_frame"] != reference["tool_frame"]:
        raise ValueError("Model base/tool frame declarations do not match the USD reference")
    return manifest
