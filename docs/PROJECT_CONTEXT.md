# NEDO Semantic Inspection Robot — project context

## Business objective
Build an A1 competition demonstration for an industrial robot inspection solution suitable for changing product variants. The value proposition is to reduce expert teaching/configuration effort when a new metal component is introduced. The 7-day prototype demonstrates how inspection requirements and observations drive additional camera views, rather than claiming a fully autonomous factory inspection system.

## Hypothesized user problem
Complex metal components with ribs/recesses have inspection target surfaces occluded from a fixed camera viewpoint. A new variant can require additional viewpoints. This is a hypothesis based on publicly described industrial inspection difficulties, not a claim established through interviews with a specific factory.

## Demonstration narrative
1. Part A: a simple metal-like plate with a low rib; required regions R1 and R2 can be observed from the initial camera view.
2. Part B: related geometry with a higher rib; the same initial view leaves part of R2 occluded.
3. The system identifies which required surface points are not visible. It distinguishes `captured` from `inspection_observation_satisfied`.
4. The knowledge layer maps `self_occlusion` to an eligible `change_viewpoint` inspection method and checks whether the device offers camera pose change capability.
5. The planner evaluates a small set of camera poses in the part coordinate frame and selects a pose predicted to expose more of the required unobserved surface.
6. Isaac Sim moves the verified free camera, captures a new frame, and updates the state based on actual geometry/observations. An optional UR10e eye-in-hand executor will reproduce the same camera goals after an independent robot feasibility check; evaluation uses the actual wrist-mounted camera pose and fresh image.
7. Switch inspection spec X (R1 only) vs Y (R1 + R2). The same part geometry must lead to different additional-view decisions.
8. Save event log and a candidate inspection recipe; do not call it approved or production-ready.

## Scope
- Ubuntu 24.04, NVIDIA GeForce RTX 4080; Isaac Sim already starts via Python from an existing working project. Installed Isaac Sim version, scripts, Python environment, robot/camera availability must be discovered from the repository and local runtime.
- Scene: procedural primitives (plate, walls/rib) with independently identifiable inspection regions; fixed light; virtual RGB camera.
- Optional arm extension: a fixed inspection part and a UR10e carrying an RGB camera on its flange. The arm moves the camera; it does not grasp, transfer or sort parts. Preserve the free-camera entrypoints and defaults.
- Geometry and region IDs are simulator-provided ground truth. No camera-image-based shape recognition or 6DoF part pose estimation in this prototype.
- Visibility: sample points on each required surface, camera frustum/angle check and line-of-sight ray casting against scene geometry. Ensure ray hits correspond to the intended surface, not merely any mesh of the same part.
- Candidate viewpoints: a small discrete set, ranked by predicted increase in observed required points and motion cost. No continuous optimization.
- Optional: synthetic line texture and simple image-based candidate extraction. This is not an industrial scratch detector.
- Required outputs: two distinct camera images with recorded poses, dynamic simulation, per-region observed/missing state, action rationale, event log, recipe candidate, short demo video and evaluation results.

## Logical separation
- Simulator/scene adapter: scene creation, camera pose, RGB acquisition, geometry/raycast interface.
- Camera motion interface: separate free-camera and arm-mounted executors accept a camera goal and return actual pose, execution status and capture metadata. The planner remains independent of robot joint control.
- Arm executor: camera-to-flange conversion, robot-base/tool-frame conversion, IK and joint-limit screening, controlled motion, settling and timeout checks. Filter infeasible camera candidates before selection; report explicit blocked reasons.
- Observation evaluator: visibility, coverage of predefined sample points, missing information, status.
- Knowledge layer: inspection requirement, surface region, missing observation cause, inspection method, equipment capability; use RDFLib/Turtle if suitable after the initial camera/geometry steps.
- Planner: select eligible actions and candidate viewpoints, return explicit reason.
- State/event store: current inspection session and immutable events; save recipe candidate with part geometry version, inspection spec version, coordinate frame, evidence and status.

## Coordinate and evidence requirements
Use explicit transform conventions such as `T_world_part` and `T_part_camera`, with `T_world_camera = T_world_part @ T_part_camera`. Document units (meters), axes, quaternion convention and camera optical frame. Log image timestamp/frame number and actual camera pose. Synthetic ground truth must never masquerade as image recognition output.

For the arm extension, define a constant `T_flange_camera` and compute `T_world_flange_target = T_world_camera_target @ inverse(T_flange_camera)`. Convert into the robot model's base and tool frames explicitly. Log `T_world_part`, target camera pose, flange pose, fixed mounting transform, actual camera pose, pose errors and timestamps. Do not equate a successful IK solution with a completed motion or a safe path.

## Current progress and arm preflight
- Day 1–4 free-camera implementation and saved results are available. Day 5 UI/recipe work has not started.
- Detected runtime: existing `env_isaac`, Python 3.12.3, Isaac Sim 6.1.0.0. The installed motion stack includes cuMotion; bundled legacy Lula APIs are deprecated.
- A local UR10e USD is available inside `isaacsim.asset.transformer.rules/data/tests/ur10e/ur10e.usd`. This is a bundled test asset, not a verified production robot configuration. The bundled cuMotion `ur10` description differs from this UR10e and must not be silently reused.
- Arm work starts after Day 4, in separately reviewed substeps. Step 1 only composes/displays the local robot and records USD joints/flange/meshes with the timeline stopped. Physics initialization, IK, controlled motion and camera mounting are subsequent steps, not Step 1 claims. See `docs/ARM_PREFLIGHT_RUNBOOK.md`.
- Step 1 automated headless check passed on 2026-10-08 (6 joints, 14 populated meshes, expected flange, timeline 0 seconds). User GUI confirmation is pending; subsequent arm steps have not started.

## Explicitly deferred
Camera-based shape/pose estimation, CAD registration, real-metal optics/reflection/oil, scratch-vs-machining-mark classification, depth measurement, grasp/regrasp, robot safety validation, full cycle-time guarantee, expert approval workflow, production-grade defect detection, LLM-driven ontology generation and RL.

## Future extension story
The same requirement -> missing evidence -> eligible action -> new observation loop can later support lighting changes, additional measurements, regrasping, uncertain pose active perception, human escalation, validated inspection recipes and real-world robot control. Each requires separate technical validation.

## Evaluation
Compare a fixed Part-A viewpoint on Part B versus planned additional viewpoints, using the same visibility evaluator. Measure observed required surface sample points, number of captures, plan runtime, whether spec X avoids unnecessary R2 captures, whether no camera motion capability produces `blocked`, and whether all candidates failing results in `blocked`. Report simulator results separately from real-world detection accuracy.
