# Codex development rules — NEDO Semantic Inspection Demo

Read `docs/PROJECT_CONTEXT.md` and `docs/DEVELOPMENT_PLAN.md` before proposing or changing code. First inspect the repository, existing entrypoint, installed Isaac Sim version and working examples. Never assume paths, versions or APIs.

## Working method
- Implement only the currently approved step. Do not advance to another step automatically.
- Before changes: report current state, files to change, assumptions, acceptance tests, and a short implementation plan.
- Prefer small, reversible changes. Preserve the existing working Python-to-Isaac-Sim startup flow. Do not rewrite unrelated code.
- Separate simulator adapter, geometry/visibility, inspection state, knowledge retrieval, planner and UI. The core must be testable without Isaac Sim.
- For region visualization, first implement the right-side list of region ID/name, evaluator-owned observation state, cumulative coverage and next capture target. Then prefer one display USD Prim per region for 3D colors; use a simple fixed-view schematic only if 3D/RGB isolation cannot be completed promptly. Do not implement both visual approaches by default. See `docs/REGION_VISUALIZATION_DESIGN.md`.
- GUI and region visualizers consume shared evaluator/state outputs; they must not decide observation validity, fabricate coverage or mark products defect-free. Keep per-region states distinct from existing overall session states and display non-required regions explicitly.
- Visualization colors/geometry must not contaminate either executor's inspection RGB, ray-cast geometry, lighting or physical behavior. Verify isolation before adopting 3D colors; otherwise use the GUI schematic fallback. Preserve original part materials and the verified camera/robot/planner flow.
- Preserve the verified free-camera mode when adding the optional UR10e eye-in-hand mode. Use separate `FreeCameraExecutor` and `ArmMountedCameraExecutor` implementations behind a common camera-pose interface. The planner emits camera target poses, not joint angles.
- Verify robot APIs against the installed Isaac Sim version. Do not assume the bundled UR10 model matches UR10e; validate USD/URDF/XRDF forward kinematics before IK or arm integration. Do not upgrade Isaac Sim.
- In arm mode, define and log `T_flange_camera`, requested camera/flange poses and actual poses. Never assume wrist, flange, tool and camera optical frames are identical.
- Reject infeasible goals and report unreached targets as `blocked`. Use supported obstacle avoidance where available; motion generation does not establish collision safety.
- Do not move the camera independently in arm mode or teleport robot joints to demonstrate target motion. Capture after convergence and settling, using a fresh image and measured camera pose.
- The user reviews each arm substep's output before the next substep. Commit completed changes locally; do not push, because the user performs pushes.
- Keep deterministic behavior, source/approval status, coordinate frames, units and event history explicit.
- No hardcoded `part_B -> camera_pose_X` decisions. No direct access to hidden defect labels by the planner.
- Distinguish simulated ground truth, derived observations, mock values and image-based estimates in output and UI.
- Do not claim real-world defect-detection accuracy, 6DoF pose estimation, collision safety or production readiness unless actually implemented and tested.
- No LLM, ROS 2, cloud services, Neo4j, training pipeline or new major dependencies without explicit approval.
- Add or update tests for every new core behavior. Run relevant tests and report commands, results and untested portions.
- If an Isaac Sim API is uncertain, inspect the installed version and existing sample before implementation; do not invent APIs.
- Do not install packages, update Isaac Sim, delete files, or perform destructive actions without permission.
- End each step with: changed files, how to run, observed output, acceptance criteria status, known limitations, and the proposed next step. Wait for user confirmation.

## Required safety and correctness invariants
- Captured != inspected; visible != defect-free; synthetic line candidate != production OK/NG decision.
- The set of required inspection regions comes from the active inspection specification, not from the part name.
- A new part or inspection specification resets incompatible observation state.
- If no valid action exists, return `blocked` with a reason; never loop indefinitely or fabricate success.
- Plan in a part-relative frame; convert to world/camera frame using explicit transforms.
