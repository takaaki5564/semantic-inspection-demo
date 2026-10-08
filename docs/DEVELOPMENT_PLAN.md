# 7-day staged plan (2026-10-08 to 2026-10-14)

Only work on one step at a time; ask for approval before proceeding.

## Day 1 — Verify existing runtime and capture two RGB views
1. Read existing working startup script and identify Isaac Sim version and Python invocation.
2. Launch unchanged and record successful baseline.
3. Add a minimal procedural part/target (or reuse an existing part if simpler) and a camera using APIs confirmed compatible with installed version.
4. Save `outputs/images/view_01.png`, move camera, save `outputs/images/view_02.png`.
5. Save `outputs/camera_poses.json` containing actual poses, frame/timestamp and coordinate-frame conventions.
Acceptance: two distinct images and corresponding metadata, repeatable via one documented command. No ontology/planner yet. If arm integration is problematic, move camera directly.

## Day 2 — Parts A/B and visibility
Generate low-rib A and high-rib B with target regions R1/R2; prove from the same initial view that B has an occluded subset of R2. Implement frustum + angle + ray-cast point visibility; unit test geometry and state transitions. Never hardcode coverage percentages.
Acceptance: changing geometry/camera pose changes computed visibility; an alternate manual view increases visible R2 points.

## Day 3 — Core knowledge and planner (Isaac-independent tests)
Define inspection specs X (R1) and Y (R1+R2), observation states, occlusion reason, `change_viewpoint` method and camera-motion capability. Add a minimal knowledge file and deterministic planner over discrete part-relative camera poses.
Acceptance: spec X/Y lead to different requirements; no motion capability returns blocked; no part-name-specific camera rules.

## Day 4 — Closed-loop integration
Connect plan -> simulator camera movement -> new RGB capture -> observation evaluator -> state update -> next plan. Avoid stale frames; enforce max actions and stop conditions.
Acceptance: Part B improves required observed surface samples with additional view; logs show actual executed steps.

## UR10e feasibility preflight — after Day 4, before Day 4.5

Day 1–4 are already implemented. Insert the proposed robot feasibility spike here rather than returning to Day 2. Target approximately 1–2 hours for the initial feasibility work, with the user reviewing each substep before proceeding.

1. **Load/display only (complete; user confirmed):** add an independent entrypoint using the existing environment and local UR10e USD. Keep the timeline stopped. Record six authored revolute joints and their limits, articulation root, flange transform, populated meshes including instance proxies, asset source and runtime. User confirms the GUI display and report. See `docs/ARM_PREFLIGHT_RUNBOOK.md`.
2. **Model consistency (complete; user confirmed):** generate UR10e URDF/XRDF from the chosen USD; compare cuMotion FK against raw USD joint frames across 15 cases and the authored zero pose. Verify joint names/limits, explicit base/flange frames and source/model hashes. Do not use the bundled `ur10` configuration. This is a numerical consistency check; physical readback is Step 3. See `docs/ARM_MODEL_RUNBOOK.md`.
3. **Controlled motion (complete; user approved proceeding):** independent `src/arm_motion_check.py` initializes the articulation and uses cuMotion IK plus smooth joint position targets for two illustrative flange goals. Gravity is enabled; actual PhysX joint/base/wrist readback determines flange convergence and settling. Check joint limits, command trajectory bounds, actual motion and simulation/wall-clock timeouts. No joint teleportation. Headless and GUI runs both pass; this does not establish collision safety or attach an inspection camera. See `docs/ARM_MOTION_RUNBOOK.md`.

Gate: two controlled arm motions and model consistency pass, and the user approved camera work. If feasibility cannot be maintained, continue the verified free-camera demo and leave further arm integration pending. Passing Step 1 alone does not pass this gate.

## Day 4.5 — Arm-camera integration (conditional on preflight)

Review checkpoints: **4.5a (current; implemented, user output review pending)** independently verifies a fixed flange camera, common camera-pose executor interface, pose conversion, two controlled motions and held-pose RGB/actual pose capture. See `docs/ARM_CAMERA_RUNBOOK.md`. **4.5b (pending)** connects the A/B inspection scene, reachability and moving-arm visibility. Closed-loop comparison is pending; the full Day 4.5 gate is not yet satisfied.

1. Introduce a common camera-pose executor interface while retaining the existing free-camera behavior and defaults.
2. Add UR10e execution and rigid flange-mounted RGB camera; define and verify `T_flange_camera`.
3. Convert requested camera poses to flange and robot-model frames; screen infeasible candidate views before planner selection.
4. Move through joint targets, wait for settling and acceptable camera pose error, then capture a fresh frame. Return measured poses and explicit blocked reasons.
5. Extend visibility to robot Mesh/instance geometry and distinguish static part/cell invariants from moving arm geometry. Update observations from the actual capture scene.
6. Compare free-camera and arm-camera runs with the same inspection scenarios and two camera goals.

Gate: actual camera mounting, controlled motion, fresh captures and observation updates pass; existing Day 1–4 behavior still works. If not, retain free-camera mode as the submission baseline.

## Day 5 — Simple display and recipe candidate
Add minimal controls for part A/B, spec X/Y, step/run/reset. Show current RGB, observed/missing regions, next action and reason. Save event log and recipe candidate. Optional synthetic line detection only if stable.
Add free-camera/UR10e mode selection only if Day 4.5 passes; include target/actual camera pose, controller status and blocked reason in saved evidence.
Acceptance: demo understandable without reading source; outputs persist.

## Day 6 — Test and compare
Run fixed-view baseline and planned-view approach on same A/B cases; test geometry changes, no available view, motion disabled, spec switch reset, part transform and replay determinism.
If Day 4.5 passes, also compare the two executors and test IK failure, unreachable goals, motion timeout, fixed camera mounting and pose errors. Collision safety and real-world robot performance remain unvalidated.
Acceptance: numerical comparison from logs, honest blocked cases, no hidden hardcoded success.

## Day 7 — Submission materials
Record 90–120s video and freeze source; summarize implemented vs mocked vs future work, compare results, and make architecture/roadmap figure. Target submission review on Oct 15; official deadline Oct 16 13:00 JST must be independently verified before submission.
Show the simulated arm and wrist camera only if Day 4.5 passes; otherwise present arm integration as future work.

## Execution protocol for each Codex step
1. Inspect only necessary files.
2. State intended edits, risks, tests.
3. Implement minimal changes.
4. Run test/launch if runtime available; otherwise state what was not run and provide user commands.
5. Report artifacts and wait for explicit go-ahead.
6. For arm work, wait for the user's output review after each substep. Commit locally; the user performs pushes.
