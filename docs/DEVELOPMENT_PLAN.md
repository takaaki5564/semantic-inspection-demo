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

Review checkpoints:

- **4.5a (complete; user approved proceeding)** independently verifies a fixed flange camera, common camera-pose executor interface, pose conversion, two controlled motions and held-pose RGB/actual pose capture. See `docs/ARM_CAMERA_RUNBOOK.md`.
- **4.5b (complete; user approved proceeding)** independently connects A/B inspection geometry, IK candidate screening and moving-arm visibility. Both parts share a fixed cell placement and the existing part-relative camera candidates. Actual captures update observation state using visible robot instance Meshes; static geometry and moving-scene hashes are checked separately. Two explicit front/rear diagnostic views pass for B/Y headless and A/Y GUI. Infeasible requested views stop before motion/capture. See `docs/ARM_INSPECTION_RUNBOOK.md`.
- **4.5c (current; implemented, user output review pending)** connects reachability and the camera executors to closed-loop planner selection. Candidate-specific robot geometry comes from validated FK at each IK solution, without moving the simulator; actual captures alone update observations. Free/arm modes share the cell, optics, part/spec and front/rear goals. B/Y selects rear and reaches 126/126 required points with two captures; A/Y and B/X stop after the initial capture. Unreachable additional candidates and unavailable motion capability return blocked. RGB renderer poses/optics and saved visibility/plans are verified. See `docs/ARM_CLOSED_LOOP_RUNBOOK.md` for commands, evidence and limitations. The full Day 4.5 gate awaits user output review; do not start Day 5 automatically.

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

### Region visualization amendment — nedo-v3

All three proposal documents from `/home/ishii/Downloads/nedo-v3` have been reviewed. See `docs/REGION_VISUALIZATION_DESIGN.md` for responsibilities, state semantics, proposed modules, isolation and acceptance checks. Design/plan updates are complete; visualization implementation has not started. The user's latest instruction schedules it after the preceding implementation completes. The exact insertion point (immediately after Day 4.5c versus after the basic Day 5 UI/recipe implementation) is being clarified; do not silently mark either prerequisite complete.

The proposal's Day 2–4 items are requirements to reconcile with current code, not a request to restart completed days. Region IDs, separate R1/R2 marker Prims, sample points, cumulative coverage and actual free/arm observation events already exist. Region names, explicit per-region display states and the GUI list remain to be added. Existing overall session statuses and planner decisions retain their meanings.

1. **Step 1 — mandatory, first visualization implementation:** add evaluator-owned `InspectionRegionState` data and a right-side list of region ID/name, state, cumulative coverage and next capture target. Reuse the current valid-observation history and actual planner output. Include `not_required` for R2 under spec X. Check A/Y, B/Y, B/X, reset and blocked behavior in both executors, then review the output before Step 2.
2. **Step 2 — preferred visual approach:** use one display USD Prim per region and shared state colors (gray/yellow/green/red). Verify separation from inspection RGB, shadows, ray geometry and physics before adopting it. Next-target highlighting is presentation only. Keep existing inspection materials and semantic region mappings.
3. **Step 3 — fallback only:** if Step 2 isolation is difficult or takes too long, keep the mandatory list and show a fixed-view schematic with the same state/colors. Completing both Step 2 and Step 3 is unnecessary.

Do not add arbitrary mesh segmentation, complex face-material assignment, high-resolution heatmaps, advanced transparency or a separate visualization application. Preserve the existing camera executors, evaluator and planner design. Visualization work must not delay reliable capture/motion/replanning or the submission demo. The basic Day 5 controls and recipe candidate remain required in the overall plan, with their ordering resolved against the user's preceding-implementation instruction.

## Day 6 — Test and compare
Run fixed-view baseline and planned-view approach on same A/B cases; test geometry changes, no available view, motion disabled, spec switch reset, part transform and replay determinism.
If Day 4.5 passes, also compare the two executors and test IK failure, unreachable goals, motion timeout, fixed camera mounting and pose errors. Collision safety and real-world robot performance remain unvalidated.
For region visualization, check A/B and X/Y state changes, cumulative coverage, GUI/3D-or-schematic agreement, arm-motion/capture updates, and inspection RGB/geometry isolation. Use one evaluator/state snapshot for all displays; never hardcode display success or coverage.
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
