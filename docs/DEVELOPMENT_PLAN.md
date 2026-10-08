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

## Day 5 — Simple display and recipe candidate
Add minimal controls for part A/B, spec X/Y, step/run/reset. Show current RGB, observed/missing regions, next action and reason. Save event log and recipe candidate. Optional synthetic line detection only if stable.
Acceptance: demo understandable without reading source; outputs persist.

## Day 6 — Test and compare
Run fixed-view baseline and planned-view approach on same A/B cases; test geometry changes, no available view, motion disabled, spec switch reset, part transform and replay determinism.
Acceptance: numerical comparison from logs, honest blocked cases, no hidden hardcoded success.

## Day 7 — Submission materials
Record 90–120s video and freeze source; summarize implemented vs mocked vs future work, compare results, and make architecture/roadmap figure. Target submission review on Oct 15; official deadline Oct 16 13:00 JST must be independently verified before submission.

## Execution protocol for each Codex step
1. Inspect only necessary files.
2. State intended edits, risks, tests.
3. Implement minimal changes.
4. Run test/launch if runtime available; otherwise state what was not run and provide user commands.
5. Report artifacts and wait for explicit go-ahead.
