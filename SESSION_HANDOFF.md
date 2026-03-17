# Semantic Boxels Codebase — Session Handoff Prompt

Copy everything below into a new AI session.

---

## Project Overview

This is a **master thesis** codebase: "Semantic Boxels for Partially Observable Deterministic Task and Motion Planning" (RWTH Aachen, Hani Alassiri Alhabboub). It implements a PyBullet-based TAMP system where a Franka Panda robot must find and pick up a hidden target object by relocating occluders and sensing shadow regions.

**Stack:** Python 3, PyBullet (physics/rendering), PDDLStream (planning framework using Fast Downward), NumPy. Runs on Windows with WSL for the planner.

**Workspace:** `c:\Users\HaniAlassiriAlhabbou\git\Semantic_Boxels`

---

## File Map (root only — archive/ contains old versions)

| File | Lines | Purpose |
|------|-------|---------|
| `test_full_pipeline.py` | 833 | **Main entry point.** Replanning loop, action execution (move/sense/pick/place), belief state tracking, `execute_pick()`, `execute_place()`, `sense_shadow_raycasting()`, `compute_shadow_blockers()` |
| `boxel_env.py` | 699 | PyBullet environment. Scene setup (plane, table, Panda arm, occluders, targets). `SceneConfig`/`ObjectSpec` for parameterized scenes. Camera, shadow calc, free-space generation, oracle object detection |
| `streams.py` | 783 | PDDLStream geometric streams: `sample_grasp` (top-down only), `plan_motion` (direct path + RRT-Connect + smoothing), `compute_kin_solution` (multi-seed IK). `RobotConfig`, `Trajectory`, `Grasp` dataclasses |
| `pddlstream_planner.py` | 421 | PDDLStream interface. `PDDLStreamPlanner` builds init state (`_build_init`), connects streams, calls `solve(algorithm='adaptive')`. Exports debug PDDL |
| `robot_utils.py` | 375 | Panda constants (joint limits, rest poses, link indices). `RenderingLock`, `is_config_collision_free`, `is_path_collision_free`, `solve_ik`, `move_robot_smooth`, `open_gripper`, `close_gripper` |
| `boxel_data.py` | 348 | `BoxelData` dataclass, `BoxelRegistry` (stores all boxels, JSON serialization), `create_boxel_registry_from_boxels()` |
| `boxel_types.py` | 74 | Core types: `ObjectInfo`, `Boxel`, `OctreeNode`, `CameraObservation` |
| `shadow_calculator.py` | 304 | Shadow computation via PyBullet raycasting. Casts rays from camera through object back-face corners, computes shadow AABBs, splits around obstacles |
| `cell_merger.py` | 228 | Greedy merge of adjacent free-space boxels into larger convex regions |
| `free_space.py` | 193 | Octree-based free-space discretization (BFS subdivision) |
| `visualization.py` | 144 | `BoxelVisualizer` — wireframe + phantom rendering in PyBullet. **Intentionally disabled** (commented out call in test_full_pipeline.py line 146) |
| `run_logger.py` | 156 | Timestamped logging with stdout tee, verbosity control, artefact saving |
| `CODEBASE_AUDIT.txt` | ~1114 | Comprehensive issue tracker (94 issues). **Read this first** for full context |
| `archive/CODEBASE_AUDIT_RESOLVED.txt` | ~1091 | 50+ resolved issues showing evolution from fake stubs to real implementation |

### PDDL Files (`pddl/`)

| File | Purpose |
|------|---------|
| `domain_pddlstream.pddl` | Domain: 4 actions (sense, move, pick, place), 3 derived predicates (blocks_view, view_blocked, view_clear), Know-If fluents for partial observability |
| `stream.pddl` | 3 streams: sample-grasp, plan-motion, compute-kin |
| `problem_debug.pddl` | Auto-generated snapshot of one problem instance (for debugging) |

---

## How the Pipeline Works

### Phase 1-3: Scene Setup + Boxel Computation
1. PyBullet loads plane, table, Panda arm, occluders, targets
2. Oracle detection (raycasting to AABB corners) finds visible objects
3. Shadow calculator computes occluded regions behind objects
4. Octree subdivides remaining space into free-space cells
5. Cell merger combines adjacent free cells
6. `BoxelRegistry` stores everything, exports to JSON

### Phase 4: Hidden Object Setup
- Targets placed behind occluders by construction
- `compute_shadow_blockers()` raycasts from camera to find ALL objects blocking each shadow (not just the creating occluder)
- `BeliefState` tracks: unknown/empty/found for each shadow

### Phase 5: Planning + Execution (Replanning Loop)

```
while target not found and plan_count < max_replans:
    plan = PDDLStreamPlanner.plan(current_config, known_empty_shadows, moved_occluders)
    for action in plan:
        if action == 'move': follow trajectory waypoints
        if action == 'sense': move arm home, raycast, evaluate (found/empty/blocked)
        if action == 'pick': 6-step hardcoded sequence (approach/open/descend/close/weld/lift)
        if action == 'place': 6-step hardcoded sequence (approach/descend/open/release/settle/retreat)
        if sense fails or IK fails: break → replan with updated belief
```

**Key architectural decision:** The PDDL `sense` action is **optimistic** — it assumes the target is always found. When execution reveals otherwise, Python breaks and replans with updated knowledge. This is because PDDLStream + Fast Downward cannot do contingent planning (#61).

### What the PDDL Planner Decides vs What Python Hardcodes

| Decided by Fast Downward | Hardcoded in Python |
|---|---|
| Which occluder to move | How to physically pick (approach/grasp/lift choreography) |
| Which shadow to sense | How to physically place (approach/release/retreat) |
| Which free-space boxel for placement | When to replan (while loop + belief tracking) |
| Action ordering (move, pick, place, sense) | How to sense (move arm home, cast rays, tri-state eval) |
| Grasp selection (from stream) | Approach/lift heights (0.10m, 0.25m) |
| IK configuration (from stream) | State drift compensation (read actual joints after every action) |
| Collision-free trajectory (RRT from stream) | |

---

## PDDL Domain Summary

**Predicates:** `Boxel`, `Obj`, `Config`, `Trajectory`, `Grasp` (types); `is_shadow`, `is_object`, `is_free_space` (classification); `blocks_view_at` (static geometry); `blocks_view`, `view_blocked`, `view_clear` (derived — auto-update when objects move); `obj_at_boxel` (world state); `obj_at_boxel_KIF` (epistemic — Know-If); `at_config`, `handempty`, `holding`, `obj_pose_known` (robot state); `valid_grasp`, `motion`, `kin_solution`, `config_for_boxel` (stream-certified)

**Derived predicate chain:** `blocks_view_at(obj,b,region)` + `obj_at_boxel(obj,b)` → `blocks_view(obj,region)` → `view_blocked(region)` → NOT → `view_clear(region)`. Moving an object automatically unblocks views.

**Actions:**
- `sense(?o, ?region)`: requires `view_clear(?region)`, optimistically adds `obj_at_boxel`
- `move(?q1, ?q2, ?b, ?t)`: requires `motion(?q1,?q2,?t)` from plan-motion stream
- `pick(?o, ?b, ?g, ?q)`: requires `kin_solution` + `obj_at_boxel` + `obj_at_boxel_KIF`
- `place(?o, ?b, ?g, ?q)`: requires `is_free_space(?b)` + `kin_solution`

---

## Open Issues (from CODEBASE_AUDIT.txt)

### Thesis-Critical
- **#76** No evaluation framework — no metrics, no baselines, no batch runner. Blocks thesis Results section.
- **#55** Scene parameterization PARTIALLY DONE — SceneConfig/presets/CLI exist, but no batch evaluation wired up.
- **#81** No recursive object discovery (proposal Section 4.2 promised it).

### Architectural
- **#93** Actions not generic — sense embeds a hidden move; pick/place embed 3 moves + 3 IK solves + gripper control + constraint manipulation. Planner can't reason about these sub-actions.
- **#59/#77** Constraint-based grasping (magic weld, objects fly away). Occluders too big for Panda gripper (0.15m > 0.08m max opening). Friction grasping blocked until objects resized.

### Deferrable
- **#44** Stale shadow registry after occluder relocation (compensated by body-ID raycasting)
- **#73** Stale shadow_occluder_map on replan (compensated by derived predicates)

### Dead Code (preserved by policy — NEVER delete commented-out code)
- **#87** ~160 lines of dead methods across 5 files (compute_neighbors, depth methods, direction_to_quat, etc.)
- **#88** 7 unused imports
- **#89** Unused `config` parameter in execute_pick/execute_place
- **#90** BoxelVisualizer intentionally disabled
- **#91** Dead CameraObservation fields
- **#92** RunLogger context manager unused
- **#94** Dead heuristic config rejection loop (lines 300-308)

### Resolved (11 [DONE] items verified in code — all genuine)
#60 RenderingLock, #67 observed_clear_regions, #69/#84 magic numbers commented, #78 compute_shadow_blockers, #79 arm-to-home before sense, #80 heuristic IK removed, #82 IK failure abort, #83 robot-arm raycasting, #85 dead push code removed, #86 actual joint state tracking

---

## Codebase Policy

1. **NEVER delete commented-out code.** It's preserved intentionally for debugging and reference. Move to archive/ if truly obsolete.
2. **Dead code is preserved.** Issues #87-#94 track it for awareness, not for cleanup.
3. **Shell is PowerShell** (not bash). No heredocs, no `&&` chaining. See `.cursor/rules/powershell-shell.mdc`.
4. **Run commands via WSL:**
   ```
   wsl bash -lc "cd /mnt/c/Users/HaniAlassiriAlhabbou/git/Semantic_Boxels && source wsl_env/bin/activate && export PYTHONPATH=/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib && python3 test_full_pipeline.py --no-gui"
   ```

---

## Key Execution Details

- **IK:** Multi-seed null-space IK via PyBullet (`calculateInverseKinematics`). 8 seeds (REST_POSES + 7 perturbations). Save/reset/restore pattern because PyBullet has only one robot state.
- **Motion planning:** Direct linear path first → if blocked, bidirectional RRT-Connect (2000 max iters, 0.2 rad step) → shortcut smoothing (75 attempts).
- **Collision checking:** `resetJointState` to test config → `performCollisionDetection` → `getContactPoints` → restore. Thousands of times per plan. `RenderingLock` prevents flickering.
- **Grasping:** Top-down only, 3 Z-offsets (0.05/0.10/0.15m). Constraint-based weld (`p.createConstraint JOINT_FIXED`), not friction-based.
- **State tracking:** After every move/pick/place, reads actual joint angles via `p.getJointState` and stores in `current_config`. Passed to planner on next replan to avoid drift.
- **Belief propagation:** `known_empty_shadows` accumulates across replans. `moved_occluders` dict tracks where each occluder was placed. Both fed to `_build_init()` to update PDDL init state.
