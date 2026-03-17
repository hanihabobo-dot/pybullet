[Back to Home](Home)

# Known Issues and Roadmap

## Overview

This page tracks the current state of the codebase audit, summarizes resolved issues, lists open issues by priority tier, documents proposal alignment gaps, and outlines future work. The canonical source for issue details is `CODEBASE_AUDIT.txt` in the repository root; this page provides a navigable overview with cross-references to the relevant wiki pages.

---

## Resolved Issues

The following issues have been fixed and archived to `archive/CODEBASE_AUDIT_RESOLVED.txt`. They are listed here for completeness.

### Tier 1 -- Quick Wins (all completed 2026-03-14)

| Issue | Description | Resolution |
|-------|-------------|------------|
| #80 | Fake IK fallback (`_heuristic_ik`) allows garbage data | Removed entirely. `BoxelStreams` raises `ValueError` if `robot_id` is `None`. |
| #83 | Raycasting blind spot -- rays hitting robot arm counted as empty | `sense_shadow_raycasting()` now accepts `robot_id`; robot-arm hits produce `still_blocked`. |
| #84 | Incomplete magic number cleanup in execution code | Added justification comments for approach/lift heights. |
| #85 | Dead/commented push code polluting codebase | Removed `sample_push_config`, push action, push stream. |

### Tier 2 -- High Value Fixes (all completed 2026-03-14)

| Issue | Description | Resolution |
|-------|-------------|------------|
| #60 | Simulation-wide flickering during planning | Reference-counted `RenderingLock` in `robot_utils.py`. |
| #67 | `_build_init()` hardcodes targets-only-in-shadows assumption | Added optional `observed_clear_regions` parameter. |
| #69 | ~30 magic numbers without justification | Justification comments added to all constants. |
| #78 | Incomplete view-blocking model -- pipeline loops forever | `compute_shadow_blockers()` finds ALL objects blocking each shadow. |
| #79 | Robot arm blocks sensing during execution | Sense handler moves arm to home before raycasting. |
| #82 | Silent IK failures in execution | `execute_pick()`/`execute_place()` abort and trigger replan on IK failure. |
| #86 | PDDL state drift after pick/place | Execution reads actual PyBullet joint state after every action. |

### Earlier Resolved Issues

| Issue | Description |
|-------|-------------|
| #1 | Oracle hiding replaced with AABB containment check |
| #2 | String-comparison sensing replaced with PyBullet raycasting |
| #5 | Linear interpolation replaced with RRT-Connect motion planning |
| #7 | Pick/place now use plan grasp + execution-time IK |
| #20 | `BoxelStreams` wired into `PDDLStreamPlanner` stream map |
| #49 | Stream IK failures on replan fixed (seed reset + null-space) |
| #52 | Move execution uses plan trajectory waypoints |
| #53 | Push replaced with pick-and-place |
| #54 | Oracle visibility uses 8-ray AABB check |
| #57 | Pick configs collide with grasped object -- added `ignored_body_ids` |
| #58 | Pipeline logging with `RunLogger` |

---

## Open Issues by Priority Tier

### Tier 3 -- Thesis-Critical

These issues directly block the thesis evaluation and results sections.

| Issue | Description | Effort | Documented In |
|-------|-------------|--------|---------------|
| #55 | **Hardcoded scene -- no scenario variation.** Scene parameterization infrastructure exists (`SceneConfig`, presets, CLI flags) but no batch evaluation runner uses it. | Partially done | [Scene Environment](Scene_Environment) |
| #76 | **No evaluation framework (PA-1).** No success rate measurement, no planning time collection, no baselines, no experiment runner. The thesis cannot have a Results section without this. | ~1-2 weeks | See PA-1 below |
| #81 | **Missing recursive object discovery (PA-2).** The proposal claims free space partitioning will recursively discover new objects. The implementation is a single-pass octree. | ~2-3 days or document as future work | [Spatial Reasoning](Spatial_Reasoning) |

### Tier 3b -- Architectural

| Issue | Description | Effort | Documented In |
|-------|-------------|--------|---------------|
| #93 | **Actions are not generic -- hidden sub-actions in execution.** `sense` contains a hidden move; `pick` contains 3 hidden moves + 3 IK solves + gripper control + constraint weld; `place` same pattern. The planner cannot reason about these sub-actions. | ~1-2 weeks | [Design Decisions](Design_Decisions), [Execution Pipeline](Execution_Pipeline) |

### Tier 4 -- Deferrable

These are accepted simplifications or fidelity improvements.

| Issue | Description | Status | Documented In |
|-------|-------------|--------|---------------|
| #44 | Shadows not recomputed after occluder relocation -- stale registry. | Accepted. Raycasting by body ID compensates. | [Spatial Reasoning](Spatial_Reasoning) |
| #73 | `shadow_occluder_map` goes stale on replan. | Low impact. Derived predicates compensate. | [Planning System](Planning_System) |
| #59 | Objects fly away on first grasp -- constraint-based grasping. | Accepted. Friction grasping is a fidelity improvement, not correctness fix. | [Design Decisions](Design_Decisions) |
| #77 | Object dimensions exceed gripper capacity (0.15 m cubes vs 0.08 m max opening). | Blocks #59. Fix together if pursued. | [Design Decisions](Design_Decisions) |

### Tier 5 -- Dead Code Inventory

Documented and preserved per codebase policy. No action needed.

| Issue | Description | Lines |
|-------|-------------|-------|
| #87 | Dead methods/properties across codebase | ~160 lines |
| #88 | Unused imports (7 statements across 3 files) | ~7 lines |
| #89 | Unused function parameter (`config` in execute_pick/place) | 2 params |
| #90 | `BoxelVisualizer` instantiated but never called (intentionally disabled) | ~145 lines |
| #91 | Dead `CameraObservation` fields (rgb_image, depth_image, point_cloud always None) | 3 fields |
| #92 | `RunLogger` context manager protocol never used | ~5 lines |
| #94 | Dead heuristic config rejection loop (unreachable since #80) | ~8 lines |

---

## Proposal Alignment Gaps

These are features described in the thesis proposal that are missing, simplified, or deviate from the proposal's design.

| ID | Gap | Proposal Section | Status | Documented In |
|----|-----|-----------------|--------|---------------|
| PA-1 | **No evaluation framework.** No metrics, baselines, batch runner, or scene variation. | Section 5 | MISSING -- CRITICAL (#76) | Above |
| PA-2 | **No recursive object discovery.** Free space partitioning is a single-pass octree. | Section 4.2 | MISSING | [Spatial Reasoning](Spatial_Reasoning) |
| PA-3 | **KIF design differs.** Single predicate instead of two-predicate `K(p)` / `K(not p)`. | Section 4.4.1 | Documented deviation | [Design Decisions](Design_Decisions) |
| PA-4 | **Fixed camera for sensing.** Proposal uses robot-mounted sensor with `stream_find_sensing_config`. | Section 4.4.2 | Accepted simplification | [Design Decisions](Design_Decisions) |
| PA-5 | **No conditional sensing outcomes.** Proposal uses `when found/not_found` branches. | Section 4.4.2 | Accepted deviation (#61) | [Design Decisions](Design_Decisions) |
| PA-6 | **Only top-down grasps.** No lateral or angled grasps. Limits evaluation scenarios. | Implicit | Known limitation | [Robot Control and Streams](Robot_Control_and_Streams) |

---

## Future Work

### Evaluation Framework (PA-1, #76)

The highest-priority missing component. Required deliverables:

- **Batch evaluation runner**: Execute the pipeline across multiple scene configurations with different random seeds.
- **Metric collection**: Success rate, planning time per plan, total plan cost (number of actions), number of replans.
- **Scene variation**: Use the existing `scalability_scene()` preset with varying `n_occluders` and `n_targets`.
- **Baselines**: At minimum, compare against uniform voxelization (fixed-resolution grid) and fixed semantic regions (hand-defined zones).

### Recursive Object Discovery (PA-2, #81)

The proposal describes a recursive process: after free-space partitioning, if new objects are found within a partition, the process repeats. Implementing this would require:

1. A perception step inside `FreeSpaceGenerator` to detect objects within free cells.
2. Re-running shadow calculation for newly detected objects.
3. Re-running free-space generation with the updated boxel set.

Alternatively, document this as an accepted simplification for the single-camera tabletop scenario where all objects are detected upfront.

### Friction-Based Grasping (#59, #77)

Replacing constraint-based grasping with friction-based grasping requires:

1. Resizing objects to fit in the Panda's 0.08 m gripper opening (#77).
2. Adjusting grasp Z offsets so fingers contact the object sides.
3. Setting appropriate `lateralFriction` on objects and gripper shapes.
4. Testing grip stability across different object shapes.

### Grasp Diversity (PA-6)

Extending `sample_grasp()` to produce lateral and angled grasps would enable handling scenes with:

- Objects near walls or edges.
- Stacked objects.
- Objects in narrow gaps.

### Incremental Boxel Recomputation

After relocating an occluder, only the affected shadows and free-space cells need recomputation. Currently the system uses a stale registry (#44). Incremental updates would improve accuracy without the cost of full recomputation.

### Action Decomposition (#93)

Decomposing pick/place into sub-actions in the PDDL domain (approach, grasp, lift as separate actions with their own streams) would allow the planner to:

- Optimize approach/retreat paths.
- Detect collisions on approach waypoints.
- Reason about post-action configurations.

The trade-off is 3x more stream evaluations and a more complex domain.

---

**See Also:**
- [Design Decisions](Design_Decisions) -- Rationale for the accepted deviations listed here.
- [Planning System](Planning_System) -- The replanning architecture that compensates for some of these limitations.
- [Scene Environment](Scene_Environment) -- Scene parameterization infrastructure (#55).
- [Architecture Overview](Architecture_Overview) -- Overall system structure.

---

[Back to Home](Home)
