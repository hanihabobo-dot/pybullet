[Back to Home](Home)

# Design Decisions

## Overview

This page documents the key architectural decisions in the Semantic Boxels system, their rationale, and where they deviate from the original thesis proposal. Each decision is grounded in practical constraints (PDDLStream limitations, simulation fidelity, development time) and in established patterns from the TAMP literature. Understanding these decisions is essential for evaluating what the system can and cannot do, and for planning future extensions.

---

## Semantic Boxels vs. Traditional Voxels

**Decision:** Discretize the workspace into task-relevant semantic regions (boxels) rather than a uniform voxel grid.

**Rationale:**

A uniform voxel grid treats all space equally -- every cell has the same size and no semantic meaning. The planner would need to reason over thousands of cells, most of which are irrelevant. Semantic boxels are different:

- **Object boxels** tightly bound detected objects -- their size varies with the object.
- **Shadow boxels** cover exactly the occluded regions -- their shape is determined by camera geometry.
- **Free-space boxels** are merged into large convex regions -- a single boxel can represent an entire empty area.

This produces a compact representation (~57 boxels for the default scene vs. thousands of uniform voxels) that the PDDL planner can reason over efficiently. Each boxel carries semantic meaning: the planner knows that a shadow is where targets might hide, a free-space region is where objects can be placed, and an object boxel is something that can be grasped.

**Trade-off:** The boxel decomposition is view-dependent (shadows depend on camera position) and assumes axis-aligned bounding boxes. Non-convex or oddly-shaped occlusion patterns are over-approximated.

---

## Optimistic Sensing with Reactive Replanning

**Decision:** The PDDL `sense` action optimistically assumes the target will be found. When execution reveals otherwise, the system replans with updated belief.

**Proposal deviation:** The thesis proposal (Section 4.4.2) defines sense with conditional effects: `stream_get_sensing_outcome` returns `found` or `not_found`, and `(when ...)` branches set different predicates. This would let the planner generate multi-step search plans within a single plan.

**Why the deviation:**

1. **PDDLStream + FastDownward cannot do contingent planning.** Conditional effects in PDDL require deterministic outcomes. Branching on observation results requires a contingent planner (POND, CLG), which is outside PDDLStream's architecture.

2. **Optimistic planning with replanning on failure is a standard TAMP pattern** under partial observability (Garrett et al., 2020; Kaelbling & Lozano-Perez, 2013). PDDLStream's own adaptive algorithm is built around optimistic assumptions.

3. **Convergence is guaranteed.** For N shadow candidates, the reactive approach converges in at most N replan cycles. Each replan searches strictly fewer candidates because `known_empty_shadows` grows monotonically.

4. **Belief propagation is correct.** Known-empty shadows and moved occluders carry over across replans. Each plan receives a strictly more informed world model than the previous one.

**Limitation:** The planner cannot optimize sensing ORDER -- it picks whichever shadow FastDownward expands first. A contingent planner could reason about most-likely-first strategies. For the current uniform-prior scenario this does not affect completeness.

**Audit reference:** #61, PA-5, PF-1.

---

## Pick-and-Place over Push

**Decision:** Occluder relocation uses pick-and-place (pick up the object, carry it to free space, place it) rather than push (apply force to slide the object).

**History:** The system originally implemented push as teleportation (`p.resetBasePositionAndOrientation()`). This caused multiple problems:

| Problem | Description |
|---------|-------------|
| No robot contact | The arm never touched the object -- it was teleported. |
| Direction heuristic | Push direction was computed from camera geometry, not physics. |
| Partial displacement | Push distance was a heuristic; insufficient pushes left shadows still blocked. |
| State desync | The planner's symbolic state diverged from the physical state after failed pushes. |
| Non-determinism | Physics settling after teleportation was unpredictable. |

Pick-and-place solves all of these: the robot physically grasps the object, the planner computes the destination (a free-space boxel), and the placement is deterministic. The trade-off is that objects must fit in the gripper, which is not currently the case for the 0.15 m occluder cubes (see [Known Issues and Roadmap](Known_Issues_and_Roadmap), #77).

**Audit reference:** #53 (replacement), #50/#51 (superseded push issues).

---

## Fixed Camera for Sensing

**Decision:** Sensing uses the fixed overhead scene camera rather than a robot-mounted sensor.

**Proposal deviation:** The proposal's `sense` action uses `stream_find_sensing_config` to compute a robot configuration that positions a sensor for observation. This requires a separate sensing motion planner.

**Rationale:**

- The fixed overhead camera is sufficient for the tabletop scenario: it can see every shadow region once the blocking occluder is removed.
- A robot-mounted sensor would add complexity: the planner would need to reason about sensor positioning, the robot might occlude the shadow while trying to observe it, and the motion planner would need to find collision-free sensing configurations.
- The simplification is well-scoped: extending to a robot-mounted sensor would require adding a `sensing_config` stream and a `(at_config ?q_sense)` precondition to the sense action. The infrastructure for this exists in the archive.

**Audit reference:** PA-4, #36.

---

## KIF Simplification

**Decision:** Use a single `obj_at_boxel_KIF` predicate instead of the proposal's two-predicate design (`obj_in_Boxel` / `obj_not_in_Boxel`).

**Proposal deviation:** The proposal (Section 4.4.1) defines two predicates: `K(p)` = "we know the object IS here" and `K(not p)` = "we know the object is NOT here". The absence of both means "we don't know."

**Implementation:** A single `obj_at_boxel_KIF` predicate:
- **Present** means "we know" (either positively or negatively, depending on whether `obj_at_boxel` is also present).
- **Absent** means "we don't know."

This is a valid Know-If simplification (Bonet & Geffner style): the KIF predicate tracks whether the truth value has been observed, and the ground-truth `obj_at_boxel` tracks the actual value. Together they encode the same three states:

| `obj_at_boxel_KIF` | `obj_at_boxel` | Meaning |
|--------------------|----------------|---------|
| Absent | Absent | Unknown |
| Present | Absent | Known NOT here |
| Present | Present | Known HERE |

**Audit reference:** PA-3.

---

## Constraint-Based Grasping

**Decision:** Grasp attachment uses `p.createConstraint(JOINT_FIXED)` rather than friction-based finger contact.

**Rationale:**

Constraint-based grasping "welds" the object to the end-effector, bypassing contact physics. This is a common simulation simplification in TAMP research:

- It works regardless of object size, shape, or weight.
- It is deterministic -- no risk of the object slipping.
- It avoids the need for precise finger-object contact modeling.

**Trade-off:** The constraint produces visual artifacts:

| Artifact | Cause |
|----------|-------|
| Objects snap to gripper | The constraint teleports the object to the attachment point. |
| Neighboring objects fly away | The constraint solver fights the rigid-body simulator, producing large corrective impulses. |
| Gripper stays open | The constraint holds the object even without finger contact. |

Switching to friction-based grasping would require:
1. Resizing objects to fit in the Panda's 0.08 m max finger opening (#77).
2. Setting appropriate lateral friction coefficients.
3. Adjusting grasp Z offsets so fingers contact the object (not the air above it).

**Audit reference:** #59, #77.

---

## Hidden Sub-Actions in Execution

**Decision:** The Python execution handlers for `pick`, `place`, and `sense` contain hidden sub-actions (moves, IK solves, gripper operations) that the PDDL planner cannot see.

**What is hidden:**

| PDDL Action | Hidden Sub-Actions |
|-------------|-------------------|
| `sense` | 1 move (arm to home) |
| `pick` | 3 moves (approach, contact, lift), 3 IK solves, 1 open_gripper, 1 close_gripper, 1 createConstraint |
| `place` | 3 moves (approach, lower, retreat), 3 IK solves, 1 open_gripper, 1 removeConstraint, 30 physics steps |

**Consequences:**
- The planner cannot optimize approach/retreat motions or detect collisions on them.
- Execution-time IK solves can fail (handled by abort-and-replan, #82).
- The hidden move in `sense` causes PDDL state drift (compensated by `current_config` tracking, #86).

**Why this is acceptable:**
- Decomposing pick/place into sub-actions in PDDL would triple the number of stream evaluations and significantly complicate the domain.
- Hardcoded approach/retreat heights (0.10 m, 0.25 m) work reliably for the tabletop scenario.
- This is a common simplification in TAMP research -- most systems model pick/place as atomic actions with hidden execution details.

**Audit reference:** #93, #79, #82, #86, PF-2, PF-3.

---

## Codebase Policy

### Commented-Out Code Preservation

Commented-out code is preserved intentionally. It serves as:
- Inline documentation of previous approaches.
- Quick-enable toggles for debugging.
- Reference for future re-implementation.

Examples:
- `test_full_pipeline.py`: Commented-out `draw_boxels` call (disabled to simplify pipeline runs).
- `archive/` folder: Entire previous implementations kept for reference.

**Rule:** Never delete commented-out code without explicit approval. If code is truly obsolete, move it to `archive/` rather than deleting.

### Dead Code Documentation

Dead methods, unused imports, and unreachable code paths documented in audit issues #87-#94 are tracked for awareness but are not scheduled for deletion. They impose no runtime cost and may be needed for future features (evaluation framework, real perception, etc.).

---

**See Also:**
- [Known Issues and Roadmap](Known_Issues_and_Roadmap) -- Open issues related to these decisions (#59, #77, #93, PA-1 through PA-6).
- [PDDL Domain Reference](PDDL_Domain_Reference) -- The formal model shaped by these decisions.
- [Execution Pipeline](Execution_Pipeline) -- Where the hidden sub-actions are implemented.
- [Planning System](Planning_System) -- How optimistic sensing and replanning work in practice.

---

[Back to Home](Home)
