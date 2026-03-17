# Semantic Boxels -- Task and Motion Planning under Partial Observability

**Project:** Master's Thesis Implementation  
**Domain:** Task and Motion Planning (TAMP) with Partial Observability  
**Simulator:** PyBullet (Franka Emika Panda)  
**Planner:** PDDLStream + FastDownward  
**Language:** Python 3

---

## Abstract

This repository implements a system for **Semantic Boxels** -- task-relevant 3D regions used for belief representation in Task and Motion Planning. The system addresses the hidden-object search problem: a robot must find and retrieve a target object that may be hidden behind occluders on a tabletop, using only partial information from a fixed overhead camera.

The workspace is discretized into three types of semantic boxels:

- **Object Boxels** -- axis-aligned bounding boxes around detected objects.
- **Shadow Boxels** -- occluded regions cast by objects from the camera's viewpoint, where hidden targets may reside.
- **Free Space Boxels** -- merged convex regions representing known empty space where objects can be placed.

A PDDLStream planner reasons over these boxels to generate plans that relocate occluders (via pick-and-place), sense shadow regions (via camera raycasting), and retrieve discovered targets. When sensing reveals that a shadow is empty, the system replans with updated belief state until the target is found.

The architecture combines symbolic planning (PDDL with derived predicates and Know-If fluents) with geometric reasoning (inverse kinematics, RRT-Connect motion planning, collision checking) through PDDLStream's stream abstraction.

---

## Table of Contents

### Foundations

1. [Architecture Overview](Architecture_Overview) -- High-level system map, module dependencies, and data flow.
2. [Design Decisions](Design_Decisions) -- Rationale for key architectural choices and deviations from the thesis proposal.

### Module Documentation

3. [Scene Environment](Scene_Environment) -- PyBullet simulation setup, camera model, object creation, and scene presets.
4. [Spatial Reasoning](Spatial_Reasoning) -- Shadow calculation, free space octree, cell merging, and registry construction.
5. [Planning System](Planning_System) -- PDDLStream planner integration, problem construction, and replanning architecture.
6. [Robot Control and Streams](Robot_Control_and_Streams) -- Inverse kinematics, motion planning, grasp sampling, and actuation.
7. [Execution Pipeline](Execution_Pipeline) -- End-to-end pipeline walkthrough, action handlers, and a concrete scenario.

### Reference

8. [Core Data Structures](Core_Data_Structures) -- All types, dataclasses, and enums with attribute tables.
9. [PDDL Domain Reference](PDDL_Domain_Reference) -- Predicates, derived predicates, actions, streams, and PDDL/Python alignment.
10. [Known Issues and Roadmap](Known_Issues_and_Roadmap) -- Audit status, open issues by priority, proposal alignment gaps, and future work.

---

## How to Navigate This Wiki

- Every page contains a **[Back to Home](Home)** link at the top and bottom for easy return navigation.
- Inline links connect related concepts across pages (e.g., a data structure on the [Core Data Structures](Core_Data_Structures) page links to the module that uses it in [Planning System](Planning_System)).
- The **See Also** section at the bottom of each page lists the most closely related pages.
- File paths reference the repository root (e.g., `boxel_env.py`, `pddl/domain_pddlstream.pddl`).

---

## Quick Start

### Installation

```bash
pip install pybullet numpy
```

PDDLStream must be cloned separately and its path configured in `pddlstream_planner.py`:

```bash
git clone https://github.com/caelan/pddlstream.git ../pddlstream_lib
```

### Running the Full Pipeline

The planner requires PDDLStream, which runs on Linux. From the project directory:

```bash
python3 -u test_full_pipeline.py
```

With verbose logging and headless mode:

```bash
python3 -u test_full_pipeline.py --log-level verbose --no-gui
```

### CLI Options

| Flag | Description |
|------|-------------|
| `--no-gui` | Run headless (no PyBullet GUI window) |
| `--log-level quiet\|normal\|verbose` | Control console verbosity (default: `normal`) |
| `--scene default\|mixed\|scalability` | Select scene preset (default: `default`) |
| `--n-occluders N` | Number of occluders for scalability scene |
| `--n-targets N` | Number of targets for scalability scene |
| `--seed N` | Random seed for scalability scene |

Each run auto-creates a timestamped log directory under `logs/run_<timestamp>/` containing the full output, boxel data JSON, and PDDL problem export.

---

## Technology Stack

| Component | Technology | Purpose |
|-----------|-----------|---------|
| Physics Simulation | PyBullet | Rigid-body dynamics, collision detection, raycasting |
| Robot | Franka Emika Panda (7-DOF) | Manipulation in the simulated environment |
| Task Planner | PDDLStream + FastDownward | Integrated task and motion planning |
| Spatial Reasoning | Custom (Python + NumPy) | Shadow calculation, octree subdivision, cell merging |
| Motion Planning | RRT-Connect | Collision-free joint-space trajectories |
| Inverse Kinematics | PyBullet IK (null-space) | End-effector positioning with joint-limit constraints |
| Logging | RunLogger (custom) | Timestamped output capture and artefact archiving |

---

*This wiki documents the Semantic Boxels codebase as of March 2026. For the current list of open issues and their resolution status, see [Known Issues and Roadmap](Known_Issues_and_Roadmap).*
