# Semantic Boxels for Task and Motion Planning

A PyBullet-based system for generating **semantic boxels** -- task-relevant 3D regions used for belief representation in Task and Motion Planning (TAMP) under partial observability. A Franka Panda robot searches for hidden objects by relocating occluders, sensing shadow regions, and retrieving targets, all planned via PDDLStream.

## Quick Start

```bash
pip install pybullet numpy
python3 -u test_full_pipeline.py
```

| Flag | Description |
|------|-------------|
| `--no-gui` | Headless mode (no PyBullet window) |
| `--log-level verbose` | Full debug output |
| `--scene default\|mixed\|scalability` | Scene preset |

## Documentation

Detailed documentation is in the **[project wiki](wiki/Home.md)**:

| Page | What it covers |
|------|---------------|
| [Architecture Overview](wiki/Architecture_Overview.md) | Module dependencies, data flow diagrams, file structure |
| [Scene Environment](wiki/Scene_Environment.md) | PyBullet setup, camera model, scene presets, object detection |
| [Spatial Reasoning](wiki/Spatial_Reasoning.md) | Shadow calculation, free-space octree, cell merging |
| [Planning System](wiki/Planning_System.md) | PDDLStream integration, problem construction, replanning |
| [Robot Control and Streams](wiki/Robot_Control_and_Streams.md) | IK, RRT-Connect motion planning, grasp sampling |
| [Execution Pipeline](wiki/Execution_Pipeline.md) | End-to-end walkthrough, action handlers, concrete scenario |
| [Core Data Structures](wiki/Core_Data_Structures.md) | All types, dataclasses, and enums |
| [PDDL Domain Reference](wiki/PDDL_Domain_Reference.md) | Predicates, actions, streams, PDDL/Python alignment |
| [Design Decisions](wiki/Design_Decisions.md) | Rationale for key choices and proposal deviations |
| [Known Issues and Roadmap](wiki/Known_Issues_and_Roadmap.md) | Audit status, open issues, future work |

## License

MIT
