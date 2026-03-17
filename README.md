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

Detailed documentation is in the **[project wiki](https://github.com/hanihabobo-dot/pybullet/wiki)**:

| Page | What it covers |
|------|---------------|
| [Architecture Overview](https://github.com/hanihabobo-dot/pybullet/wiki/Architecture_Overview) | Module dependencies, data flow diagrams, file structure |
| [Scene Environment](https://github.com/hanihabobo-dot/pybullet/wiki/Scene_Environment) | PyBullet setup, camera model, scene presets, object detection |
| [Spatial Reasoning](https://github.com/hanihabobo-dot/pybullet/wiki/Spatial_Reasoning) | Shadow calculation, free-space octree, cell merging |
| [Planning System](https://github.com/hanihabobo-dot/pybullet/wiki/Planning_System) | PDDLStream integration, problem construction, replanning |
| [Robot Control and Streams](https://github.com/hanihabobo-dot/pybullet/wiki/Robot_Control_and_Streams) | IK, RRT-Connect motion planning, grasp sampling |
| [Execution Pipeline](https://github.com/hanihabobo-dot/pybullet/wiki/Execution_Pipeline) | End-to-end walkthrough, action handlers, concrete scenario |
| [Core Data Structures](https://github.com/hanihabobo-dot/pybullet/wiki/Core_Data_Structures) | All types, dataclasses, and enums |
| [PDDL Domain Reference](https://github.com/hanihabobo-dot/pybullet/wiki/PDDL_Domain_Reference) | Predicates, actions, streams, PDDL/Python alignment |
| [Design Decisions](https://github.com/hanihabobo-dot/pybullet/wiki/Design_Decisions) | Rationale for key choices and proposal deviations |
| [Known Issues and Roadmap](https://github.com/hanihabobo-dot/pybullet/wiki/Known_Issues_and_Roadmap) | Audit status, open issues, future work |

## License

MIT
