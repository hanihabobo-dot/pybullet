[Back to Home](Home)

# Core Data Structures

## Overview

This page documents all data structures used throughout the Semantic Boxels system. These types are defined across four modules and serve as the shared vocabulary between perception, spatial reasoning, planning, and execution. The structures fall into four categories: scene representation types, boxel types, planning types, and belief types.

---

## Scene Representation Types

### ObjectInfo (`boxel_types.py`)

`ObjectInfo` stores metadata about a single object in the PyBullet scene. It is populated during object detection and consumed by the boxel generation pipeline.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `object_id` | `int` | PyBullet body ID. Used for raycasting, collision queries, and constraint creation. |
| `name` | `str` | Human-readable name (e.g., `"occluder_1"`, `"target_3"`). Used as the key in all dictionaries and PDDL facts. |
| `position` | `np.ndarray` | World-frame position `[x, y, z]` from `p.getBasePositionAndOrientation()`. |
| `orientation` | `np.ndarray` | Quaternion `[x, y, z, w]` from PyBullet. |
| `size` | `np.ndarray` | Full dimensions `[width, height, depth]` of the object's collision shape. |
| `is_visible` | `bool` | Whether the object is visible from the camera (determined by `oracle_detect_objects()`). |
| `is_occluder` | `bool` | Whether this object casts shadows. Set to `True` during boxel generation when the object produces at least one shadow boxel. |

---

### ObjectShape (`boxel_env.py`)

An enum defining the supported collision shape types for scene objects.

| Value | Description |
|-------|-------------|
| `BOX` | Rectangular cuboid. Used for the default scene's occluders and targets. |
| `CYLINDER` | Upright cylinder. Used in the mixed-shapes scene preset. |
| `SPHERE` | Sphere. Used in the mixed-shapes scene preset. |

---

### ObjectSpec (`boxel_env.py`)

A dataclass specifying the physical properties of a single scene object. Used by `SceneConfig` to parameterize object creation.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `shape` | `ObjectShape` | Collision shape type (BOX, CYLINDER, or SPHERE). |
| `size` | `Tuple[float, ...]` | Dimensions: `(half_x, half_y, half_z)` for BOX, `(radius, height)` for CYLINDER, `(radius,)` for SPHERE. |
| `color` | `Tuple[float, float, float, float]` | RGBA color for visual rendering (e.g., `(0.8, 0.2, 0.2, 1.0)` for red). |
| `mass` | `float` | Mass in kg. Default `1.0`. |
| `lateral_friction` | `float` | Friction coefficient. Default `0.5`. |

**Properties:**

| Property | Returns | Explanation |
|----------|---------|-------------|
| `aabb_half_extents` | `np.ndarray` | Half-extents of the axis-aligned bounding box, regardless of shape type. |
| `full_extents` | `np.ndarray` | Full AABB dimensions (2 * half-extents). |
| `max_horizontal_width` | `float` | Maximum width in the XY plane. Relevant for gripper capacity checks. |

---

### SceneConfig (`boxel_env.py`)

A dataclass defining a complete scene configuration: which objects to create, where to place them, and optional randomization.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `occluders` | `List[ObjectSpec]` | Specifications for occluder objects. |
| `targets` | `List[ObjectSpec]` | Specifications for target objects. |
| `occluder_positions` | `Optional[List[Tuple]]` | Fixed XY positions for occluders. If `None`, positions are randomly generated. |
| `target_positions` | `Optional[List[Tuple]]` | Fixed XY positions for targets. If `None`, positions are randomly generated. |
| `seed` | `Optional[int]` | Random seed for reproducible random placement. |

**Preset factories:**

| Function | Description |
|----------|-------------|
| `default_scene()` | The original hardcoded scene: 3 occluder cubes (0.15 m) and 4 target cubes (0.08 m) at fixed positions. |
| `mixed_shapes_scene()` | Demonstration scene with cylinders, boxes, and spheres of varying sizes. |
| `scalability_scene(n_occluders, n_targets, seed)` | Randomly generated scene for evaluation. Uses rejection sampling to avoid overlapping placements. |

---

### CameraObservation (`boxel_types.py`)

Container for the results of a camera observation cycle. Returned by `BoxelTestEnv.get_camera_observation()`.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `visible_objects` | `List[str]` | Names of objects visible from the camera. |
| `object_poses` | `Dict[str, Tuple[np.ndarray, np.ndarray]]` | Maps object names to `(position, orientation)` tuples. |
| `boxels` | `Optional[List[Boxel]]` | Semantic boxels generated from this observation. `None` until `generate_boxels()` runs. |
| `rgb_image` | `Optional[np.ndarray]` | RGB image `(H, W, 3)`. Currently always `None` -- reserved for future real-perception integration. |
| `depth_image` | `Optional[np.ndarray]` | Depth image `(H, W)` in meters. Currently always `None`. |
| `point_cloud` | `Optional[np.ndarray]` | Point cloud `(N, 3)` in world coordinates. Currently always `None`. |

---

## Boxel Types

### Boxel (`boxel_types.py`)

The fundamental spatial primitive: a semantic cuboid representing a task-relevant region. All spatial reasoning and planning operates on boxels.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `center` | `np.ndarray` | World-frame center `[x, y, z]` of the axis-aligned bounding box. |
| `extent` | `np.ndarray` | Half-dimensions `[x_half, y_half, z_half]` from center to each face. |
| `object_name` | `Optional[str]` | Name of the associated object (e.g., `"occluder_1"`), or `"free_space"` / `"free_space_merged"` for free-space boxels. `None` for unassigned. |
| `is_occluded` | `bool` | Whether this boxel is known to be occluded (belief state). Default `False`. |
| `is_shadow` | `bool` | Whether this is a shadow region cast by an occluder. Default `False`. |
| `is_free` | `bool` | Whether this boxel represents known free space. Default `False`. |
| `is_candidate` | `bool` | Whether this is a candidate node being processed during octree subdivision. Default `False`. |
| `is_occluder` | `bool` | Whether this object boxel casts shadows (set during `generate_boxels()`). Default `False`. |

---

### BoxelType (`boxel_data.py`)

An enum classifying boxels into their semantic role.

| Value | Description | ID Prefix |
|-------|-------------|-----------|
| `OBJECT` | Bounding box around a detected physical object. | `obj_XXX` |
| `SHADOW` | Occluded region where hidden objects may reside. | `shadow_XXX` |
| `FREE_SPACE` | Known empty space available for object placement. | `free_XXX` |

---

### BoxelData (`boxel_data.py`)

The rich, registry-ready representation of a boxel. Extends the basic `Boxel` with a unique ID, typed classification, geometric properties, and relational metadata. Instances are stored in the `BoxelRegistry`.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `id` | `str` | Unique identifier (e.g., `"obj_006"`, `"shadow_001"`, `"free_042"`). |
| `boxel_type` | `BoxelType` | Semantic classification: OBJECT, SHADOW, or FREE_SPACE. |
| `min_corner` | `np.ndarray` | AABB minimum corner `[x_min, y_min, z_min]`. |
| `max_corner` | `np.ndarray` | AABB maximum corner `[x_max, y_max, z_max]`. |
| `object_name` | `Optional[str]` | Name of the associated object (for OBJECT boxels) or `None`. |
| `is_occluder` | `bool` | Whether this object casts shadows. Only relevant for OBJECT boxels. |
| `shadow_boxel_ids` | `List[str]` | IDs of shadow boxels created by this object. Only relevant for OBJECT boxels that are occluders. |
| `created_by_boxel_id` | `Optional[str]` | ID of the OBJECT boxel that cast this shadow. Only relevant for SHADOW boxels. |
| `created_by_object` | `Optional[str]` | Name of the object that cast this shadow. Only relevant for SHADOW boxels. |
| `neighbor_ids` | `Dict[str, List[str]]` | Adjacent boxels in 6 directions: `x_pos`, `x_neg`, `y_pos`, `y_neg`, `z_pos`, `z_neg`. |
| `on_surface` | `Optional[str]` | Name of the supporting surface (e.g., `"table"`). |
| `surface_z` | `Optional[float]` | Z coordinate of the supporting surface. |

**Computed Properties:**

| Property | Type | Explanation |
|----------|------|-------------|
| `center` | `np.ndarray` | Midpoint of min_corner and max_corner. |
| `extent` | `np.ndarray` | Half-dimensions `(max_corner - min_corner) / 2`. |
| `volume` | `float` | Volume in cubic meters. |

**Methods:**

| Method | Returns | Explanation |
|--------|---------|-------------|
| `to_dict()` | `dict` | Serializes to a JSON-compatible dictionary. |
| `from_dict(data)` | `BoxelData` | Class method. Deserializes from a dictionary. |

---

### BoxelRegistry (`boxel_data.py`)

The central container for all boxels in a scene. Provides typed lookup, ID generation, and JSON serialization.

| Method | Signature | Explanation |
|--------|-----------|-------------|
| `generate_id` | `(boxel_type: BoxelType) -> str` | Produces the next unique ID for the given type (e.g., `"obj_007"`, `"shadow_003"`). |
| `add_boxel` | `(boxel: BoxelData) -> None` | Inserts a boxel into the registry. |
| `get_boxel` | `(boxel_id: str) -> Optional[BoxelData]` | Retrieves a boxel by ID. |
| `get_boxels_by_type` | `(boxel_type: BoxelType) -> List[BoxelData]` | Returns all boxels of the given type. |
| `get_object_boxels` | `() -> List[BoxelData]` | Shorthand for OBJECT-typed boxels. |
| `get_shadow_boxels` | `() -> List[BoxelData]` | Shorthand for SHADOW-typed boxels. |
| `get_free_space_boxels` | `() -> List[BoxelData]` | Shorthand for FREE_SPACE-typed boxels. |
| `compute_neighbors` | `() -> None` | Populates `neighbor_ids` via O(n^2) adjacency checks. Currently not called from active code. |
| `to_dict` | `() -> dict` | Serializes the entire registry including a summary. |
| `save_to_json` | `(filepath: str) -> None` | Writes the registry to a JSON file. |
| `load_from_json` | `(filepath: str) -> BoxelRegistry` | Class method. Loads from a JSON file. |

**Factory Function:**

`create_boxel_registry_from_boxels(boxels: List[Boxel]) -> BoxelRegistry` converts a flat list of `Boxel` objects (from perception) into a typed `BoxelRegistry` with unique IDs and shadow-to-object relationships.

---

### OctreeNode (`boxel_types.py`)

A node in the octree used for free-space subdivision. Not a dataclass -- uses a standard `__init__`.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `center` | `np.ndarray` | World-frame center of this octree cell. |
| `extent` | `np.ndarray` | Half-dimensions of this cell. |
| `children` | `List[OctreeNode]` | Child nodes (8 children when subdivided, empty when leaf). |
| `is_leaf` | `bool` | Whether this is a leaf node (no further subdivision). |
| `state` | `str` | One of `'FREE'`, `'OCCUPIED'`, or `'MIXED'`. |

**Properties:**

| Property | Returns | Explanation |
|----------|---------|-------------|
| `min_bound` | `np.ndarray` | `center - extent` (AABB minimum corner). |
| `max_bound` | `np.ndarray` | `center + extent` (AABB maximum corner). |

---

## Planning Types

### RobotConfig (`streams.py`)

Represents a robot arm configuration (joint angles) produced by IK solving during planning.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `joint_positions` | `tuple` | 7 joint angles in radians for the Franka Panda arm. Stored as a tuple for hashability. |
| `name` | `str` | Human-readable label (e.g., `"q_kin_obj_005_2"`, `"q_home"`). Used in plan output. |
| `is_heuristic` | `bool` | Whether this config was produced by the (now-removed) heuristic IK fallback. Always `False` in current code. |
| `ignored_body_ids` | `frozenset` | PyBullet body IDs to exclude from collision checks when this config is used as a motion endpoint. Typically contains the grasped object's body ID. |

`RobotConfig` is hashable (via `name`) and implements `__eq__` and `__repr__` for use as a PDDLStream atom.

---

### Trajectory (`streams.py`)

A collision-free path through joint space, produced by the motion planner.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `waypoints` | `List[RobotConfig]` | Ordered sequence of joint configurations from start to goal. The first waypoint is the start config; the last is the goal config. |
| `name` | `str` | Human-readable label (e.g., `"traj_q_home_to_q_kin_obj_005_2"`). |

Hashable via `name`. During execution, `waypoints[1:]` are iterated and each is passed to `move_robot_smooth()`.

---

### Grasp (`streams.py`)

A grasp pose relative to the object being grasped.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `position` | `tuple` | Offset `(dx, dy, dz)` from the object's center to the end-effector position. For top-down grasps: `(0, 0, z_offset)` where `z_offset` is 0.05, 0.10, or 0.15 m. |
| `orientation` | `tuple` | End-effector orientation as a quaternion `(x, y, z, w)`. For top-down grasps: pointing straight down. |
| `name` | `str` | Human-readable label (e.g., `"grasp_obj_004_1"`). |

Hashable via `name`. The grasp defines the end-effector goal pose as `object_position + grasp.position` with the given orientation.

---

## Belief Type

### BeliefState (`test_full_pipeline.py`)

Tracks the robot's knowledge about where the target might be hidden. Updated after each sensing action and carried across replanning cycles.

| Attribute | Type | Explanation |
|-----------|------|-------------|
| `shadow_status` | `Dict[str, str]` | Maps shadow boxel IDs to one of: `"unknown"` (not yet sensed), `"not_here"` (sensed and empty), `"found"` (target discovered here). |
| `known_empty_shadows` | `set` | Shadow IDs confirmed to be empty. Passed to the planner so it does not re-plan sensing for these shadows. |
| `occluders_moved` | `Dict[str, str]` | Maps occluder IDs to their destination boxel IDs (e.g., `{"obj_004": "free_023"}`). Passed to the planner to update object positions. |

The belief state is the bridge between execution and replanning: it accumulates observations across planning cycles, ensuring each replan searches strictly fewer shadow candidates than the previous one.

---

**See Also:**
- [Architecture Overview](Architecture_Overview) -- How these types flow through the system.
- [Scene Environment](Scene_Environment) -- Where `ObjectInfo`, `ObjectSpec`, and `SceneConfig` are created.
- [Spatial Reasoning](Spatial_Reasoning) -- Where `Boxel`, `OctreeNode`, and `BoxelRegistry` are populated.
- [Robot Control and Streams](Robot_Control_and_Streams) -- Where `RobotConfig`, `Trajectory`, and `Grasp` are produced.
- [PDDL Domain Reference](PDDL_Domain_Reference) -- How these types map to PDDL predicates and atoms.

---

[Back to Home](Home)
