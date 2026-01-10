# Semantic Boxels for Task and Motion Planning

A PyBullet-based system for generating **semantic boxels** - task-relevant 3D regions used for belief representation in Task and Motion Planning (TAMP). This system discretizes a workspace into meaningful regions including object bounding boxes, occlusion shadows, and free space cells.

## Overview

The system provides:
- **Object Boxels**: Axis-aligned bounding boxes around detected objects
- **Shadow Boxels**: Occluded regions cast by objects from the camera's viewpoint
- **Free Space Boxels**: Merged convex regions representing navigable/manipulable space
- **Spatial Relationships**: Neighbor connectivity between adjacent boxels
- **PDDLStream Integration**: JSON and PDDL fact export for planning

## Installation

```bash
pip install pybullet numpy
```

## Quick Start

```bash
# Run the demo simulation
python run_demo.py

# View the generated boxel data (requires local server)
python -m http.server 8080
# Then open: http://localhost:8080/boxel_viewer.html
```

## Project Structure

```
├── run_demo.py          # Main entry point - runs the simulation demo
├── boxel_env.py         # PyBullet environment setup and boxel generation
├── boxel_types.py       # Core Boxel dataclass definition
├── boxel_data.py        # Rich BoxelData structure for PDDLStream
├── shadow_calculator.py # Ray-casting based shadow region computation
├── free_space.py        # Octree-based free space discretization
├── cell_merger.py       # Merges adjacent free space cells
├── visualization.py     # PyBullet debug visualization helpers
├── boxel_viewer.html    # 3D web viewer for boxel data (Three.js)
├── boxel_data.json      # Generated boxel data output
└── boxel_facts.pddl     # Generated PDDL facts for planning
```

## Boxel Types

### Object Boxels
Bounding boxes around visible objects. Colored by occlusion status:
- 🔴 **Red**: Occluder (casts shadows)
- 🔵 **Blue**: Non-occluder (doesn't block view)

### Shadow Boxels
Regions occluded from the camera's viewpoint:
- ⬛ **Gray**: Unobserved regions where objects might be hidden
- Calculated via ray-casting from camera through object corners
- Split when intersecting with other objects

### Free Space Boxels
Known empty regions:
- 🟢 **Green**: Merged convex free space cells
- Generated via octree subdivision
- Adjacent cells merged for efficiency

## Configuration

Key constants in `run_demo.py`:

```python
PHASE_WAIT_SECONDS = 0      # Animation delay between phases (0 = fast)
BOXEL_FILL_OPACITY = 0.05   # Transparency of filled boxels
FINAL_HOLD_SECONDS = 6      # How long to keep simulation open
ENABLE_FREE_SPACE = True    # Toggle free space generation
```

## Output Data

### boxel_data.json

```json
{
  "boxels": [
    {
      "id": "obj_006",
      "boxel_type": "object",
      "min_corner": [0.42, 0.12, 0.32],
      "max_corner": [0.57, 0.27, 0.47],
      "object_name": "occluder_1",
      "is_occluder": true,
      "shadow_boxel_ids": ["shadow_000"],
      "neighbor_ids": {
        "x_pos": ["shadow_001"],
        "y_pos": ["shadow_000"],
        ...
      },
      "observed": true,
      "on_surface": "table"
    }
  ],
  "summary": {
    "total": 57,
    "objects": 4,
    "shadows": 6,
    "free_space": 47
  }
}
```

### Key Fields

| Field | Description |
|-------|-------------|
| `id` | Unique identifier (obj_XXX, shadow_XXX, free_XXX) |
| `boxel_type` | "object", "shadow", or "free_space" |
| `min_corner` / `max_corner` | AABB bounds [x, y, z] |
| `is_occluder` | Does this object cast shadows? |
| `shadow_boxel_ids` | Shadows created by this object |
| `created_by_boxel_id` | Object that creates this shadow |
| `neighbor_ids` | Adjacent boxels in 6 directions |
| `observed` | True for visible boxels, False for shadows |
| `on_surface` | Supporting surface name (e.g., "table") |

## Camera Controls (Simulation)

- **W/A/S/D or Arrow Keys**: Pan camera
- **Q/E**: Move camera up/down
- **R/F**: Zoom in/out
- **Mouse Scroll**: Zoom

## Web Viewer Controls

- **Left Click + Drag**: Rotate view
- **Right Click + Drag**: Pan view
- **Scroll**: Zoom
- **Click Boxel**: Show detailed info
- **Checkboxes**: Toggle boxel type visibility
- **Sliders**: Adjust opacity per type

## Architecture

```
Camera Observation
       │
       ▼
┌──────────────────┐
│  Object Detection │  → Object Boxels (red/blue)
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Shadow Calculation│  → Shadow Boxels (gray)
│   (Ray Casting)   │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Free Space Octree │  → Free Space Boxels (cyan)
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│   Cell Merger     │  → Merged Free Space (green)
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│ Neighbor Compute  │  → Spatial Graph
└────────┬─────────┘
         │
         ▼
   JSON + PDDL Export
```

## License

MIT
