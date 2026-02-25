"""
Boxel Data Structure and Serialization for PDDLStream Integration.

This module provides:
- BoxelData: Rich data structure for semantic boxels
- BoxelRegistry: Container for all boxels with relationship tracking
- Serialization to JSON for persistence
- PDDL fact generation for PDDLStream integration
"""

import json
import numpy as np
from typing import List, Dict, Optional, Set, Tuple, Any
from dataclasses import dataclass, field, asdict
from enum import Enum


class BoxelType(Enum):
    """Semantic type of a boxel."""
    OBJECT = "object"           # Bounding box around a detected object
    SHADOW = "shadow"           # Occluded region cast by an object
    FREE_SPACE = "free_space"   # Known free space (merged)


@dataclass
class BoxelData:
    """
    Complete data structure for a Semantic Boxel.
    
    Designed for PDDLStream integration with all necessary fields for:
    - Geometric queries (motion planning)
    - Belief state representation (K-literals)
    - Spatial relationships (neighbors, occlusion)
    - Manipulation planning (reachability, placement)
    """
    
    # === IDENTITY ===
    id: str                                     # Unique identifier (e.g., "boxel_007")
    boxel_type: BoxelType                       # Semantic type
    
    # === GEOMETRY (AABB) ===
    min_corner: np.ndarray                      # [x, y, z] minimum corner
    max_corner: np.ndarray                      # [x, y, z] maximum corner
    
    # === OBJECT BOXEL FIELDS ===
    object_name: Optional[str] = None           # Physical object name (for type=OBJECT)
    is_occluder: bool = False                   # Does this cast shadows?
    shadow_boxel_ids: List[str] = field(default_factory=list)  # IDs of shadows this creates
    
    # === SHADOW BOXEL FIELDS ===
    created_by_boxel_id: Optional[str] = None   # Which object boxel creates this shadow
    created_by_object: Optional[str] = None     # Object name that creates this shadow
    
    # === SPATIAL RELATIONSHIPS ===
    neighbor_ids: Dict[str, List[str]] = field(default_factory=lambda: {
        "x_pos": [], "x_neg": [], "y_pos": [], "y_neg": [], "z_pos": [], "z_neg": []
    })
    on_surface: Optional[str] = None            # Which support surface (e.g., "table")
    surface_z: Optional[float] = None           # Z height of support surface
    
    # === BELIEF STATE ===
    possibly_contains: List[str] = field(default_factory=list)  # Objects that MIGHT be here
    confirmed_contains: Optional[str] = None    # K(InBoxel(obj, this)) - known to contain
    confirmed_empty: bool = False               # K(¬InBoxel(any, this)) - known empty
    observed: bool = False                      # Has this been sensed?
    last_observation_time: Optional[float] = None
    
    # === REACHABILITY & MANIPULATION ===
    is_reachable: bool = True                   # Can robot gripper reach?
    is_observable: bool = True                  # Can camera see this boxel?
    # Note: blocking_boxels removed - use created_by_boxel_id for shadows
    
    @property
    def center(self) -> np.ndarray:
        """Compute center from corners."""
        return (self.min_corner + self.max_corner) / 2.0
    
    @property
    def extent(self) -> np.ndarray:
        """Compute half-extents from corners."""
        return (self.max_corner - self.min_corner) / 2.0
    
    @property
    def volume(self) -> float:
        """Compute volume in cubic meters."""
        dims = self.max_corner - self.min_corner
        return float(np.prod(dims))
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to JSON-serializable dictionary."""
        return {
            "id": self.id,
            "boxel_type": self.boxel_type.value,
            "min_corner": self.min_corner.tolist(),
            "max_corner": self.max_corner.tolist(),
            "center": self.center.tolist(),
            "extent": self.extent.tolist(),
            "volume": self.volume,
            "object_name": self.object_name,
            "is_occluder": self.is_occluder,
            "shadow_boxel_ids": self.shadow_boxel_ids,
            "created_by_boxel_id": self.created_by_boxel_id,
            "created_by_object": self.created_by_object,
            "neighbor_ids": self.neighbor_ids,
            "on_surface": self.on_surface,
            "surface_z": self.surface_z,
            "possibly_contains": self.possibly_contains,
            "confirmed_contains": self.confirmed_contains,
            "confirmed_empty": self.confirmed_empty,
            "observed": self.observed,
            "last_observation_time": self.last_observation_time,
            "is_reachable": self.is_reachable,
            "is_observable": self.is_observable,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'BoxelData':
        """Create BoxelData from dictionary."""
        return cls(
            id=data["id"],
            boxel_type=BoxelType(data["boxel_type"]),
            min_corner=np.array(data["min_corner"]),
            max_corner=np.array(data["max_corner"]),
            object_name=data.get("object_name"),
            is_occluder=data.get("is_occluder", False),
            shadow_boxel_ids=data.get("shadow_boxel_ids", []),
            created_by_boxel_id=data.get("created_by_boxel_id"),
            created_by_object=data.get("created_by_object"),
            neighbor_ids=data.get("neighbor_ids", {}),
            on_surface=data.get("on_surface"),
            surface_z=data.get("surface_z"),
            possibly_contains=data.get("possibly_contains", []),
            confirmed_contains=data.get("confirmed_contains"),
            confirmed_empty=data.get("confirmed_empty", False),
            observed=data.get("observed", False),
            last_observation_time=data.get("last_observation_time"),
            is_reachable=data.get("is_reachable", True),
            is_observable=data.get("is_observable", True),
        )


class BoxelRegistry:
    """
    Container for all boxels with relationship tracking and serialization.
    
    Provides methods for:
    - Adding/retrieving boxels
    - Computing spatial relationships (neighbors)
    - Serializing to JSON
    - Generating PDDL facts
    """
    
    def __init__(self):
        self.boxels: Dict[str, BoxelData] = {}
        self._next_id = 0
    
    def generate_id(self, prefix: str = "boxel") -> str:
        """Generate a unique boxel ID."""
        boxel_id = f"{prefix}_{self._next_id:03d}"
        self._next_id += 1
        return boxel_id
    
    def add_boxel(self, boxel: BoxelData) -> None:
        """Add a boxel to the registry."""
        self.boxels[boxel.id] = boxel
    
    def get_boxel(self, boxel_id: str) -> Optional[BoxelData]:
        """Get a boxel by ID."""
        return self.boxels.get(boxel_id)
    
    def get_boxels_by_type(self, boxel_type: BoxelType) -> List[BoxelData]:
        """Get all boxels of a specific type."""
        return [b for b in self.boxels.values() if b.boxel_type == boxel_type]
    
    def get_object_boxels(self) -> List[BoxelData]:
        """Get all object boxels."""
        return self.get_boxels_by_type(BoxelType.OBJECT)
    
    def get_shadow_boxels(self) -> List[BoxelData]:
        """Get all shadow boxels."""
        return self.get_boxels_by_type(BoxelType.SHADOW)
    
    def get_free_space_boxels(self) -> List[BoxelData]:
        """Get all free space boxels."""
        return self.get_boxels_by_type(BoxelType.FREE_SPACE)
    
    def compute_neighbors(self, tolerance: float = 0.01) -> None:
        """
        Compute neighbor relationships for all boxels.
        
        Two boxels are neighbors if they share a face (are adjacent with matching dimensions).
        """
        boxel_list = list(self.boxels.values())
        
        for i, boxel_a in enumerate(boxel_list):
            for j, boxel_b in enumerate(boxel_list):
                if i >= j:
                    continue
                
                direction = self._check_adjacency(boxel_a, boxel_b, tolerance)
                if direction:
                    # Add bidirectional neighbor relationship
                    opposite = self._opposite_direction(direction)
                    if boxel_b.id not in boxel_a.neighbor_ids[direction]:
                        boxel_a.neighbor_ids[direction].append(boxel_b.id)
                    if boxel_a.id not in boxel_b.neighbor_ids[opposite]:
                        boxel_b.neighbor_ids[opposite].append(boxel_a.id)
    
    def _check_adjacency(self, a: BoxelData, b: BoxelData, tol: float) -> Optional[str]:
        """
        Check if two boxels are adjacent (touching) and return direction from a to b.
        
        Uses a lenient check: boxels are neighbors if they touch on a face,
        meaning they share an overlapping region on two axes and touch on the third.
        """
        # Check each axis for adjacency
        for axis, (pos_dir, neg_dir) in enumerate([
            ("x_pos", "x_neg"), ("y_pos", "y_neg"), ("z_pos", "z_neg")
        ]):
            other_axes = [i for i in range(3) if i != axis]
            
            # Check if boxes OVERLAP on the other two axes (not exact alignment)
            overlaps_on_other_axes = True
            for oa in other_axes:
                # Two ranges overlap if: max(a_min, b_min) < min(a_max, b_max)
                overlap_min = max(a.min_corner[oa], b.min_corner[oa])
                overlap_max = min(a.max_corner[oa], b.max_corner[oa])
                if overlap_max - overlap_min < tol:  # No meaningful overlap
                    overlaps_on_other_axes = False
                    break
            
            if not overlaps_on_other_axes:
                continue
            
            # Check if adjacent (touching) along this axis
            if abs(a.max_corner[axis] - b.min_corner[axis]) < tol:
                return pos_dir  # b is in positive direction from a
            if abs(b.max_corner[axis] - a.min_corner[axis]) < tol:
                return neg_dir  # b is in negative direction from a
        
        return None
    
    def _opposite_direction(self, direction: str) -> str:
        """Get the opposite direction."""
        opposites = {
            "x_pos": "x_neg", "x_neg": "x_pos",
            "y_pos": "y_neg", "y_neg": "y_pos",
            "z_pos": "z_neg", "z_neg": "z_pos",
        }
        return opposites[direction]
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert registry to JSON-serializable dictionary."""
        return {
            "boxels": [b.to_dict() for b in self.boxels.values()],
            "summary": {
                "total": len(self.boxels),
                "objects": len(self.get_object_boxels()),
                "shadows": len(self.get_shadow_boxels()),
                "free_space": len(self.get_free_space_boxels()),
            }
        }
    
    def save_to_json(self, filepath: str) -> None:
        """Save registry to JSON file."""
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
        print(f"Saved {len(self.boxels)} boxels to {filepath}")
    
    @classmethod
    def load_from_json(cls, filepath: str) -> 'BoxelRegistry':
        """Load registry from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        registry = cls()
        for boxel_data in data["boxels"]:
            boxel = BoxelData.from_dict(boxel_data)
            registry.add_boxel(boxel)
        
        return registry
    
    def generate_pddl_facts(self) -> List[str]:
        """
        Generate PDDL facts using predicates from domain_pddlstream.pddl.

        Produces static scene structure facts only (type predicates and
        relationships). Dynamic state (occluder_blocking, obj_at_boxel_KIF,
        at_config, etc.) depends on execution context and is managed by
        PDDLStreamPlanner.create_problem().

        Returns:
            List of PDDL fact strings matching domain_pddlstream.pddl.
        """
        facts = []

        for boxel in self.boxels.values():
            facts.append(f"(Boxel {boxel.id})")

            if boxel.boxel_type == BoxelType.SHADOW:
                facts.append(f"(is_shadow {boxel.id})")
            elif boxel.boxel_type == BoxelType.OBJECT:
                facts.append(f"(is_occluder {boxel.id})")
            elif boxel.boxel_type == BoxelType.FREE_SPACE:
                facts.append(f"(is_free_space {boxel.id})")

            if boxel.boxel_type == BoxelType.OBJECT:
                for shadow_id in boxel.shadow_boxel_ids:
                    facts.append(f"(casts_shadow {boxel.id} {shadow_id})")

        return facts
    
    def save_pddl_facts(self, filepath: str) -> None:
        """Save PDDL facts to a file.

        Output uses predicates from domain_pddlstream.pddl.  Only static
        scene structure is included; dynamic state (occluder_blocking,
        obj_at_boxel_KIF, at_config, etc.) is added at planning time by
        PDDLStreamPlanner.create_problem().
        """
        facts = self.generate_pddl_facts()
        with open(filepath, 'w') as f:
            f.write(";; Boxel PDDL Facts — domain_pddlstream.pddl predicates\n")
            f.write(";; Auto-generated by BoxelRegistry.save_pddl_facts()\n")
            f.write(";; Static scene structure only (type predicates + relationships)\n\n")
            for fact in facts:
                f.write(f"{fact}\n")
        print(f"Saved {len(facts)} PDDL facts to {filepath}")


def create_boxel_registry_from_boxels(boxels, table_surface_height: float) -> BoxelRegistry:
    """
    Create a BoxelRegistry from a list of Boxel objects (from boxel_types.py).
    
    This bridges the old Boxel class with the new BoxelData structure.
    
    Args:
        boxels: List of Boxel objects from the existing system
        table_surface_height: Z height of the table surface
        
    Returns:
        BoxelRegistry with all boxels converted
    """
    from boxel_types import Boxel
    
    registry = BoxelRegistry()
    boxel_id_map = {}  # Map old boxel to new ID
    object_name_to_id = {}  # Map object name to boxel ID
    shadow_parent_map = {}  # Map shadow boxel ID to parent object name
    
    # First pass: Create all boxel data objects
    for i, boxel in enumerate(boxels):
        # Determine type
        if boxel.is_shadow:
            boxel_type = BoxelType.SHADOW
        elif boxel.is_free or (boxel.object_name and boxel.object_name.startswith("free_space")):
            boxel_type = BoxelType.FREE_SPACE
        else:
            boxel_type = BoxelType.OBJECT
        
        # Generate ID based on type
        if boxel_type == BoxelType.OBJECT:
            prefix = "obj"
        elif boxel_type == BoxelType.SHADOW:
            prefix = "shadow"
        else:
            prefix = "free"
        
        boxel_id = registry.generate_id(prefix)
        boxel_id_map[id(boxel)] = boxel_id
        
        # Track object name to ID mapping
        if boxel_type == BoxelType.OBJECT and boxel.object_name:
            object_name_to_id[boxel.object_name] = boxel_id
        
        # Extract parent object name for shadow boxels (format: "shadow_of_<object_name>")
        parent_object_name = None
        if boxel_type == BoxelType.SHADOW and boxel.object_name:
            if boxel.object_name.startswith("shadow_of_"):
                parent_object_name = boxel.object_name[len("shadow_of_"):]
                shadow_parent_map[boxel_id] = parent_object_name
        
        # Create BoxelData
        min_corner = boxel.center - boxel.extent
        max_corner = boxel.center + boxel.extent
        
        # observed = True for objects and free space (visible), False for shadows (occluded)
        is_observed = (boxel_type != BoxelType.SHADOW)
        
        boxel_data = BoxelData(
            id=boxel_id,
            boxel_type=boxel_type,
            min_corner=min_corner,
            max_corner=max_corner,
            object_name=boxel.object_name if boxel_type == BoxelType.OBJECT else None,
            is_occluder=boxel.is_occluder if hasattr(boxel, 'is_occluder') else False,
            created_by_object=parent_object_name,  # For shadow boxels
            on_surface="table" if min_corner[2] <= table_surface_height + 0.01 else None,
            surface_z=table_surface_height,
            observed=is_observed,  # Objects and free space are observed, shadows are not
        )
        
        registry.add_boxel(boxel_data)
    
    # Second pass: Link shadow boxels to parent object boxels
    for shadow_id, parent_name in shadow_parent_map.items():
        shadow_boxel = registry.get_boxel(shadow_id)
        if parent_name in object_name_to_id:
            parent_id = object_name_to_id[parent_name]
            parent_boxel = registry.get_boxel(parent_id)
            
            # Link shadow to parent
            shadow_boxel.created_by_boxel_id = parent_id
            
            # Link parent to shadow
            if shadow_id not in parent_boxel.shadow_boxel_ids:
                parent_boxel.shadow_boxel_ids.append(shadow_id)
    
    # Compute neighbor relationships
    registry.compute_neighbors()
    
    return registry
