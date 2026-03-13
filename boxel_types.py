"""
Data structures for Semantic Boxel representation.

This module contains the core data types used throughout the boxel system:
- ObjectInfo: Stores PyBullet object metadata
- Boxel: Semantic cuboid for belief representation
- OctreeNode: Helper for spatial subdivision
- CameraObservation: Container for camera capture results
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass


@dataclass
class ObjectInfo:
    """Data structure to store information about objects in the scene."""
    object_id: int  # PyBullet object ID
    name: str  # Human-readable name
    position: np.ndarray  # [x, y, z] position
    orientation: np.ndarray  # [x, y, z, w] quaternion
    size: np.ndarray  # [width, height, depth] dimensions
    is_visible: bool  # Whether object is currently visible from camera
    is_occluder: bool  # Whether this object is an occluder (larger cube)


@dataclass
class Boxel:
    """
    Semantic Boxel: A task-relevant cuboid for belief representation.
    
    In this implementation, Boxels are axis-aligned bounding boxes (AABBs) 
    that bound objects or regions of interest. This abstraction allows the 
    planner to reason about "regions" rather than precise object meshes.
    """
    center: np.ndarray  # [x, y, z] center coordinates of the box in world frame
    extent: np.ndarray  # [x_half, y_half, z_half] half-dimensions (distance from center to edge)
    object_name: Optional[str] = None  # Name of the object this boxel bounds (if any)
    is_occluded: bool = False  # Belief state: Is this boxel currently known to be occluded?
    is_shadow: bool = False  # Is this a shadow/occlusion region cast by another object?
    is_free: bool = False  # Is this boxel representing known free space?
    is_candidate: bool = False  # Is this a candidate node currently being processed?
    is_occluder: bool = False  # Does this object boxel cast shadows (occlude other regions)?


class OctreeNode:
    """Helper class for Octree spatial subdivision."""
    
    def __init__(self, center: np.ndarray, extent: np.ndarray):
        self.center = center
        self.extent = extent
        self.children: List['OctreeNode'] = []
        self.is_leaf = True
        self.state = 'FREE'  # 'FREE', 'OCCUPIED', 'MIXED'

    @property
    def min_bound(self):
        return self.center - self.extent

    @property
    def max_bound(self):
        return self.center + self.extent


@dataclass
class CameraObservation:
    """Data structure for camera observations."""
    rgb_image: np.ndarray  # RGB image (H, W, 3)
    depth_image: np.ndarray  # Depth image (H, W) in meters
    point_cloud: np.ndarray  # Point cloud (N, 3) in world coordinates
    visible_objects: List[str]  # List of object names that are visible
    object_poses: Dict[str, Tuple[np.ndarray, np.ndarray]]  # Dict mapping object names to (position, orientation)
    boxels: Optional[List[Boxel]] = None  # List of Semantic Boxels generated from the observation
