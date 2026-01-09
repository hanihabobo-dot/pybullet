"""
Free Space Discretization using Octree.

This module handles the discretization of free space above the table surface
using an octree-based breadth-first subdivision algorithm.
"""

import numpy as np
import pybullet as p
import time
from typing import List
from boxel_types import Boxel, OctreeNode


class FreeSpaceGenerator:
    """
    Generates free space boxels using octree subdivision.
    
    The algorithm starts with a large bounding box covering the workspace
    and recursively subdivides regions that intersect with known objects
    until reaching a minimum resolution.
    """
    
    def __init__(self, table_surface_height: float, min_resolution: float = 0.05):
        """
        Initialize the free space generator.
        
        Args:
            table_surface_height: Z height of the table surface
            min_resolution: Minimum boxel size in meters (default 3cm)
        """
        self.table_surface_height = table_surface_height
        self.min_resolution = min_resolution
        
        # Workspace bounds (volume above table)
        self.ws_min = np.array([0.0, -0.5, table_surface_height])
        self.ws_max = np.array([1.0, 0.5, table_surface_height + 0.5])
        
        # Debug drawing state
        self.debug_items = []
        self.candidate_debug_items = []
    
    def generate(self, known_boxels: List[Boxel], visualize: bool = False) -> List[Boxel]:
        """
        Discretize the free space using an Octree (Breadth-First Search).
        
        Args:
            known_boxels: List of Object and Shadow Boxels
            visualize: If True, animates the generation process (1s per depth layer)
            
        Returns:
            List of Boxels representing the free space
        """
        root_center = (self.ws_min + self.ws_max) / 2.0
        root_extent = (self.ws_max - self.ws_min) / 2.0
        
        root = OctreeNode(root_center, root_extent)
        free_boxels = []
        
        # Helper to get bounds of a boxel
        def get_bounds(b: Boxel):
            return b.center - b.extent, b.center + b.extent
        
        # Pre-compute bounds for known boxels
        known_bounds = [get_bounds(b) for b in known_boxels]
        
        # BFS Queue
        current_layer = [root]
        drawn_free_boxels = set()
        
        while current_layer:
            next_layer = []
            
            for node in current_layer:
                node_min = node.min_bound
                node_max = node.max_bound
                
                is_mixed = False
                for b_min, b_max in known_bounds:
                    if (np.all(node_max >= b_min) and np.all(node_min <= b_max)):
                        is_mixed = True
                        break
                
                if not is_mixed:
                    # FREE
                    node.state = 'FREE'
                    new_boxel = Boxel(
                        center=node.center,
                        extent=node.extent,
                        object_name="free_space",
                        is_occluded=False,
                        is_shadow=False,
                        is_free=True
                    )
                    free_boxels.append(new_boxel)
                    
                    # Visualization
                    boxel_key = (tuple(node.center), tuple(node.extent))
                    if visualize and boxel_key not in drawn_free_boxels:
                        drawn_free_boxels.add(boxel_key)
                        self._draw_boxel_wireframe(node.center, node.extent, [0, 1, 1])
                    
                    continue
                
                # Check size
                max_dim = np.max(node.extent * 2)
                if max_dim <= self.min_resolution:
                    node.state = 'OCCUPIED'
                    continue
                
                # Split
                node.is_leaf = False
                node.state = 'MIXED'
                
                offsets = [
                    [-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
                    [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1]
                ]
                child_extent = node.extent / 2.0
                
                for off in offsets:
                    child_center = node.center + np.array(off) * child_extent
                    child = OctreeNode(child_center, child_extent)
                    node.children.append(child)
                    next_layer.append(child)
            
            # Visualization Step
            if visualize:
                # Clear previous yellow candidates
                for item_id in self.candidate_debug_items:
                    p.removeUserDebugItem(item_id)
                self.candidate_debug_items = []
                
                # Draw current candidates (yellow)
                for node in next_layer:
                    self._draw_boxel_wireframe(node.center, node.extent, [1, 1, 0], 
                                               track_as_candidate=True)
                
                time.sleep(1.0)
            
            current_layer = next_layer

        return free_boxels
    
    def _draw_boxel_wireframe(self, center, extent, color, track_as_candidate=False):
        """Draw a wireframe box using PyBullet debug lines."""
        c = center
        e = extent
        corners = [
            c + np.array([-e[0], -e[1], -e[2]]),
            c + np.array([e[0], -e[1], -e[2]]),
            c + np.array([-e[0], e[1], -e[2]]),
            c + np.array([e[0], e[1], -e[2]]),
            c + np.array([-e[0], -e[1], e[2]]),
            c + np.array([e[0], -e[1], e[2]]),
            c + np.array([-e[0], e[1], e[2]]),
            c + np.array([e[0], e[1], e[2]])
        ]
        edges = [
            (0, 1), (0, 2), (0, 4), (1, 3), (1, 5),
            (2, 3), (2, 6), (3, 7), (4, 5), (4, 6),
            (5, 7), (6, 7)
        ]
        for start_idx, end_idx in edges:
            line_id = p.addUserDebugLine(
                lineFromXYZ=corners[start_idx],
                lineToXYZ=corners[end_idx],
                lineColorRGB=color,
                lineWidth=1.0,
                lifeTime=0
            )
            if track_as_candidate:
                self.candidate_debug_items.append(line_id)
            else:
                self.debug_items.append(line_id)
    
    def clear_debug_items(self):
        """Clear all debug visualization items."""
        for item_id in self.debug_items:
            p.removeUserDebugItem(item_id)
        self.debug_items = []
        
        for item_id in self.candidate_debug_items:
            p.removeUserDebugItem(item_id)
        self.candidate_debug_items = []
