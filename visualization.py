"""
Boxel Visualization utilities.

This module handles rendering of boxels in the PyBullet GUI
using debug lines and semi-transparent phantom objects.
"""

import numpy as np
import pybullet as p
from typing import List
from boxel_types import Boxel


class BoxelVisualizer:
    """
    Visualizes boxels in PyBullet using debug lines and phantom objects.
    
    Color coding:
    - Red = Occluder (Obstacle)
    - Blue = Target (Goal)
    - Gray = Shadow (Unknown/Occluded Region)
    - Cyan = Free Space
    - Yellow = Candidate (Processing)
    - Green = Other
    """
    
    def __init__(self):
        """Initialize the visualizer."""
        self.debug_items = []
        self.shadow_bodies = []
    
    def draw_boxels(self, boxels: List[Boxel], duration: float = 0, clear_previous: bool = True,
                    fill_opacity: float = 0.05):
        """
        Visualize Semantic Boxels in the PyBullet GUI using debug lines.
        
        Args:
            boxels: List of Boxel objects to visualize
            duration: How long lines remain visible (0 = forever)
            clear_previous: If True, clears previous debug items before drawing
            fill_opacity: Opacity for filled boxel phantoms (0.0 = invisible, 1.0 = solid)
        """
        # Remove existing shadow bodies
        for body_id in self.shadow_bodies:
            p.removeBody(body_id)
        self.shadow_bodies = []
        
        # Optionally remove existing debug lines
        if clear_previous:
            for item_id in self.debug_items:
                p.removeUserDebugItem(item_id)
            self.debug_items = []
        
        for boxel in boxels:
            c = boxel.center
            e = boxel.extent
            
            # Define the 8 corners
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
            
            # Define edges
            edges = [
                (0, 1), (0, 2), (0, 4),
                (1, 3), (1, 5),
                (2, 3), (2, 6),
                (3, 7),
                (4, 5), (4, 6),
                (5, 7),
                (6, 7)
            ]
            
            # Determine color
            color = self._get_boxel_color(boxel)
            
            # Determine line width
            is_thin = boxel.is_shadow or boxel.is_free or boxel.is_candidate
            
            # Draw wireframe
            for start_idx, end_idx in edges:
                line_id = p.addUserDebugLine(
                    lineFromXYZ=corners[start_idx],
                    lineToXYZ=corners[end_idx],
                    lineColorRGB=color,
                    lineWidth=1.0 if is_thin else 2.0, 
                    lifeTime=duration
                )
                self.debug_items.append(line_id)
            
            # Draw filled phantom for all boxels
            self._draw_boxel_phantom(c, e, color, fill_opacity)
    
    def _get_boxel_color(self, boxel: Boxel) -> List[float]:
        """Get the color for a boxel based on its type."""
        if boxel.is_candidate:
            return [1, 1, 0]  # Yellow - being processed
        elif boxel.is_shadow:
            return [0.5, 0.5, 0.5]  # Gray - shadow/occluded region
        elif boxel.is_free:
            return [0, 1, 1]  # Cyan - free space (before merge)
        elif boxel.object_name and boxel.object_name.startswith("free_space"):
            return [0, 1, 0]  # Green - merged free space
        elif boxel.is_occluder:
            return [1, 0, 0]  # Red - object that casts shadows (occluding something)
        elif boxel.object_name:
            return [0, 0, 1]  # Blue - object that doesn't occlude anything
        else:
            return [0, 1, 0]  # Green - fallback
    
    def _draw_boxel_phantom(self, center, extent, color, opacity):
        """Draw a semi-transparent phantom object for boxel visualization."""
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=extent,
            rgbaColor=[color[0], color[1], color[2], opacity],
            specularColor=[0, 0, 0]
        )
        
        body_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=center,
            baseOrientation=[0, 0, 0, 1]
        )
        
        self.shadow_bodies.append(body_id)
    
    def clear_all(self):
        """Clear all debug items and shadow bodies."""
        for body_id in self.shadow_bodies:
            p.removeBody(body_id)
        self.shadow_bodies = []
        
        for item_id in self.debug_items:
            p.removeUserDebugItem(item_id)
        self.debug_items = []
