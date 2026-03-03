"""
Shadow Boxel Calculator.

This module handles the calculation of shadow (occlusion) regions cast by objects.
It uses PyBullet ray casting to determine shadow extent and handles splitting
when shadows intersect with other objects.
"""

import numpy as np
import pybullet as p
from typing import List
from boxel_types import Boxel


class ShadowCalculator:
    """
    Calculates shadow boxels for objects in the scene.
    
    Shadow boxels represent the occluded volume behind an object from
    the camera's perspective.
    """
    
    def __init__(self, camera_position: np.ndarray, table_surface_height: float,
                 table_x_range: tuple = (0.0, 1.0),
                 table_y_range: tuple = (-0.5, 0.5)):
        """
        Initialize the shadow calculator.
        
        Args:
            camera_position: [x, y, z] position of the camera
            table_surface_height: Z height of the table surface
            table_x_range: (min, max) X bounds of the table surface
            table_y_range: (min, max) Y bounds of the table surface
        """
        self.camera_position = camera_position
        self.table_surface_height = table_surface_height
        
        self.table_x_min, self.table_x_max = table_x_range
        self.table_y_min, self.table_y_max = table_y_range
    
    def calculate_shadow_boxel(self, obj_boxel: Boxel, obstacles: List[Boxel]) -> List[Boxel]:
        """
        Calculate the Shadow Boxel(s) cast by an object, accounting for ray casting and obstacles.
        
        1. Ray Casting: Uses PyBullet rayTestBatch to find shadow extent against table/world.
        2. Splitting: Breaks shadow boxels if they intersect with other objects.
        3. Height Constraint: Shadow height is an overestimate (at least as tall as object).
        
        Args:
            obj_boxel: The visible object casting the shadow.
            obstacles: List of other solid boxels that might block the shadow.
            
        Returns:
            List[Boxel]: One or more boxels representing the shadow volume.
        """
        cam_pos = self.camera_position
        obj_center = obj_boxel.center
        obj_extent = obj_boxel.extent
        
        # --- Step 1: Determine Shadow Start (Back Face) ---
        # Direction from camera to object center
        cam_to_obj = obj_center - cam_pos
        cam_to_obj_norm = cam_to_obj / np.linalg.norm(cam_to_obj)
        
        # Generate all 8 corners
        all_corners = [
            obj_center + np.array([-obj_extent[0], -obj_extent[1], -obj_extent[2]]),
            obj_center + np.array([ obj_extent[0], -obj_extent[1], -obj_extent[2]]),
            obj_center + np.array([-obj_extent[0],  obj_extent[1], -obj_extent[2]]),
            obj_center + np.array([ obj_extent[0],  obj_extent[1], -obj_extent[2]]),
            obj_center + np.array([-obj_extent[0], -obj_extent[1],  obj_extent[2]]),
            obj_center + np.array([ obj_extent[0], -obj_extent[1],  obj_extent[2]]),
            obj_center + np.array([-obj_extent[0],  obj_extent[1],  obj_extent[2]]),
            obj_center + np.array([ obj_extent[0],  obj_extent[1],  obj_extent[2]])
        ]
        
        # Filter to only back corners (corners that are away from camera relative to center)
        back_corners = []
        for corner in all_corners:
            offset = corner - obj_center
            dot = np.dot(offset, cam_to_obj_norm)
            if dot > 0:  # Corner is on the "back" side
                back_corners.append(corner)
        
        # Fallback if no back corners found
        if len(back_corners) == 0:
            back_corners = all_corners
            
        corners = back_corners
        
        # Construct rays from camera through back corners
        ray_starts = [cam_pos] * len(corners)
        max_dist = 5.0  # meters
        ray_ends = []
        for corner in corners:
            direction = corner - cam_pos
            direction = direction / np.linalg.norm(direction)
            ray_target = corner + direction * max_dist
            ray_ends.append(ray_target)
            
        # Batch Ray Test
        results = p.rayTestBatch(corners, ray_ends)
        
        table_z = self.table_surface_height
        
        hit_points = []
        for i, res in enumerate(results):
            hit_obj_id = res[0]
            if hit_obj_id != -1:
                hit_pt = np.array(res[3])
                # Clamp to table bounds
                hit_pt[0] = np.clip(hit_pt[0], self.table_x_min, self.table_x_max)
                hit_pt[1] = np.clip(hit_pt[1], self.table_y_min, self.table_y_max)
                hit_pt[2] = max(hit_pt[2], table_z)
                hit_points.append(hit_pt)
            else:
                # No hit - project ray to table plane and clamp
                start = corners[i]
                direction = ray_ends[i] - start
                direction = direction / np.linalg.norm(direction)
                
                if abs(direction[2]) > 1e-6 and direction[2] < 0:
                    t = (table_z - start[2]) / direction[2]
                    if t > 0:
                        pt_on_plane = start + direction * t
                    else:
                        pt_on_plane = np.array([start[0], start[1], table_z])
                else:
                    pt_on_plane = np.array([start[0], start[1], table_z])
                
                clamped_pt = pt_on_plane.copy()
                clamped_pt[0] = np.clip(clamped_pt[0], self.table_x_min, self.table_x_max)
                clamped_pt[1] = np.clip(clamped_pt[1], self.table_y_min, self.table_y_max)
                clamped_pt[2] = table_z
                
                hit_points.append(clamped_pt)
        
        # --- Step 2: Construct Initial Shadow AABB ---
        h_min = np.min(hit_points, axis=0)
        h_max = np.max(hit_points, axis=0)
        h_min[2] = max(h_min[2], table_z)
        
        o_min = obj_center - obj_extent
        o_max = obj_center + obj_extent
        
        full_min = np.minimum(o_min, h_min)
        full_max = np.maximum(o_max, h_max)
        
        # Clamp shadow bounds to table boundaries
        full_min[0] = max(full_min[0], self.table_x_min)
        full_min[1] = max(full_min[1], self.table_y_min)
        full_min[2] = max(full_min[2], table_z)
        full_max[0] = min(full_max[0], self.table_x_max)
        full_max[1] = min(full_max[1], self.table_y_max)
        
        # Enforce Height Overestimate
        full_max[2] = max(full_max[2], o_max[2])
        
        # Subtract object from shadow
        shadow_dir = np.mean(hit_points, axis=0) - obj_center
        dom_axis = np.argmax(np.abs(shadow_dir))
        
        s_min = full_min.copy()
        s_max = full_max.copy()
        
        if shadow_dir[dom_axis] > 0:
            s_min[dom_axis] = max(s_min[dom_axis], o_max[dom_axis])
        else:
            s_max[dom_axis] = min(s_max[dom_axis], o_min[dom_axis])
            
        # Initial Shadow Boxel
        s_center = (s_min + s_max) / 2.0
        s_extent = (s_max - s_min) / 2.0
        
        initial_shadow = Boxel(
            center=s_center,
            extent=s_extent,
            object_name=f"shadow_of_{obj_boxel.object_name}",
            is_occluded=True,
            is_shadow=True
        )
        
        # --- Step 3: Handle Obstacles (Splitting) ---
        active_shadows = [initial_shadow]
        
        for obstacle in obstacles:
            next_active = []
            for shadow in active_shadows:
                if self._check_aabb_intersection(shadow, obstacle):
                    fragments = self._subtract_aabb(shadow, obstacle, shadow_dir)
                    next_active.extend(fragments)
                else:
                    next_active.append(shadow)
            active_shadows = next_active
            
        return active_shadows

    def _check_aabb_intersection(self, b1: Boxel, b2: Boxel) -> bool:
        """Check if two boxels intersect."""
        min1 = b1.center - b1.extent
        max1 = b1.center + b1.extent
        min2 = b2.center - b2.extent
        max2 = b2.center + b2.extent
        
        return (np.all(min1 <= max2) and np.all(max1 >= min2))

    def _subtract_aabb(self, shadow: Boxel, obstacle: Boxel, direction: np.ndarray) -> List[Boxel]:
        """
        Subtract obstacle from shadow, keeping parts 'before' and 'around' the obstacle.
        """
        s_min = shadow.center - shadow.extent
        s_max = shadow.center + shadow.extent
        o_min = obstacle.center - obstacle.extent
        o_max = obstacle.center + obstacle.extent
        
        fragments = []
        
        # Split along each axis
        # 1. Left of Obstacle (Min X)
        if s_min[0] < o_min[0]:
            new_max = s_max.copy()
            new_max[0] = o_min[0]
            fragments.append(self._create_boxel_from_bounds(s_min, new_max, shadow))
            s_min[0] = max(s_min[0], o_min[0])
            
        # 2. Right of Obstacle (Max X)
        if s_max[0] > o_max[0]:
            new_min = s_min.copy()
            new_min[0] = o_max[0]
            fragments.append(self._create_boxel_from_bounds(new_min, s_max, shadow))
            s_max[0] = min(s_max[0], o_max[0])
            
        # 3. Front of Obstacle (Min Y)
        if s_min[1] < o_min[1]:
            new_max = s_max.copy()
            new_max[1] = o_min[1]
            fragments.append(self._create_boxel_from_bounds(s_min, new_max, shadow))
            s_min[1] = max(s_min[1], o_min[1])
            
        # 4. Back of Obstacle (Max Y)
        if s_max[1] > o_max[1]:
            new_min = s_min.copy()
            new_min[1] = o_max[1]
            fragments.append(self._create_boxel_from_bounds(new_min, s_max, shadow))
            s_max[1] = min(s_max[1], o_max[1])
            
        # 5. Bottom of Obstacle (Min Z)
        if s_min[2] < o_min[2]:
            new_max = s_max.copy()
            new_max[2] = o_min[2]
            fragments.append(self._create_boxel_from_bounds(s_min, new_max, shadow))
            s_min[2] = max(s_min[2], o_min[2])
            
        # 6. Top of Obstacle (Max Z)
        if s_max[2] > o_max[2]:
            new_min = s_min.copy()
            new_min[2] = o_max[2]
            fragments.append(self._create_boxel_from_bounds(new_min, s_max, shadow))
            s_max[2] = min(s_max[2], o_max[2])
        
        # Filter out None and downstream fragments
        filtered_fragments = []
        for frag in fragments:
            if frag is not None and not self._is_downstream(frag, obstacle, direction):
                filtered_fragments.append(frag)
                
        return filtered_fragments

    def _create_boxel_from_bounds(self, min_pt, max_pt, template_boxel):
        """Create a boxel from min/max bounds."""
        center = (min_pt + max_pt) / 2.0
        extent = (max_pt - min_pt) / 2.0
        MIN_EXTENT = 0.001  # 1mm minimum size
        if np.any(extent <= 0) or np.any(extent < MIN_EXTENT):
            return None
            
        return Boxel(
            center=center,
            extent=extent,
            object_name=template_boxel.object_name,
            is_occluded=True,
            is_shadow=True
        )

    def _is_downstream(self, frag: Boxel, obstacle: Boxel, direction: np.ndarray) -> bool:
        """Check if a fragment is 'behind' the obstacle relative to shadow direction."""
        dom_axis = np.argmax(np.abs(direction))
        sign = np.sign(direction[dom_axis])
        
        if sign > 0:
            f_min = frag.center[dom_axis] - frag.extent[dom_axis]
            o_max = obstacle.center[dom_axis] + obstacle.extent[dom_axis]
            if f_min >= o_max - 1e-4:
                return True
        else:
            f_max = frag.center[dom_axis] + frag.extent[dom_axis]
            o_min = obstacle.center[dom_axis] - obstacle.extent[dom_axis]
            if f_max <= o_min + 1e-4:
                return True
                
        return False
