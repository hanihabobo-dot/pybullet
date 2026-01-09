"""
Boxel Test Environment for Semantic POD-TAMP Research

This module implements a PyBullet simulation environment designed for testing
Semantic Partitioning for Partially Observable Deterministic Task and Motion Planning.
The environment includes:
- A Franka Emika Panda robotic arm
- Occluder objects (larger cubes) that can hide target objects
- Target objects (smaller cubes) that may be partially or fully occluded
- A fixed depth camera for perception
- An oracle perception system that can detect visible objects and generate point clouds

This environment supports the research on "Semantic Boxels" - task-relevant cuboids
that serve as the foundation for belief representation in partially observable TAMP.
"""

import numpy as np
import pybullet as p
import pybullet_data
import time
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
    is_candidate: bool = False # Is this a candidate node currently being processed?


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

    def intersect(self, other_min: np.ndarray, other_max: np.ndarray) -> bool:
        """Check AABB intersection."""
        return np.all(self.min_bound <= other_max) and np.all(self.max_bound >= other_min)

    def contains(self, other_min: np.ndarray, other_max: np.ndarray) -> bool:
        """Check if this node fully contains the other AABB."""
        return np.all(self.min_bound <= other_min) and np.all(self.max_bound >= other_max)



@dataclass
class CameraObservation:
    """Data structure for camera observations."""
    rgb_image: np.ndarray  # RGB image (H, W, 3)
    depth_image: np.ndarray  # Depth image (H, W) in meters
    point_cloud: np.ndarray  # Point cloud (N, 3) in world coordinates
    visible_objects: List[str]  # List of object names that are visible
    object_poses: Dict[str, Tuple[np.ndarray, np.ndarray]]  # Dict mapping object names to (position, orientation)
    boxels: List[Boxel] = None  # List of Semantic Boxels generated from the observation


class BoxelTestEnv:
    """
    PyBullet environment for testing Semantic Boxel-based POD-TAMP.
    
    This environment creates a scene with:
    - A Franka Panda arm (fixed base)
    - Multiple occluder cubes (larger, can hide objects)
    - Multiple target cubes (smaller, may be hidden)
    - A fixed depth camera for perception
    - Oracle functions for object detection and pose estimation
    """
    
    def __init__(
        self,
        gui: bool = True,
        camera_position: Optional[np.ndarray] = None,
        camera_target: Optional[np.ndarray] = None,
        camera_up: Optional[np.ndarray] = None,
        image_width: int = 640,
        image_height: int = 480,
        fov: float = 60.0,
        near_plane: float = 0.01,
        far_plane: float = 5.0
    ):
        """
        Initialize the Boxel test environment.
        
        Args:
            gui: Whether to show the PyBullet GUI
            camera_position: Position of the camera [x, y, z]. If None, uses default.
            camera_target: Point the camera looks at [x, y, z]. If None, uses default.
            camera_up: Camera up vector [x, y, z]. If None, uses default.
            image_width: Width of camera images in pixels
            image_height: Height of camera images in pixels
            fov: Field of view in degrees
            near_plane: Near clipping plane distance
            far_plane: Far clipping plane distance
        """
        # Store camera parameters
        self.image_width = image_width
        self.image_height = image_height
        self.fov = fov
        self.near_plane = near_plane
        self.far_plane = far_plane
        
        # Set default camera position (overhead view, slightly angled)
        # Camera positioned to view the table surface (lowered to z ~ 0.5m)
        if camera_position is None:
            camera_position = np.array([0.5, -0.8, 0.7])  # Adjusted to see lowered table
        if camera_target is None:
            camera_target = np.array([0.5, 0.0, 0.5])  # Target lowered table surface
        if camera_up is None:
            camera_up = np.array([0, 0, 1])
        
        self.camera_position = camera_position
        self.camera_target = camera_target
        self.camera_up = camera_up
        
        # Connect to PyBullet
        if gui:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)
        
        # Reset simulation
        p.resetSimulation()
        
        # Set up physics
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)  # Step-based simulation
        
        # Set search path for URDFs
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Store object information
        self.objects: Dict[str, ObjectInfo] = {}
        
        # Debug items (lines, text) to clear on reset or update
        self.debug_items = []
        
        # Track candidate debug items separately (yellow lines that get replaced each iteration)
        self.candidate_debug_items = []
        
        # Track visual bodies for shadows (ghost objects)
        self.shadow_bodies = []
        
        # Initialize the scene
        self._setup_scene()
        
        print("Boxel Test Environment initialized successfully!")
        print(f"Camera position: {self.camera_position}")
        print(f"Camera target: {self.camera_target}")
        print(f"Objects in scene: {list(self.objects.keys())}")
    
    def _setup_scene(self):
        """
        Set up the simulation scene with plane, table, robot, and objects.
        
        This function:
        1. Loads a ground plane
        2. Loads a table URDF model
        3. Loads the Franka Panda arm positioned to reach the table
        4. Creates occluder cubes (larger cubes that can hide objects) on the table
        5. Creates target cubes (smaller cubes that may be hidden) on the table
        6. Positions objects to create occlusion scenarios
        """
        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        self.objects["plane"] = ObjectInfo(
            object_id=plane_id,
            name="plane",
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1]),
            size=np.array([10, 10, 0.1]),
            is_visible=True,
            is_occluder=False
        )
        
        # Load table
        # Table lowered for better robot access
        # Lower the table by moving it down in z-direction
        table_z_offset = -0.3  # Lower table by 30cm
        table_position = [0.5, 0.0, table_z_offset]  # Center table in workspace, lowered
        table_orientation = [0, 0, 0, 1]
        table_id = p.loadURDF(
            "table/table.urdf",
            table_position,
            table_orientation,
            useFixedBase=True
        )
        
        # Get table dimensions (approximate - table surface is typically at z ~ 0.8)
        # PyBullet table URDF has surface at approximately 0.625m height from its base (empirically determined)
        # Since we lowered the table by table_z_offset, adjust surface height accordingly
        table_surface_height = 0.625 + table_z_offset  # Height of table surface from ground (~0.325m)
        self.table_surface_height = table_surface_height
        
        self.objects["table"] = ObjectInfo(
            object_id=table_id,
            name="table",
            position=np.array(table_position),
            orientation=np.array(table_orientation),
            size=np.array([1.0, 1.0, 0.8]),  # Approximate table size
            is_visible=True,
            is_occluder=False
        )
        
        # Load Franka Panda arm (fixed base)
        # Position it so it can reach objects on the table
        # Robot base positioned to the side/back of table to avoid intersection
        # Franka Panda has ~0.85m reach, so position at -0.4m allows reaching table center
        robot_base_position = [-0.4, 0.0, 0.0]  # Moved back to avoid table intersection
        robot_base_orientation = [0, 0, 0, 1]
        robot_id = p.loadURDF(
            "franka_panda/panda.urdf",
            robot_base_position,
            robot_base_orientation,
            useFixedBase=True
        )
        self.objects["robot"] = ObjectInfo(
            object_id=robot_id,
            name="robot",
            position=np.array(robot_base_position),
            orientation=np.array(robot_base_orientation),
            size=np.array([0.5, 0.5, 0.8]),  # Approximate robot size
            is_visible=True,
            is_occluder=False
        )
        
        # Create occluder cubes (larger cubes that can hide target objects)
        # These will be placed on the table surface to create occlusion scenarios
        # Z position is table_surface_height + half cube height
        occluder_size = [0.15, 0.15, 0.15]  # 15cm cubes
        occluder_z = table_surface_height + occluder_size[2] / 2
        
        occluder_positions = [
            [0.5, 0.2, occluder_z],   # Occluder 1: in front, can hide objects behind it
            [0.6, -0.1, occluder_z],  # Occluder 2: offset position
            [0.4, -0.2, occluder_z],  # Occluder 3: another position
        ]
        
        for i, pos in enumerate(occluder_positions):
            # Create a visual shape (box) for the occluder
            visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[s/2 for s in occluder_size],
                rgbaColor=[0.8, 0.3, 0.3, 1.0]  # Red-ish color for occluders
            )
            # Create collision shape
            collision_shape_id = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[s/2 for s in occluder_size]
            )
            # Create multi-body object
            occluder_id = p.createMultiBody(
                baseMass=0.5,  # 0.5 kg
                baseCollisionShapeIndex=collision_shape_id,
                baseVisualShapeIndex=visual_shape_id,
                basePosition=pos,
                baseOrientation=[0, 0, 0, 1]
            )
            
            name = f"occluder_{i+1}"
            self.objects[name] = ObjectInfo(
                object_id=occluder_id,
                name=name,
                position=np.array(pos),
                orientation=np.array([0, 0, 0, 1]),
                size=np.array(occluder_size),
                is_visible=True,
                is_occluder=True
            )
        
        # Create target cubes (smaller cubes that may be hidden)
        # Position some behind occluders to create occlusion scenarios
        # Z position is table_surface_height + half cube height
        target_size = [0.08, 0.08, 0.08]  # 8cm cubes (smaller than occluders)
        target_z = table_surface_height + target_size[2] / 2
        
        target_positions = [
            [0.5, 0.4, target_z],   # Target 1: behind occluder_1 (may be occluded)
            [0.6, 0.1, target_z],   # Target 2: partially visible
            [0.4, -0.1, target_z],  # Target 3: visible
            [0.7, -0.2, target_z],  # Target 4: visible
        ]
        
        for i, pos in enumerate(target_positions):
            # Create visual shape for target cube
            visual_shape_id = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[s/2 for s in target_size],
                rgbaColor=[0.3, 0.3, 0.8, 1.0]  # Blue color for targets
            )
            # Create collision shape
            collision_shape_id = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[s/2 for s in target_size]
            )
            # Create multi-body object
            target_id = p.createMultiBody(
                baseMass=0.1,  # 0.1 kg (lighter than occluders)
                baseCollisionShapeIndex=collision_shape_id,
                baseVisualShapeIndex=visual_shape_id,
                basePosition=pos,
                baseOrientation=[0, 0, 0, 1]
            )
            
            name = f"target_{i+1}"
            self.objects[name] = ObjectInfo(
                object_id=target_id,
                name=name,
                position=np.array(pos),
                orientation=np.array([0, 0, 0, 1]),
                size=np.array(target_size),
                is_visible=False,  # Will be updated by oracle
                is_occluder=False
            )
        
        # Step simulation a few times to let objects settle on table
        for _ in range(10):
            p.stepSimulation()
    
    def generate_boxels(self, visible_objects: List[str]) -> List[Boxel]:
        """
        Generate Semantic Boxels for the currently visible objects and their occlusion shadows.
        
        This corresponds to the "Abstraction" step in the POD-TAMP pipeline.
        It converts continuous object poses/meshes into discrete semantic regions (Boxels).
        
        It also calculates "Shadow Boxels" (Occlusion Regions) for visible objects.
        
        Args:
            visible_objects (List[str]): A list of object names (e.g. ['occluder_1', 'target_4'])
                                         that are currently detected by the perception system.
            
        Returns:
            List[Boxel]: A list of Boxel objects representing the semantic abstraction 
                         of the visible scene AND the unknown occluded regions.
        """
        boxels = []
        
        # 1. First Pass: Generate all "Solid" Object Boxels
        solid_boxels = []
        for obj_name in visible_objects:
            if obj_name not in self.objects:
                continue
                
            obj_info = self.objects[obj_name]
            
            # Retrieve the Axis-Aligned Bounding Box (AABB) from PyBullet
            aabb_min, aabb_max = p.getAABB(obj_info.object_id)
            
            aabb_min = np.array(aabb_min)
            aabb_max = np.array(aabb_max)
            
            # Calculate geometric properties
            center = (aabb_min + aabb_max) / 2.0
            extent = (aabb_max - aabb_min) / 2.0  # Half-extents
            
            # Create the Object Boxel
            obj_boxel = Boxel(
                center=center,
                extent=extent,
                object_name=obj_name,
                is_occluded=False,
                is_shadow=False
            )
            solid_boxels.append(obj_boxel)
            
        boxels.extend(solid_boxels)

        # 2. Second Pass: Generate Shadow Boxels (considering other objects as obstacles)
        for obj_boxel in solid_boxels:
            # Get potential obstacles (all solid boxels except the one casting the shadow)
            obstacles = [b for b in solid_boxels if b.object_name != obj_boxel.object_name]
            
            # Calculate shadow(s)
            shadow_parts = self.calculate_shadow_boxel(obj_boxel, obstacles)
            boxels.extend(shadow_parts)
            
        return boxels

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
        # Find the "back" face of the object relative to the camera.
        # Only use the 4 corners of the back face to cast rays.
        # This prevents the shadow from extending into areas that ARE visible.
        
        # Direction from camera to object center
        cam_to_obj = obj_center - cam_pos
        cam_to_obj_norm = cam_to_obj / np.linalg.norm(cam_to_obj)
        
        # For each axis, determine if the "back" side is + or - 
        # If camera is looking in +X direction, back face is at +X (max X)
        # If camera is looking in -X direction, back face is at -X (min X)
        back_signs = np.sign(cam_to_obj_norm)  # [sign_x, sign_y, sign_z]
        
        # Generate only the 4 "back" corners
        # A corner is on the back if its offset aligns with back_signs on the dominant axes
        # Actually, we want corners where the offset matches the sign for EACH axis
        # But that only gives us 1 corner (the furthest corner).
        
        # Better approach: Use the 4 corners of the dominant back FACE.
        # The dominant axis is where |cam_to_obj| is largest.
        # Actually for proper shadow, we should use silhouette corners.
        
        # Simplified: Use corners that are "behind" the object center from camera's view.
        # A corner is "behind" if (corner - obj_center) dot cam_to_obj > 0
        
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
            # Dot product: positive means corner is in the direction camera is looking
            dot = np.dot(offset, cam_to_obj_norm)
            if dot > 0:  # Corner is on the "back" side
                back_corners.append(corner)
        
        # If we somehow got no back corners (shouldn't happen), fall back to all
        if len(back_corners) == 0:
            back_corners = all_corners
            
        corners = back_corners
        
        # Construct rays from camera through back corners
        ray_starts = [cam_pos] * len(corners)
        
        # Extend rays far enough to hit table/background
        max_dist = 5.0 # meters
        ray_ends = []
        for corner in corners:
            direction = corner - cam_pos
            direction = direction / np.linalg.norm(direction)
            # Start the ray AT the corner and shoot outward.
            ray_target = corner + direction * max_dist
            ray_ends.append(ray_target)
            
        # Batch Ray Test (starting from back corners outward)
        results = p.rayTestBatch(corners, ray_ends)
        
        # Define table bounds (based on scene setup)
        # Table is at [0.5, 0.0] with size ~1.0x1.0
        table_x_min = 0.0
        table_x_max = 1.0
        table_y_min = -0.5
        table_y_max = 0.5
        table_z = self.table_surface_height
        
        hit_points = []
        for i, res in enumerate(results):
            # res: objectUid, linkIndex, hitFraction, hitPosition, hitNormal
            hit_obj_id = res[0]
            if hit_obj_id != -1:
                hit_pt = np.array(res[3])
                # Even for hits, clamp to table bounds to be safe
                hit_pt[0] = np.clip(hit_pt[0], table_x_min, table_x_max)
                hit_pt[1] = np.clip(hit_pt[1], table_y_min, table_y_max)
                hit_pt[2] = max(hit_pt[2], table_z)
                hit_points.append(hit_pt)
            else:
                # No hit (ray went off table or into sky).
                # Project ray to table plane and clamp to table boundaries.
                
                start = corners[i]
                direction = ray_ends[i] - start
                direction = direction / np.linalg.norm(direction)
                
                # Intersect with Table Plane (Z = table_z)
                # Ray: P(t) = start + t * dir
                # start.z + t * dir.z = table_z  =>  t = (table_z - start.z) / dir.z
                
                if abs(direction[2]) > 1e-6 and direction[2] < 0:
                    # Ray is going downward towards table
                    t = (table_z - start[2]) / direction[2]
                    if t > 0:
                        pt_on_plane = start + direction * t
                    else:
                        # Intersection behind start, use start projected down
                        pt_on_plane = np.array([start[0], start[1], table_z])
                else:
                    # Ray is going up or parallel - just project start point down
                    pt_on_plane = np.array([start[0], start[1], table_z])
                
                # Clamp to Table Bounds (X and Y)
                clamped_pt = pt_on_plane.copy()
                clamped_pt[0] = np.clip(clamped_pt[0], table_x_min, table_x_max)
                clamped_pt[1] = np.clip(clamped_pt[1], table_y_min, table_y_max)
                clamped_pt[2] = table_z
                
                hit_points.append(clamped_pt)
        
        # --- Step 2: Construct Initial Shadow AABB ---
        # The shadow volume is bounded by the Object's back face (approx as corners)
        # and the Hit Points.
        # But we want the shadow to START at the object back.
        # A simple AABB approximation is the bounding box of (Object Corners U Hit Points)
        # BUT adjusted so it doesn't include the object itself (as much as possible).
        
        # Actually, the problem states: "shadow should not include the object itself. 
        # it should start from the backspace of the bounding boxel."
        
        # Let's compute the bounding box of the HIT points first.
        # Then extend it back towards the object, but stop AT the object.
        
        # To ensure we don't include the object:
        # We can form a box from min(hit_points) to max(hit_points)
        # And union it with the object's "back" face.
        
        # Simplest approach satisfying "overestimate height":
        # 1. Get bounds of hit points
        h_min = np.min(hit_points, axis=0)
        h_max = np.max(hit_points, axis=0)
        
        # Force shadow bottom to be at least table surface height
        h_min[2] = max(h_min[2], self.table_surface_height)
        
        # 2. Get bounds of object (start)
        o_min = obj_center - obj_extent
        o_max = obj_center + obj_extent
        
        # 3. Combine. The shadow generally extends from Object to Hits.
        # Direction of shadow is roughly Hit_Center - Obj_Center.
        
        # For AABB representation, we essentially union the two sets of points,
        # but then we trim the part that overlaps the object.
        # However, AABBs can't represent arbitrary frustums.
        # We will make an AABB that covers the hits and touches the object.
        
        full_min = np.minimum(o_min, h_min)
        full_max = np.maximum(o_max, h_max)
        
        # Clamp shadow bounds to table boundaries
        full_min[0] = max(full_min[0], table_x_min)
        full_min[1] = max(full_min[1], table_y_min)
        full_min[2] = max(full_min[2], table_z)
        full_max[0] = min(full_max[0], table_x_max)
        full_max[1] = min(full_max[1], table_y_max)
        
        # Enforce Height Overestimate (Constraint 4)
        # "height should be an overestimate... at least as high as the original object"
        full_max[2] = max(full_max[2], o_max[2])
        
        # Now, "subtract" the object from this full box.
        # Since we are using AABBs, we can't perfectly subtract.
        # But usually the shadow is "behind".
        # We can find the dominant axis of the shadow direction and chop off the object part.
        
        shadow_dir = np.mean(hit_points, axis=0) - obj_center
        dom_axis = np.argmax(np.abs(shadow_dir))
        
        # Copy full bounds
        s_min = full_min.copy()
        s_max = full_max.copy()
        
        # Trim start based on dominant direction
        if shadow_dir[dom_axis] > 0:
            # Shadow is in +Axis direction (e.g. +X)
            # So shadow starts at Object Max X
            s_min[dom_axis] = max(s_min[dom_axis], o_max[dom_axis])
        else:
            # Shadow is in -Axis direction
            # So shadow ends at Object Min X
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
                # Check intersection
                if self._check_aabb_intersection(shadow, obstacle):
                    # Subtract/Split
                    fragments = self._subtract_aabb(shadow, obstacle, shadow_dir)
                    next_active.extend(fragments)
                else:
                    next_active.append(shadow)
            active_shadows = next_active
            
        return active_shadows

    def _check_aabb_intersection(self, b1: Boxel, b2: Boxel) -> bool:
        min1 = b1.center - b1.extent
        max1 = b1.center + b1.extent
        min2 = b2.center - b2.extent
        max2 = b2.center + b2.extent
        
        return (np.all(min1 <= max2) and np.all(max1 >= min2))

    def _subtract_aabb(self, shadow: Boxel, obstacle: Boxel, direction: np.ndarray) -> List[Boxel]:
        """
        Subtract obstacle from shadow, keeping parts 'before' the obstacle
        and 'around' it, but discarding parts 'behind' or 'inside'.
        
        Strategy: Split shadow into up to 6 fragments based on obstacle planes.
        """
        s_min = shadow.center - shadow.extent
        s_max = shadow.center + shadow.extent
        o_min = obstacle.center - obstacle.extent
        o_max = obstacle.center + obstacle.extent
        
        fragments = []
        
        # Determine "Front" of obstacle relative to shadow direction
        # If shadow moves +X, front is Min X. If -X, front is Max X.
        # We want to keep the shadow segment that is "Upstream" (closer to light source)
        
        # We can generate potential fragments by slicing the shadow AABB
        # against the 6 planes of the obstacle AABB.
        
        # 1. Left of Obstacle (Min X)
        if s_min[0] < o_min[0]:
            # Create fragment [s_min.x, o_min.x] x [s_y] x [s_z]
            new_max = s_max.copy()
            new_max[0] = o_min[0]
            fragments.append(self._create_boxel_from_bounds(s_min, new_max, shadow))
            # Shrink remaining shadow to start at o_min[0]
            s_min[0] = max(s_min[0], o_min[0])
            
        # 2. Right of Obstacle (Max X)
        if s_max[0] > o_max[0]:
            # Create fragment [o_max.x, s_max.x]
            new_min = s_min.copy()
            new_min[0] = o_max[0]
            fragments.append(self._create_boxel_from_bounds(new_min, s_max, shadow))
            # Shrink remaining
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
            
        # The remaining part (s_min to s_max) is effectively INSIDE the obstacle 
        # (intersection core).
        # We generally discard this core because the shadow stops AT the surface.
        
        # However, we must filter these fragments. 
        # Some are "side" shadows (passing by), some are "upstream" (before hitting).
        # We keep all "side" fragments.
        # The "downstream" fragments (shadow continuing AFTER obstacle) should be removed
        # because the obstacle blocks the light.
        
        filtered_fragments = []
        for frag in fragments:
            if frag is not None and not self._is_downstream(frag, obstacle, direction):
                filtered_fragments.append(frag)
                
        return filtered_fragments

    def _create_boxel_from_bounds(self, min_pt, max_pt, template_boxel):
        center = (min_pt + max_pt) / 2.0
        extent = (max_pt - min_pt) / 2.0
        # Sanity check for negative or very small dimensions (minimum 1mm)
        MIN_EXTENT = 0.001  # 1mm minimum size
        if np.any(extent <= 0) or np.any(extent < MIN_EXTENT):
            return None  # Filter out degenerate/tiny boxels
            
        return Boxel(
            center=center,
            extent=extent,
            object_name=template_boxel.object_name,
            is_occluded=True,
            is_shadow=True
        )

    def _is_downstream(self, frag: Boxel, obstacle: Boxel, direction: np.ndarray) -> bool:
        """
        Check if a fragment is 'behind' the obstacle relative to light direction.
        Actually, simplified logic:
        We decomposed the intersection. The fragments generated are the parts of the 
        shadow that are *outside* the obstacle.
        We want to keep:
        1. Parts *before* the obstacle (upstream).
        2. Parts *beside* the obstacle (passing by).
        We want to discard:
        3. Parts *after* the obstacle (downstream) - because the obstacle blocks the light 
           so the shadow shouldn't exist there? 
           Wait, if the original big shadow volume represents the "potential shadow" 
           of Object A, and Object B is inside that volume:
           - The shadow exists UP TO Object B.
           - The shadow DOES NOT exist INSIDE Object B.
           - The shadow DOES NOT exist BEHIND Object B (because B is now casting its own shadow,
             or simply because A's shadow is blocked by B's surface).
             
        So essentially, we just want to remove the volume "Behind" B.
        
        But the `_subtract_aabb` method above essentially carved out the core intersection.
        The fragments returned are everything *except* the intersection.
        This includes the "upstream", "downstream", and "sides".
        
        We need to identify which fragments are "downstream".
        
        A fragment is downstream if it is positioned further along the direction vector 
        than the obstacle.
        """
        # Simple centroid check against obstacle centroid projected on direction
        # Or check bounds.
        
        # Let's look at the dominant axis again.
        dom_axis = np.argmax(np.abs(direction))
        sign = np.sign(direction[dom_axis])
        
        f_center = frag.center[dom_axis]
        o_center = obstacle.center[dom_axis]
        
        if sign > 0:
            # Moving +X. Downstream means X > Obstacle X
            # Use strict bound check: Fragment Min > Obstacle Max
            f_min = frag.center[dom_axis] - frag.extent[dom_axis]
            o_max = obstacle.center[dom_axis] + obstacle.extent[dom_axis]
            if f_min >= o_max - 1e-4:
                return True
        else:
            # Moving -X. Downstream means X < Obstacle X
            # Fragment Max < Obstacle Min
            f_max = frag.center[dom_axis] + frag.extent[dom_axis]
            o_min = obstacle.center[dom_axis] - obstacle.extent[dom_axis]
            if f_max <= o_min + 1e-4:
                return True
                
        return False


    def generate_free_space(self, known_boxels: List[Boxel], visualize: bool = False) -> List[Boxel]:
        """
        Discretize the free space using an Octree (Breadth-First Search).
        
        Args:
            known_boxels: List of Object and Shadow Boxels
            visualize: If True, animates the generation process (1s per depth layer)
            
        Returns:
            List of Boxels representing the free space
        """
        # Define Workspace Bounds (Volume above table)
        ws_min = np.array([0.0, -0.5, self.table_surface_height])
        ws_max = np.array([1.0, 0.5, self.table_surface_height + 0.5])
        
        root_center = (ws_min + ws_max) / 2.0
        root_extent = (ws_max - ws_min) / 2.0
        
        root = OctreeNode(root_center, root_extent)
        free_boxels = []
        min_size = 0.05 # 5cm minimum resolution
        
        # Helper to get bounds of a boxel
        def get_bounds(b: Boxel):
            return b.center - b.extent, b.center + b.extent
        
        # Pre-compute bounds for known boxels
        known_bounds = [get_bounds(b) for b in known_boxels]
        
        # BFS Queue (List of OctreeNodes)
        current_layer = [root]
        
        # Track which free boxels we've already drawn (to avoid redrawing)
        drawn_free_boxels = set()
        
        while current_layer:
            next_layer = []
            
            for node in current_layer:
                # Check intersection with any known boxel
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
                    
                    # Track this boxel for visualization (only draw if new)
                    boxel_key = (tuple(node.center), tuple(node.extent))
                    if visualize and boxel_key not in drawn_free_boxels:
                        drawn_free_boxels.add(boxel_key)
                        # Draw this new free boxel immediately (cyan)
                        c = node.center
                        e = node.extent
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
                                lineColorRGB=[0, 1, 1],  # Cyan
                                lineWidth=1.0,
                                lifeTime=0
                            )
                            self.debug_items.append(line_id)
                    
                    continue # Stop processing this branch
                
                # If Mixed/Occupied, check size
                max_dim = np.max(node.extent * 2)
                if max_dim <= min_size:
                    node.state = 'OCCUPIED'
                    continue # Stop (too small)
                
                # Split (Mixed and big enough)
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
            
            # Visualization Step (End of Layer)
            if visualize:
                # Strategy: 
                # 1. Clear only previous yellow candidates (they change each iteration)
                # 2. Free boxels are drawn immediately when created (above), so they persist
                # 3. Draw new yellow candidates for next layer
                
                # Clear previous yellow candidates
                for item_id in self.candidate_debug_items:
                    p.removeUserDebugItem(item_id)
                self.candidate_debug_items = []
                
                # Draw current candidates (yellow) - store IDs separately so we can clear them next iteration
                for node in next_layer:
                    c = node.center
                    e = node.extent
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
                            lineColorRGB=[1, 1, 0],  # Yellow
                            lineWidth=1.0,
                            lifeTime=0
                        )
                        self.candidate_debug_items.append(line_id)  # Track separately
                
                time.sleep(1.0) # 1 second per step
            
            # Move to next layer
            current_layer = next_layer

        return free_boxels

    def clear_all_debug_items(self):
        """Clear all debug lines and shadow bodies."""
        # Remove any existing shadow bodies
        for body_id in self.shadow_bodies:
            p.removeBody(body_id)
        self.shadow_bodies = []
        
        # Remove any existing debug lines
        for item_id in self.debug_items:
            p.removeUserDebugItem(item_id)
        self.debug_items = []
        
        # Remove candidate debug items
        for item_id in self.candidate_debug_items:
            p.removeUserDebugItem(item_id)
        self.candidate_debug_items = []
    
    def draw_boxels(self, boxels: List[Boxel], duration: float = 0, clear_previous: bool = True):
        """
        Visualize Semantic Boxels in the PyBullet GUI using debug lines.
        
        This overlay allows us to verify that the semantic abstraction matches
        the physical reality.
        
        Args:
            boxels (List[Boxel]): The list of Boxel objects to visualize.
            duration (float): How long the lines should remain visible in seconds.
                              0 = forever (until manually removed).
                              0.1 = good for dynamic updates in a loop.
            clear_previous (bool): If True, clears all previous debug items before drawing.
                                  If False, accumulates (useful for animated sequences).
        """
        # Remove any existing shadow bodies from previous frame
        for body_id in self.shadow_bodies:
            p.removeBody(body_id)
        self.shadow_bodies = []
        
        # Optionally remove existing debug lines (default: yes, for clean updates)
        if clear_previous:
            for item_id in self.debug_items:
                p.removeUserDebugItem(item_id)
            self.debug_items = []
        
        for boxel in boxels:
            # --- Draw Wireframe (for all boxels) ---
            c = boxel.center
            e = boxel.extent
            
            # Define the 8 corners of the box relative to center
            corners = [
                c + np.array([-e[0], -e[1], -e[2]]), # 0: Bottom-Left-Back
                c + np.array([e[0], -e[1], -e[2]]),  # 1: Bottom-Right-Back
                c + np.array([-e[0], e[1], -e[2]]),  # 2: Top-Left-Back
                c + np.array([e[0], e[1], -e[2]]),   # 3: Top-Right-Back
                c + np.array([-e[0], -e[1], e[2]]),  # 4: Bottom-Left-Front
                c + np.array([e[0], -e[1], e[2]]),   # 5: Bottom-Right-Front
                c + np.array([-e[0], e[1], e[2]]),   # 6: Top-Left-Front
                c + np.array([e[0], e[1], e[2]])     # 7: Top-Right-Front
            ]
            
            # Define connections between corners to draw a box
            edges = [
                (0, 1), (0, 2), (0, 4), # Edges from corner 0
                (1, 3), (1, 5),         # From corner 1
                (2, 3), (2, 6),         # From corner 2
                (3, 7),                 # From corner 3
                (4, 5), (4, 6),         # From corner 4
                (5, 7),                 # From corner 5
                (6, 7)                  # From corner 6
            ]
            
            # Semantic Color Coding:
            # Red    = Occluder (Obstacle)
            # Blue   = Target (Goal)
            # Gray   = Shadow (Unknown/Occluded Region)
            # Cyan   = Free Space
            # Yellow = Candidate (Processing)
            # Green  = Other
            
            if boxel.is_candidate:
                color = [1, 1, 0] # Yellow for processing candidates
            elif boxel.is_shadow:
                color = [0.5, 0.5, 0.5] # Gray for shadows
            elif boxel.is_free:
                color = [0, 1, 1] # Cyan for free space
            elif boxel.object_name and boxel.object_name.startswith("occluder"):
                color = [1, 0, 0] # Red for occluders
            elif boxel.object_name and boxel.object_name.startswith("target"):
                color = [0, 0, 1] # Blue for targets
            else:
                color = [0, 1, 0] # Green for others
                
            # Draw the lines
            # Free space lines should be thin
            is_thin = boxel.is_shadow or boxel.is_free or boxel.is_candidate
            
            for start_idx, end_idx in edges:
                line_id = p.addUserDebugLine(
                    lineFromXYZ=corners[start_idx],
                    lineToXYZ=corners[end_idx],
                    lineColorRGB=color,
                    lineWidth=1.0 if is_thin else 2.0, 
                    lifeTime=duration
                )
                self.debug_items.append(line_id)
            
            # --- Draw Filled Phantom (for shadow boxels only) ---
            if boxel.is_shadow:
                # Create a visual-only body (phantom)
                # Create visual shape (box)
                visual_shape_id = p.createVisualShape(
                    shapeType=p.GEOM_BOX,
                    halfExtents=e,
                    rgbaColor=[0.5, 0.5, 0.5, 0.3], # Semi-transparent gray
                    specularColor=[0, 0, 0] # No specular
                )
                
                # Create multi-body (visual only, no collision)
                shadow_body_id = p.createMultiBody(
                    baseMass=0, # Static (no mass)
                    baseVisualShapeIndex=visual_shape_id,
                    basePosition=c,
                    baseOrientation=[0, 0, 0, 1] # Aligned with world
                )
                
                self.shadow_bodies.append(shadow_body_id)
    
    def get_camera_observation(self) -> CameraObservation:
        """
        Capture an observation from the fixed camera.
        
        This function:
        1. Renders RGB and depth images from the camera
        2. Generates a point cloud from the depth image
        3. Uses the oracle to detect visible objects
        4. Generates Boxels for visible objects AND their shadows
        5. Returns all observation data
        
        Returns:
            CameraObservation containing RGB, depth, point cloud, and object information
        """
        # Compute view and projection matrices
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=self.camera_position,
            cameraTargetPosition=self.camera_target,
            cameraUpVector=self.camera_up
        )
        
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=self.image_width / self.image_height,
            nearVal=self.near_plane,
            farVal=self.far_plane
        )
        
        # Render camera image
        # Returns: width, height, rgb, depth, segmentation
        # NOTE: When using OpenGL hardware rendering (p.ER_BULLET_HARDWARE_OPENGL),
        # PyBullet may spawn "Synthetic Camera" preview windows. These are useful for debugging
        # but can be ignored or minimized.
        _, _, rgb_array, depth_array, _ = p.getCameraImage(
            width=self.image_width,
            height=self.image_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # Convert RGB array to numpy and reshape
        # PyBullet returns flattened array: [R, G, B, A, R, G, B, A, ...]
        rgb_image = np.array(rgb_array, dtype=np.uint8)
        rgb_image = rgb_image.reshape((self.image_height, self.image_width, 4))
        rgb_image = rgb_image[:, :, :3]  # Remove alpha channel, keep RGB
        
        # Convert depth array to numpy and transform to actual depth in meters
        # PyBullet returns depth as flattened array, reshape to image dimensions
        depth_image = np.array(depth_array)
        depth_image = depth_image.reshape((self.image_height, self.image_width))
        # Convert from [0, 1] to actual depth using projection matrix
        depth_image = self._depth_buffer_to_meters(depth_image)
        
        # Generate point cloud from depth image
        point_cloud = self._depth_to_point_cloud(depth_image, view_matrix, projection_matrix)
        
        # Use oracle to detect visible objects
        visible_objects, object_poses = self.oracle_detect_objects()
        
        # Generate Boxels for visible objects and their shadows
        boxels = self.generate_boxels(visible_objects)
        
        return CameraObservation(
            rgb_image=rgb_image,
            depth_image=depth_image,
            point_cloud=point_cloud,
            visible_objects=visible_objects,
            object_poses=object_poses,
            boxels=boxels
        )
    
    def _depth_buffer_to_meters(self, depth_buffer: np.ndarray) -> np.ndarray:
        """
        Convert depth buffer values to actual depth in meters.
        
        PyBullet returns depth in normalized device coordinates [0, 1].
        This function converts to actual depth in meters using the projection matrix.
        
        Args:
            depth_buffer: Depth buffer from PyBullet (normalized [0, 1])
            
        Returns:
            Depth values in meters
        """
        # PyBullet depth is in normalized device coordinates
        # Convert to actual depth using near and far planes
        depth_meters = self.far_plane * self.near_plane / (
            self.far_plane - (self.far_plane - self.near_plane) * depth_buffer
        )
        return depth_meters
    
    def _depth_to_point_cloud(
        self,
        depth_image: np.ndarray,
        view_matrix: np.ndarray,
        projection_matrix: np.ndarray
    ) -> np.ndarray:
        """
        Convert depth image to 3D point cloud in world coordinates.
        
        This function:
        1. Creates a grid of pixel coordinates
        2. Unprojects each pixel using depth and camera matrices
        3. Transforms points from camera frame to world frame
        
        Args:
            depth_image: Depth image in meters (H, W)
            view_matrix: Camera view matrix (4x4)
            projection_matrix: Camera projection matrix (4x4)
            
        Returns:
            Point cloud as numpy array (N, 3) in world coordinates
        """
        height, width = depth_image.shape
        
        # Create pixel coordinate grid
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        # Convert to normalized device coordinates
        x_ndc = (2.0 * u / width) - 1.0
        y_ndc = 1.0 - (2.0 * v / height)  # Flip Y axis
        z_ndc = depth_image.flatten()
        
        # Unproject to camera space
        # Using simplified unprojection (assuming perspective projection)
        fx = fy = (width / 2.0) / np.tan(np.radians(self.fov / 2.0))
        cx = width / 2.0
        cy = height / 2.0
        
        x_cam = (u.flatten() - cx) * z_ndc / fx
        y_cam = (v.flatten() - cy) * z_ndc / fy
        z_cam = z_ndc
        
        # Stack camera coordinates
        points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
        
        # Transform from camera to world coordinates
        # Extract rotation and translation from view matrix
        view_matrix_4x4 = np.array(view_matrix).reshape(4, 4)
        camera_to_world = np.linalg.inv(view_matrix_4x4)
        
        # Convert to homogeneous coordinates
        points_cam_homogeneous = np.hstack([points_cam, np.ones((points_cam.shape[0], 1))])
        
        # Transform to world coordinates
        points_world_homogeneous = (camera_to_world @ points_cam_homogeneous.T).T
        points_world = points_world_homogeneous[:, :3]
        
        # Filter out invalid points (too far or too close)
        valid_mask = (points_world[:, 2] > -1.0) & (points_world[:, 2] < 2.0)
        points_world = points_world[valid_mask]
        
        return points_world
    
    def oracle_detect_objects(
        self,
        check_occlusion: bool = True
    ) -> Tuple[List[str], Dict[str, Tuple[np.ndarray, np.ndarray]]]:
        """
        Oracle function to detect visible objects and their poses.
        
        This is an "oracle" because it has perfect knowledge of object locations.
        In a real system, this would be replaced by actual perception (e.g., object detection
        from RGB-D images). For now, this function:
        1. Gets actual object poses from PyBullet
        2. Optionally checks if objects are occluded using ray casting
        3. Returns list of visible objects and their poses
        
        Args:
            check_occlusion: If True, uses ray casting to check if objects are occluded
            
        Returns:
            Tuple of:
            - List of visible object names
            - Dictionary mapping object names to (position, orientation) tuples
        """
        visible_objects = []
        object_poses = {}
        
        # Get current object poses from PyBullet
        for name, obj_info in self.objects.items():
            # Skip plane, table, and robot for now (focus on manipulable objects)
            if name in ["plane", "table", "robot"]:
                continue
            
            # Get current pose
            pos, orn = p.getBasePositionAndOrientation(obj_info.object_id)
            position = np.array(pos)
            orientation = np.array(orn)
            
            # Update object info
            obj_info.position = position
            obj_info.orientation = orientation
            
            # Check visibility using ray casting from camera to object center
            is_visible = True
            if check_occlusion:
                # Cast ray from camera to object center
                ray_from = self.camera_position
                ray_to = position
                
                # Perform ray test
                ray_result = p.rayTest(ray_from, ray_to)
                
                if len(ray_result) > 0:
                    # Check if the first hit is the object itself
                    hit_object_id = ray_result[0][0]
                    hit_fraction = ray_result[0][2]
                    
                    # If hit fraction is close to 1.0, object is directly visible
                    # If something else is hit first, object is occluded
                    if hit_object_id != obj_info.object_id and hit_fraction < 0.95:
                        is_visible = False
                        # Check if the occluder is actually blocking (not just a near miss)
                        occluder_pos, _ = p.getBasePositionAndOrientation(hit_object_id)
                        occluder_dist = np.linalg.norm(np.array(occluder_pos) - ray_from)
                        obj_dist = np.linalg.norm(position - ray_from)
                        if occluder_dist < obj_dist:
                            is_visible = False
                        else:
                            is_visible = True
                    else:
                        is_visible = True
                else:
                    is_visible = True
            
            obj_info.is_visible = is_visible
            
            if is_visible:
                visible_objects.append(name)
            
            # Always store pose (even if occluded) for oracle knowledge
            object_poses[name] = (position.copy(), orientation.copy())
        
        return visible_objects, object_poses
    
    def get_object_point_cloud(self, object_name: str) -> Optional[np.ndarray]:
        """
        Get point cloud for a specific object using oracle knowledge.
        
        This function generates a point cloud representation of an object by:
        1. Getting the object's bounding box
        2. Sampling points within the bounding box
        3. Transforming to world coordinates
        
        Args:
            object_name: Name of the object (e.g., "target_1", "occluder_1")
            
        Returns:
            Point cloud as numpy array (N, 3) in world coordinates, or None if object not found
        """
        if object_name not in self.objects:
            return None
        
        obj_info = self.objects[object_name]
        
        # Get object pose
        pos, orn = p.getBasePositionAndOrientation(obj_info.object_id)
        position = np.array(pos)
        orientation = np.array(orn)
        
        # Sample points within the object's bounding box
        # For simplicity, we'll create a grid of points
        half_extents = obj_info.size / 2.0
        resolution = 0.02  # 2cm resolution
        
        # Create grid of points in object-local coordinates
        x = np.arange(-half_extents[0], half_extents[0] + resolution, resolution)
        y = np.arange(-half_extents[1], half_extents[1] + resolution, resolution)
        z = np.arange(-half_extents[2], half_extents[2] + resolution, resolution)
        
        xx, yy, zz = np.meshgrid(x, y, z)
        points_local = np.stack([xx.flatten(), yy.flatten(), zz.flatten()], axis=1)
        
        # Transform to world coordinates
        # Convert quaternion to rotation matrix
        quat = orientation
        rotation_matrix = self._quaternion_to_rotation_matrix(quat)
        
        # Apply rotation and translation
        points_world = (rotation_matrix @ points_local.T).T + position
        
        return points_world
    
    def _quaternion_to_rotation_matrix(self, quat: np.ndarray) -> np.ndarray:
        """
        Convert quaternion [x, y, z, w] to rotation matrix.
        
        Args:
            quat: Quaternion as [x, y, z, w]
            
        Returns:
            3x3 rotation matrix
        """
        x, y, z, w = quat
        rotation_matrix = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])
        return rotation_matrix
    
    def step_simulation(self, num_steps: int = 1):
        """
        Step the simulation forward.
        
        Args:
            num_steps: Number of simulation steps to take
        """
        for _ in range(num_steps):
            p.stepSimulation()
    
    def reset(self):
        """
        Reset the environment to initial state.
        
        This removes all objects and recreates the scene.
        """
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.objects.clear()
        self._setup_scene()
    
    def close(self):
        """Close the PyBullet connection."""
        p.disconnect(self.client_id)


def save_point_cloud_to_ply(points: np.ndarray, filename: str):
    """
    Save point cloud to a PLY file.
    
    Args:
        points: Numpy array of shape (N, 3) containing point coordinates
        filename: Output filename (should end with .ply)
    """
    print(f"Saving point cloud with {len(points)} points to {filename}...")
    
    header = f"""ply
format ascii 1.0
element vertex {len(points)}
property float x
property float y
property float z
end_header
"""
    with open(filename, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")
    print("Save complete.")


def main():
    """
    Main function to test the Boxel Test Environment.
    
    This function:
    1. Creates the environment
    2. Runs simulation for a few seconds
    3. Captures camera observations
    4. Tests oracle functions
    5. Saves point clouds to files
    6. Prints results
    """
    print("=" * 60)
    print("Boxel Test Environment - Test Run")
    print("=" * 60)
    
    # Create environment
    env = BoxelTestEnv(gui=True)
    
    # Let objects settle
    print("\nLetting objects settle...")
    for _ in range(100):
        env.step_simulation()
        time.sleep(1.0 / 240.0)
    
    # Get camera observation
    print("\nCapturing camera observation...")
    obs = env.get_camera_observation()
    
    print(f"\nCamera Observation Results:")
    print(f"  RGB image shape: {obs.rgb_image.shape}")
    print(f"  Depth image shape: {obs.depth_image.shape}")
    print(f"  Point cloud shape: {obs.point_cloud.shape}")
    print(f"  Visible objects: {obs.visible_objects}")
    print(f"  Total objects detected: {len(obs.object_poses)}")
    print(f"  Boxels generated: {len(obs.boxels) if obs.boxels else 0}")
    
    # Save the scene point cloud
    save_point_cloud_to_ply(obs.point_cloud, "scene_point_cloud.ply")
    
    # Test oracle detection
    print("\nOracle Detection Results:")
    visible, poses = env.oracle_detect_objects(check_occlusion=True)
    print(f"  Visible objects: {visible}")
    print(f"  Total objects: {list(poses.keys())}")
    
    for obj_name in visible:
        pos, orn = poses[obj_name]
        print(f"    {obj_name}: position={pos}, orientation={orn}")
    
    # Test point cloud generation for a target object
    print("\nTesting point cloud generation...")
    target_pc = env.get_object_point_cloud("target_1")
    if target_pc is not None:
        print(f"  Generated point cloud for target_1: {target_pc.shape} points")
        save_point_cloud_to_ply(target_pc, "target_1_point_cloud.ply")
    
    # Run simulation for visualization
    print("\nRunning visualization sequence...")
    print("(Close the PyBullet window or press Ctrl+C to exit)")
    
    try:
        # Separate Objects and Shadows
        all_known = obs.boxels
        obj_boxels = [b for b in all_known if not b.is_shadow]
        shadow_boxels = [b for b in all_known if b.is_shadow]
        
        # DEBUG: Print all boxels to find the weird blue line
        print("\n=== DEBUG: All Generated Boxels ===")
        for i, b in enumerate(all_known):
            boxel_type = "SHADOW" if b.is_shadow else "OBJECT"
            print(f"  [{i}] {boxel_type}: {b.object_name}")
            print(f"       center: {b.center}")
            print(f"       extent: {b.extent}")
            # Flag suspicious boxels (very thin in any dimension)
            if np.any(b.extent < 0.01):
                print(f"       *** WARNING: Very thin boxel! ***")
            if np.any(b.extent > 1.0):
                print(f"       *** WARNING: Very large extent! ***")
        print("=== END DEBUG ===\n")
        
        # Timing constants
        PHASE_WAIT_SECONDS = 0  # Set to 0 for fast debug, 1.0 for normal viewing
        FINAL_HOLD_SECONDS = 10  # How long to keep window open at end
        ENABLE_FREE_SPACE = True  # Whether to run free space discretization
        
        # Set camera to view the table clearly
        table_center = np.array([0.5, 0.0, env.table_surface_height])
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=table_center
        )
        
        # Phase 1: Show Objects
        print("Phase 1: Objects")
        env.draw_boxels(obj_boxels, duration=0)
        for _ in range(int(240 * PHASE_WAIT_SECONDS)): 
            env.step_simulation()
            time.sleep(1.0/240.0)
            
        # Phase 2: Show Shadows
        print("Phase 2: Shadows")
        env.draw_boxels(all_known, duration=0)
        for _ in range(int(240 * PHASE_WAIT_SECONDS)):
            env.step_simulation()
            time.sleep(1.0/240.0)
            
        # Phase 3: Generate Free Space
        print("Phase 3: Free Space Discretization")
        env.clear_all_debug_items()
        env.draw_boxels(all_known, duration=0, clear_previous=True)
        if ENABLE_FREE_SPACE:
            # Run discretization (visualize=False for speed, we'll draw results after)
            free_boxels = env.generate_free_space(all_known, visualize=False)
            # Draw the final free space boxels
            env.draw_boxels(all_known + free_boxels, duration=0, clear_previous=True)
        
        # Phase 4: Hold Result
        print("Phase 4: Hold Result")
        for _ in range(int(240 * PHASE_WAIT_SECONDS * 3)):
            env.step_simulation()
            time.sleep(1.0/240.0)
            
        # Phase 5: Keep window open for FINAL_HOLD_SECONDS
        print(f"Visualization complete. Keeping window open for {FINAL_HOLD_SECONDS} seconds...")
        for _ in range(int(240 * FINAL_HOLD_SECONDS)):
            env.step_simulation()
            time.sleep(1.0/240.0)
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    # Close environment
    env.close()
    print("\nEnvironment closed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()
