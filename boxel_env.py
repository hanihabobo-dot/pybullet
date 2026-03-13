"""
Boxel Test Environment for Semantic POD-TAMP Research.

This module implements the main PyBullet simulation environment for testing
Semantic Partitioning for Partially Observable Deterministic Task and Motion Planning.
"""

import numpy as np
import pybullet as p
import pybullet_data
from typing import List, Dict, Tuple, Optional

from boxel_types import ObjectInfo, Boxel, CameraObservation
from shadow_calculator import ShadowCalculator
from free_space import FreeSpaceGenerator
from visualization import BoxelVisualizer


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
        far_plane: float = 5.0,
        window_width: int = 1280,
        window_height: int = 600
    ):
        """
        Initialize the Boxel test environment.
        
        Args:
            gui: Whether to show the PyBullet GUI
            camera_position: Position of the camera [x, y, z]
            camera_target: Point the camera looks at [x, y, z]
            camera_up: Camera up vector [x, y, z]
            image_width: Width of camera images in pixels
            image_height: Height of camera images in pixels
            fov: Field of view in degrees
            near_plane: Near clipping plane distance
            far_plane: Far clipping plane distance
            window_width: Width of the PyBullet GUI window
            window_height: Height of the PyBullet GUI window
        """
        # Store camera parameters
        self.image_width = image_width
        self.image_height = image_height
        self.fov = fov
        self.near_plane = near_plane
        self.far_plane = far_plane
        
        # Set default camera position
        if camera_position is None:
            camera_position = np.array([0.5, -0.8, 0.7])
        if camera_target is None:
            camera_target = np.array([0.5, 0.0, 0.5])
        if camera_up is None:
            camera_up = np.array([0, 0, 1])
        
        self.camera_position = camera_position
        self.camera_target = camera_target
        self.camera_up = camera_up
        
        # Connect to PyBullet with window size options
        if gui:
            self.client_id = p.connect(p.GUI, options=f"--width={window_width} --height={window_height}")
            # Disable mouse picking so left-click rotates camera instead of grabbing objects
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
            # Disable segmentation mask preview window
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        else:
            self.client_id = p.connect(p.DIRECT)
        
        # Reset simulation
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)  # standard gravitational acceleration
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Store object information
        self.objects: Dict[str, ObjectInfo] = {}
        
        # Initialize the scene
        self._setup_scene()
        
        # Initialize helper components
        self.shadow_calculator = ShadowCalculator(
            self.camera_position, self.table_surface_height,
            table_x_range=self.table_x_range, table_y_range=self.table_y_range
        )
        self.free_space_generator = FreeSpaceGenerator(
            self.table_surface_height,
            table_x_range=self.table_x_range, table_y_range=self.table_y_range
        )
        self.visualizer = BoxelVisualizer()
        
        # Initialize debug camera state for keyboard navigation
        self.debug_camera_distance = 1.5
        self.debug_camera_yaw = 45.0
        self.debug_camera_pitch = -30.0
        self.debug_camera_target = np.array([0.5, 0.0, self.table_surface_height])
        
        print("Boxel Test Environment initialized successfully!")
        print(f"Camera position: {self.camera_position}")
        print(f"Camera target: {self.camera_target}")
        print(f"Objects in scene: {list(self.objects.keys())}")
    
    def _setup_scene(self):
        """Set up the simulation scene with plane, table, robot, and objects."""
        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        self.objects["plane"] = ObjectInfo(
            object_id=plane_id, name="plane",
            position=np.array([0, 0, 0]), orientation=np.array([0, 0, 0, 1]),
            size=np.array([10, 10, 0.1]), is_visible=True, is_occluder=False
        )
        
        # Load table
        table_z_offset = -0.3
        table_position = [0.5, 0.0, table_z_offset]
        table_id = p.loadURDF("table/table.urdf", table_position, [0, 0, 0, 1], useFixedBase=True)
        self.table_surface_height = 0.625 + table_z_offset

        # Table XY bounds — derived from table position [0.5, 0] and size [1.0, 1.0].
        # Defined once here; passed to ShadowCalculator, FreeSpaceGenerator, and
        # any external code that needs to check table boundaries.
        self.table_x_range = (0.0, 1.0)
        self.table_y_range = (-0.5, 0.5)
        
        self.objects["table"] = ObjectInfo(
            object_id=table_id, name="table",
            position=np.array(table_position), orientation=np.array([0, 0, 0, 1]),
            size=np.array([1.0, 1.0, 0.8]), is_visible=True, is_occluder=False
        )
        
        # Load robot
        robot_pos = [-0.4, 0.0, 0.0]
        robot_id = p.loadURDF("franka_panda/panda.urdf", robot_pos, [0, 0, 0, 1], useFixedBase=True)
        self.objects["robot"] = ObjectInfo(
            object_id=robot_id, name="robot",
            position=np.array(robot_pos), orientation=np.array([0, 0, 0, 1]),
            size=np.array([0.5, 0.5, 0.8]), is_visible=True, is_occluder=False
        )
        
        # Create occluders
        self._create_occluders()
        
        # Create targets
        self._create_targets()
        
        # Let objects settle under gravity after placement.
        # 10 steps at 240 Hz ≈ 0.04 s — sufficient for cubes on a flat
        # table to reach static equilibrium.
        for _ in range(10):
            p.stepSimulation()
        
        self.update_object_positions()
    
    def _create_occluders(self):
        """Create occluder cubes on the table."""
        # 0.15 m (15 cm) cubes — large enough to fully occlude a target
        # (0.08 m) from the overhead camera, small enough for the Panda
        # gripper to grasp (max opening 0.08 m allows top-down pinch).
        occluder_size = [0.15, 0.15, 0.15]
        occluder_z = self.table_surface_height + occluder_size[2] / 2
        
        positions = [
            [0.5, 0.2, occluder_z],
            [0.6, -0.1, occluder_z],
            [0.4, -0.2, occluder_z],
        ]
        
        for i, pos in enumerate(positions):
            visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[s/2 for s in occluder_size],
                                            rgbaColor=[0.8, 0.3, 0.3, 1.0])
            collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s/2 for s in occluder_size])
            body_id = p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=collision_id,
                                        baseVisualShapeIndex=visual_id, basePosition=pos)
            
            name = f"occluder_{i+1}"
            self.objects[name] = ObjectInfo(
                object_id=body_id, name=name,
                position=np.array(pos), orientation=np.array([0, 0, 0, 1]),
                size=np.array(occluder_size), is_visible=True, is_occluder=True
            )
    
    def _create_targets(self):
        """Create target cubes on the table."""
        # 0.08 m (8 cm) cubes — small enough to be fully hidden behind
        # occluders (0.15 m), graspable by the Panda (0.08 m max opening).
        target_size = [0.08, 0.08, 0.08]
        target_z = self.table_surface_height + target_size[2] / 2
        
        positions = [
            [0.5, 0.4, target_z],
            [0.6, 0.1, target_z],
            [0.4, -0.1, target_z],
            [0.7, -0.2, target_z],
        ]
        
        for i, pos in enumerate(positions):
            visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[s/2 for s in target_size],
                                            rgbaColor=[0.3, 0.3, 0.8, 1.0])
            collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s/2 for s in target_size])
            body_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=collision_id,
                                        baseVisualShapeIndex=visual_id, basePosition=pos)
            
            name = f"target_{i+1}"
            self.objects[name] = ObjectInfo(
                object_id=body_id, name=name,
                position=np.array(pos), orientation=np.array([0, 0, 0, 1]),
                size=np.array(target_size), is_visible=False, is_occluder=False
            )
    
    def generate_boxels(self, visible_objects: List[str]) -> List[Boxel]:
        """
        Generate Semantic Boxels for visible objects and their shadows.
        
        Args:
            visible_objects: List of visible object names
            
        Returns:
            List of Boxel objects (objects + shadows)
        """
        boxels = []
        solid_boxels = []
        
        # Generate object boxels (initially not marked as occluders)
        for obj_name in visible_objects:
            if obj_name not in self.objects:
                continue
                
            obj_info = self.objects[obj_name]
            aabb_min, aabb_max = p.getAABB(obj_info.object_id)
            
            center = (np.array(aabb_min) + np.array(aabb_max)) / 2.0
            extent = (np.array(aabb_max) - np.array(aabb_min)) / 2.0
            
            obj_boxel = Boxel(center=center, extent=extent, object_name=obj_name,
                             is_occluded=False, is_shadow=False, is_occluder=False)
            solid_boxels.append(obj_boxel)

        # Generate shadow boxels and mark objects as occluders if they cast shadows
        for obj_boxel in solid_boxels:
            obstacles = [b for b in solid_boxels if b.object_name != obj_boxel.object_name]
            shadow_parts = self.shadow_calculator.calculate_shadow_boxel(obj_boxel, obstacles)
            
            # If this object casts any shadows, mark it as an occluder
            if shadow_parts:
                obj_boxel.is_occluder = True
            
            boxels.extend(shadow_parts)
        
        # Add all object boxels (now with correct is_occluder status)
        boxels.extend(solid_boxels)
            
        return boxels
    
    def generate_free_space(self, known_boxels: List[Boxel], visualize: bool = False) -> List[Boxel]:
        """
        Discretize the free space using octree subdivision.
        
        Args:
            known_boxels: List of known boxels (objects + shadows)
            visualize: If True, animates the generation process
            
        Returns:
            List of free space boxels
        """
        return self.free_space_generator.generate(known_boxels, visualize)
    
    def get_camera_observation(self) -> CameraObservation:
        """
        Capture an observation from the camera.

        Only computes what downstream code actually consumes: visible objects,
        object poses, and semantic boxels.  RGB, depth, and point-cloud fields
        default to None (no consumer exists; see audit #56).
        """
        visible_objects, object_poses = self.oracle_detect_objects()
        boxels = self.generate_boxels(visible_objects)

        return CameraObservation(
            visible_objects=visible_objects, object_poses=object_poses, boxels=boxels
        )
    
    def _depth_buffer_to_meters(self, depth_buffer: np.ndarray) -> np.ndarray:
        """Convert depth buffer values to meters."""
        return self.far_plane * self.near_plane / (
            self.far_plane - (self.far_plane - self.near_plane) * depth_buffer
        )
    
    def _depth_to_point_cloud(self, depth_image: np.ndarray, view_matrix, projection_matrix) -> np.ndarray:
        """Convert depth image to 3D point cloud."""
        height, width = depth_image.shape
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        z_ndc = depth_image.flatten()
        
        fx = fy = (width / 2.0) / np.tan(np.radians(self.fov / 2.0))
        cx, cy = width / 2.0, height / 2.0
        
        x_cam = (u.flatten() - cx) * z_ndc / fx
        y_cam = (v.flatten() - cy) * z_ndc / fy
        
        points_cam = np.stack([x_cam, y_cam, z_ndc], axis=1)
        
        view_matrix_4x4 = np.array(view_matrix).reshape(4, 4)
        camera_to_world = np.linalg.inv(view_matrix_4x4)
        
        points_cam_homogeneous = np.hstack([points_cam, np.ones((points_cam.shape[0], 1))])
        points_world = (camera_to_world @ points_cam_homogeneous.T).T[:, :3]
        
        valid_mask = (points_world[:, 2] > -1.0) & (points_world[:, 2] < 2.0)
        return points_world[valid_mask]
    
    def oracle_detect_objects(self, check_occlusion: bool = True) -> Tuple[List[str], Dict[str, Tuple[np.ndarray, np.ndarray]]]:
        """
        Oracle function to detect visible objects and their poses.

        Visibility is determined by casting rays from the camera to the
        8 corners of each object's AABB.  An object is visible if ANY ray
        reaches it (hit body == object body).  This catches partial
        visibility where an object edge sticks out from behind an occluder.

        Only used for initial scene observation — the sensing action uses
        ``sense_shadow_raycasting()`` with its own ray grid.
        
        Returns:
            Tuple of (visible object names, dict of all object poses)
        """
        visible_objects = []
        object_poses = {}
        
        for name, obj_info in self.objects.items():
            if name in ["plane", "table", "robot"]:
                continue
            
            pos, orn = p.getBasePositionAndOrientation(obj_info.object_id)
            position = np.array(pos)
            orientation = np.array(orn)
            
            obj_info.position = position
            obj_info.orientation = orientation
            
            is_visible = True
            if check_occlusion:
                aabb_min, aabb_max = p.getAABB(obj_info.object_id)
                ray_targets = []
                for x in (aabb_min[0], aabb_max[0]):
                    for y in (aabb_min[1], aabb_max[1]):
                        for z in (aabb_min[2], aabb_max[2]):
                            ray_targets.append([x, y, z])

                cam = self.camera_position.tolist()
                results = p.rayTestBatch(
                    [cam] * len(ray_targets), ray_targets
                )
                is_visible = any(
                    r[0] == obj_info.object_id for r in results
                )
            
            obj_info.is_visible = is_visible
            if is_visible:
                visible_objects.append(name)
            object_poses[name] = (position.copy(), orientation.copy())
        
        return visible_objects, object_poses

    def update_object_positions(self):
        """
        Synchronise every dynamic object's ObjectInfo with its current PyBullet
        pose.  Call this after any batch of physics steps (settling, pushing,
        etc.) so that downstream code never uses stale spawn-time positions.

        Static bodies (plane, table, robot) are skipped — they cannot move.
        """
        for name, obj_info in self.objects.items():
            if name in ("plane", "table", "robot"):
                continue
            pos, orn = p.getBasePositionAndOrientation(obj_info.object_id)
            obj_info.position = np.array(pos)
            obj_info.orientation = np.array(orn)

    def step_simulation(self, num_steps: int = 1):
        """Step the simulation forward."""
        for _ in range(num_steps):
            p.stepSimulation()
    
    def reset(self):
        """Reset the environment to initial state."""
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)  # standard gravitational acceleration
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.objects.clear()
        self._setup_scene()
    
    def close(self):
        """Close the PyBullet connection."""
        p.disconnect(self.client_id)
