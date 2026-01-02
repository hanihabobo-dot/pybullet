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
class CameraObservation:
    """Data structure for camera observations."""
    rgb_image: np.ndarray  # RGB image (H, W, 3)
    depth_image: np.ndarray  # Depth image (H, W) in meters
    point_cloud: np.ndarray  # Point cloud (N, 3) in world coordinates
    visible_objects: List[str]  # List of object names that are visible
    object_poses: Dict[str, Tuple[np.ndarray, np.ndarray]]  # Dict mapping object names to (position, orientation)


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
        # PyBullet table URDF has surface at approximately 0.8m height from its base
        # Since we lowered the table by table_z_offset, adjust surface height accordingly
        table_surface_height = 0.8 + table_z_offset  # Height of table surface from ground (0.5m)
        
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
    
    def get_camera_observation(self) -> CameraObservation:
        """
        Capture an observation from the fixed camera.
        
        This function:
        1. Renders RGB and depth images from the camera
        2. Generates a point cloud from the depth image
        3. Uses the oracle to detect visible objects
        4. Returns all observation data
        
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
        
        return CameraObservation(
            rgb_image=rgb_image,
            depth_image=depth_image,
            point_cloud=point_cloud,
            visible_objects=visible_objects,
            object_poses=object_poses
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
    print("\nRunning simulation for 5 seconds...")
    print("(Close the PyBullet window or press Ctrl+C to exit)")
    
    try:
        for step in range(240 * 100):  # 5 seconds at 240Hz
            env.step_simulation()
            
            # Update camera view every 10 steps
            # Focus on table center (where objects are) rather than robot
            if step % 10 == 0:
                # Focus on table center at table surface height
                table_center = np.array([0.5, 0.0, 0.5])  # Lowered table surface height
                p.resetDebugVisualizerCamera(
                    cameraDistance=1.5,
                    cameraYaw=45,
                    cameraPitch=-30,
                    cameraTargetPosition=table_center
                )
            
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    # Close environment
    env.close()
    print("\nEnvironment closed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()
