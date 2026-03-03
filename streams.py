"""
PDDLStream Streams for Semantic Boxel TAMP.

This module implements the geometric reasoning streams that generate and test
continuous parameters for the symbolic planner. Streams interface with PyBullet
for inverse kinematics, motion planning, and visibility checking.

Streams:
    - sample_sensing_config: Generate robot configs to observe a boxel
    - sample_grasp: Generate grasp poses for an object
    - plan_motion: Plan collision-free trajectory between configs
    - test_visibility: Check if boxel is observable from config
"""

import numpy as np
import pybullet as p
from typing import List, Tuple, Optional, Generator, Iterator
from dataclasses import dataclass

from boxel_data import BoxelRegistry, BoxelData, BoxelType
from robot_utils import (ARM_JOINT_INDICES, END_EFFECTOR_LINK, FINGER_JOINTS,
                         JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH, REST_POSES)


@dataclass
class RobotConfig:
    """Robot configuration (joint angles for Franka Panda)."""
    joint_positions: np.ndarray  # 7 DOF
    name: str = ""
    
    def __hash__(self):
        return hash(self.name)
    
    def __eq__(self, other):
        return self.name == other.name


@dataclass 
class Trajectory:
    """Motion trajectory as sequence of configurations."""
    waypoints: List[RobotConfig]
    name: str = ""


@dataclass
class Grasp:
    """Grasp transformation relative to object frame."""
    position: np.ndarray   # [x, y, z] offset
    orientation: np.ndarray  # [x, y, z, w] quaternion
    name: str = ""


class BoxelStreams:
    """
    PDDLStream-compatible streams for Semantic Boxel TAMP.
    
    These are generator functions that yield tuples of output values.
    PDDLStream calls them lazily during planning.
    """
    
    def __init__(self, registry: BoxelRegistry, robot_id: int = None,
                 physics_client: int = None):
        """
        Initialize streams with environment context.
        
        Args:
            registry: BoxelRegistry containing all boxels
            robot_id: PyBullet body ID of the robot (required for real IK)
            physics_client: PyBullet physics client ID (0 if using default)
        """
        self.registry = registry
        self.robot_id = robot_id
        self.physics_client = physics_client if physics_client is not None else 0
        
        # Home configuration
        self.home_config = RobotConfig(
            joint_positions=np.array(REST_POSES),
            name="q_home"
        )
        
        # Config counter for naming
        self._config_counter = 0
        self._traj_counter = 0
        self._grasp_counter = 0
        
        # IK parameters
        self.ik_max_iterations = 100
        self.ik_residual_threshold = 1e-4
    
    # =========================================================================
    # STREAM: Sample Push Configuration (for moving occluders aside)
    # =========================================================================
    def sample_push_config(self, occluder_id: str) -> Iterator[Tuple[RobotConfig]]:
        """
        Generate robot configurations for pushing an occluder aside.
        
        PDDLStream declaration:
            (:stream sample-push-config
              :inputs (?occ)
              :domain (is_object ?occ)
              :outputs (?q)
              :certified (and (Config ?q) (push_config ?occ ?q)))
        
        Args:
            occluder_id: ID of the occluder boxel to push
            
        Yields:
            Tuples of (config,) that can push the occluder
        """
        boxel = self.registry.get_boxel(occluder_id)
        if boxel is None:
            return
        
        # Generate push positions (approach from the side)
        occluder_center = boxel.center
        
        for angle in np.linspace(0, 2*np.pi, 4, endpoint=False):
            # Position to the side of the occluder
            push_pos = occluder_center + np.array([
                0.15 * np.cos(angle),
                0.15 * np.sin(angle),
                0.1  # Slightly above
            ])
            
            # Compute IK
            config = self._compute_sensing_ik(push_pos, occluder_center)
            
            if config is not None:
                self._config_counter += 1
                config.name = f"q_push_{occluder_id}_{self._config_counter}"
                yield (config,)
    
    # =========================================================================
    # STREAM: Sample Sensing Configuration
    # =========================================================================
    def sample_sensing_config(self, boxel_id: str) -> Iterator[Tuple[RobotConfig]]:
        """
        Generate robot configurations that can observe a given boxel.
        
        PDDLStream declaration:
            (:stream sample-sensing-config
              :inputs (?b - boxel)
              :domain (is_shadow ?b)
              :outputs (?q - config)
              :certified (sensing_config ?b ?q))
        
        Args:
            boxel_id: ID of the boxel to observe
            
        Yields:
            Tuples of (config,) that can observe the boxel
        """
        boxel = self.registry.get_boxel(boxel_id)
        if boxel is None:
            return
        
        # Generate viewpoints around the boxel
        boxel_center = boxel.center
        
        # Sample viewing angles
        for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
            for height_offset in [0.0, 0.15, 0.3]:
                # Compute camera position for this view
                distance = 0.4  # Distance from boxel center
                view_pos = boxel_center + np.array([
                    distance * np.cos(angle),
                    distance * np.sin(angle),
                    height_offset + 0.2
                ])
                
                # Compute IK for this end-effector position looking at boxel
                config = self._compute_sensing_ik(view_pos, boxel_center)
                
                if config is not None:
                    self._config_counter += 1
                    config.name = f"q_sense_{boxel_id}_{self._config_counter}"
                    yield (config,)
    
    def _compute_sensing_ik(self, ee_pos: np.ndarray, 
                            look_at: np.ndarray) -> Optional[RobotConfig]:
        """
        Compute IK for end-effector position looking at target.
        
        Uses PyBullet's calculateInverseKinematics if robot_id is set,
        otherwise falls back to heuristic for testing.
        
        Args:
            ee_pos: Desired end-effector position [x, y, z]
            look_at: Point the end-effector should look at
            
        Returns:
            RobotConfig if IK solution found, None otherwise
        """
        # Compute orientation: end-effector pointing toward look_at
        direction = look_at - ee_pos
        direction = direction / (np.linalg.norm(direction) + 1e-8)
        
        # End-effector z-axis points down, so we want -direction
        # Use rotation to align with downward-looking pose
        ee_orn = self._direction_to_quat(-direction)
        
        # Try real IK if robot is available
        if self.robot_id is not None:
            return self._pybullet_ik(ee_pos, ee_orn)
        
        # Fallback: heuristic for testing without robot
        return self._heuristic_ik(ee_pos, look_at)
    
    def _pybullet_ik(self, ee_pos: np.ndarray, 
                     ee_orn: np.ndarray) -> Optional[RobotConfig]:
        """
        Compute IK using PyBullet's calculateInverseKinematics.
        
        Args:
            ee_pos: Desired end-effector position
            ee_orn: Desired end-effector orientation (quaternion)
            
        Returns:
            RobotConfig if valid solution found, None otherwise
        """
        try:
            # Simple IK call - more robust than null-space version
            joint_positions = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=END_EFFECTOR_LINK,
                targetPosition=ee_pos.tolist(),
                targetOrientation=ee_orn.tolist(),
                maxNumIterations=self.ik_max_iterations,
                residualThreshold=self.ik_residual_threshold,
                physicsClientId=self.physics_client
            )
            
            if joint_positions is None or len(joint_positions) < 7:
                return None
            
            # Extract only the arm joints (first 7)
            arm_joints = np.array(joint_positions[:7])
            
            # Validate joint limits (with small tolerance)
            if np.any(arm_joints < JOINT_LIMITS_LOW - 0.1) or \
               np.any(arm_joints > JOINT_LIMITS_HIGH + 0.1):
                return None
            
            # Clamp to limits
            arm_joints = np.clip(arm_joints, JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)
            
            # Verify solution by checking forward kinematics error
            # (skip for now to avoid state changes)
            
            return RobotConfig(joint_positions=arm_joints)
            
        except Exception as e:
            # IK failed
            return None
    
    def _heuristic_ik(self, ee_pos: np.ndarray, 
                      look_at: np.ndarray) -> Optional[RobotConfig]:
        """
        Heuristic IK for testing without robot.
        
        Generates plausible joint configurations based on target position.
        """
        # Simple heuristic: perturb home config based on target
        offset = (ee_pos - np.array([0.5, 0, 0.5])) * 0.5
        joint_offsets = np.array([offset[1], offset[2], offset[0], 
                                  0, offset[1], offset[2], 0])
        
        new_joints = self.home_config.joint_positions + joint_offsets * 0.3
        
        # Clamp to limits
        new_joints = np.clip(new_joints, JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)
        
        return RobotConfig(joint_positions=new_joints)
    
    def _direction_to_quat(self, direction: np.ndarray) -> np.ndarray:
        """
        Convert a direction vector to a quaternion.
        
        The resulting orientation has z-axis aligned with direction.
        
        Args:
            direction: Normalized direction vector
            
        Returns:
            Quaternion [x, y, z, w]
        """
        # Default: pointing down (-z)
        z_axis = direction / (np.linalg.norm(direction) + 1e-8)
        
        # Choose x-axis perpendicular to z
        if abs(z_axis[2]) < 0.9:
            x_axis = np.cross([0, 0, 1], z_axis)
        else:
            x_axis = np.cross([0, 1, 0], z_axis)
        x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-8)
        
        # y-axis completes the frame
        y_axis = np.cross(z_axis, x_axis)
        
        # Build rotation matrix
        R = np.array([x_axis, y_axis, z_axis]).T
        
        # Convert to quaternion
        return self._rotation_matrix_to_quat(R)
    
    def _rotation_matrix_to_quat(self, R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([x, y, z, w])
    
    # =========================================================================
    # STREAM: Sample Grasp
    # =========================================================================
    def sample_grasp(self, obj_id: str) -> Iterator[Tuple[Grasp]]:
        """
        Generate valid grasp poses for an object.
        
        PDDLStream declaration (see pddl/stream.pddl):
            (:stream sample-grasp
              :inputs (?o)
              :domain (Obj ?o)
              :outputs (?g)
              :certified (and (Grasp ?g) (valid_grasp ?o ?g)))
        
        Args:
            obj_id: ID of the object to grasp
            
        Yields:
            Tuples of (grasp,) for the object
        """
        # Generate top-down grasps at different orientations
        for yaw in np.linspace(0, np.pi, 4, endpoint=False):
            self._grasp_counter += 1
            grasp = Grasp(
                position=np.array([0, 0, 0.05]),  # Approach from above
                orientation=self._euler_to_quat(0, np.pi, yaw),
                name=f"grasp_{obj_id}_{self._grasp_counter}"
            )
            yield (grasp,)
    
    def _euler_to_quat(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert Euler angles to quaternion [x, y, z, w]."""
        cy, sy = np.cos(yaw/2), np.sin(yaw/2)
        cp, sp = np.cos(pitch/2), np.sin(pitch/2)
        cr, sr = np.cos(roll/2), np.sin(roll/2)
        
        return np.array([
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
            cr*cp*cy + sr*sp*sy
        ])
    
    # =========================================================================
    # STREAM: Plan Motion
    # =========================================================================
    def plan_motion(self, q1: RobotConfig, q2: RobotConfig) -> Iterator[Tuple[Trajectory]]:
        """
        Plan collision-free motion between configurations.
        
        PDDLStream declaration:
            (:stream plan-motion
              :inputs (?q1 ?q2 - config)
              :domain (and (at_config ?q1))
              :outputs (?t - trajectory)
              :certified (motion_plan ?q1 ?q2 ?t))
        
        Args:
            q1: Start configuration
            q2: Goal configuration
            
        Yields:
            Tuples of (trajectory,) connecting q1 to q2
        """
        # TODO: Implement actual motion planning (RRT, PRM, etc.)
        # For now, simple linear interpolation (no collision checking)
        
        n_waypoints = 10
        waypoints = []
        
        for t in np.linspace(0, 1, n_waypoints):
            interp_joints = (1-t) * q1.joint_positions + t * q2.joint_positions
            waypoints.append(RobotConfig(
                joint_positions=interp_joints,
                name=f"{q1.name}_to_{q2.name}_wp{int(t*10)}"
            ))
        
        self._traj_counter += 1
        traj = Trajectory(
            waypoints=waypoints,
            name=f"traj_{self._traj_counter}"
        )
        
        yield (traj,)
    
    # =========================================================================
    # STREAM: Compute IK for Pick/Place
    # =========================================================================
    def compute_kin_solution(self, obj_id: str, boxel_id: str, 
                             grasp: Grasp) -> Iterator[Tuple[RobotConfig]]:
        """
        Compute IK solution for picking object from boxel with grasp.
        
        PDDLStream declaration:
            (:stream compute-kin
              :inputs (?o - obj ?b - boxel ?g - grasp)
              :domain (and (obj_at_boxel ?o ?b) (valid_grasp ?o ?g))
              :outputs (?q - config)
              :certified (kin_solution ?o ?b ?g ?q))
        
        Args:
            obj_id: Object to grasp
            boxel_id: Boxel containing object
            grasp: Grasp to use
            
        Yields:
            Tuples of (config,) for grasping
        """
        boxel = self.registry.get_boxel(boxel_id)
        if boxel is None:
            return
        
        # Target end-effector position: boxel center + grasp offset
        target_pos = boxel.center + grasp.position
        
        # Compute IK
        config = self._compute_sensing_ik(target_pos, boxel.center)
        
        if config is not None:
            self._config_counter += 1
            config.name = f"q_pick_{obj_id}_{self._config_counter}"
            yield (config,)
    
    # =========================================================================
    # TEST: Check Visibility
    # =========================================================================
    def test_visibility(self, boxel_id: str, config: RobotConfig) -> bool:
        """
        Test if boxel is visible from configuration (for stream certification).
        
        Args:
            boxel_id: Boxel to check
            config: Robot configuration
            
        Returns:
            True if boxel is observable from config
        """
        # TODO: Implement actual visibility check with ray casting
        # For now, always return True for mock testing
        return True
