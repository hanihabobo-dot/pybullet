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
                 camera_position: np.ndarray = None):
        """
        Initialize streams with environment context.
        
        Args:
            registry: BoxelRegistry containing all boxels
            robot_id: PyBullet body ID of the robot
            camera_position: Default camera position for sensing
        """
        self.registry = registry
        self.robot_id = robot_id
        self.camera_position = camera_position or np.array([0.5, -0.8, 0.7])
        
        # Franka Panda joint limits
        self.joint_limits_low = np.array([-2.8973, -1.7628, -2.8973, -3.0718, 
                                          -2.8973, -0.0175, -2.8973])
        self.joint_limits_high = np.array([2.8973, 1.7628, 2.8973, -0.0698,
                                           2.8973, 3.7525, 2.8973])
        
        # Home configuration
        self.home_config = RobotConfig(
            joint_positions=np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
            name="q_home"
        )
        
        # Config counter for naming
        self._config_counter = 0
        self._traj_counter = 0
        self._grasp_counter = 0
    
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
        
        For now, returns a mock configuration. In full implementation,
        would use PyBullet's calculateInverseKinematics.
        """
        # TODO: Implement actual IK with PyBullet
        # For now, return mock configs for testing
        
        # Simple heuristic: perturb home config based on target
        offset = (ee_pos - np.array([0.5, 0, 0.5])) * 0.5
        joint_offsets = np.array([offset[1], offset[2], offset[0], 
                                  0, offset[1], offset[2], 0])
        
        new_joints = self.home_config.joint_positions + joint_offsets * 0.3
        
        # Clamp to limits
        new_joints = np.clip(new_joints, self.joint_limits_low, self.joint_limits_high)
        
        return RobotConfig(joint_positions=new_joints)
    
    # =========================================================================
    # STREAM: Sample Grasp
    # =========================================================================
    def sample_grasp(self, obj_id: str) -> Iterator[Tuple[Grasp]]:
        """
        Generate valid grasp poses for an object.
        
        PDDLStream declaration:
            (:stream sample-grasp
              :inputs (?o - obj)
              :domain (obj_graspable ?o)
              :outputs (?g - grasp)
              :certified (valid_grasp ?o ?g))
        
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


# =============================================================================
# PDDLStream Stream Definitions (for integration)
# =============================================================================

def get_stream_definitions():
    """
    Return PDDLStream stream definitions as a string.
    
    These are used by PDDLStream to know what streams are available
    and how they relate inputs to outputs.
    """
    return """
    (:stream sample-sensing-config
      :inputs (?b - boxel)
      :domain (is_shadow ?b)
      :outputs (?q - config)
      :certified (sensing_config ?b ?q)
    )
    
    (:stream sample-grasp
      :inputs (?o - obj)
      :domain (obj_graspable ?o)
      :outputs (?g - grasp)
      :certified (valid_grasp ?o ?g)
    )
    
    (:stream plan-motion
      :inputs (?q1 ?q2 - config)
      :domain (at_config ?q1)
      :outputs (?t - trajectory)
      :certified (motion_plan ?q1 ?q2 ?t)
    )
    
    (:stream compute-kin
      :inputs (?o - obj ?b - boxel ?g - grasp)
      :domain (and (obj_at_boxel ?o ?b) (valid_grasp ?o ?g))
      :outputs (?q - config)
      :certified (kin_solution ?o ?b ?g ?q)
    )
    """
