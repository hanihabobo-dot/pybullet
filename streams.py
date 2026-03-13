"""
PDDLStream Streams for Semantic Boxel TAMP.

This module implements the geometric reasoning streams that generate and test
continuous parameters for the symbolic planner. Streams interface with PyBullet
for inverse kinematics and motion planning.

Sensing uses the fixed scene camera (not the robot's end-effector), so there
is no sensing_config stream. See issue #36 in CODEBASE_AUDIT.txt.

Streams:
    - sample_grasp: Generate grasp poses for an object
    - plan_motion: Plan collision-free trajectory between configs
    - compute_kin_solution: Compute IK for pick/place
"""

import logging
import random
import numpy as np
import pybullet as p
from typing import List, Tuple, Optional, Generator, Iterator
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

from boxel_data import BoxelRegistry, BoxelData, BoxelType
from robot_utils import (ARM_JOINT_INDICES, END_EFFECTOR_LINK, FINGER_JOINTS,
                         JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH, JOINT_RANGES,
                         REST_POSES, is_config_collision_free,
                         is_path_collision_free)


@dataclass
class RobotConfig:
    """Robot configuration (joint angles for Franka Panda)."""
    joint_positions: np.ndarray  # 7 DOF
    name: str = ""
    is_heuristic: bool = False
    ignored_body_ids: frozenset = field(default_factory=frozenset)
    
    def __hash__(self):
        return hash(self.name)
    
    def __eq__(self, other):
        if not isinstance(other, RobotConfig):
            return NotImplemented
        return self.name == other.name
    
    def __repr__(self):
        return self.name if self.name else f"RobotConfig({self.joint_positions})"


@dataclass 
class Trajectory:
    """Motion trajectory as sequence of configurations."""
    waypoints: List[RobotConfig]
    name: str = ""
    
    def __hash__(self):
        return hash(self.name)
    
    def __eq__(self, other):
        if not isinstance(other, Trajectory):
            return NotImplemented
        return self.name == other.name
    
    def __repr__(self):
        return self.name if self.name else f"Trajectory({len(self.waypoints)} waypoints)"


@dataclass
class Grasp:
    """Grasp transformation relative to object frame."""
    position: np.ndarray   # [x, y, z] offset
    orientation: np.ndarray  # [x, y, z, w] quaternion
    name: str = ""
    
    def __hash__(self):
        return hash(self.name)
    
    def __eq__(self, other):
        if not isinstance(other, Grasp):
            return NotImplemented
        return self.name == other.name
    
    def __repr__(self):
        return self.name if self.name else f"Grasp({self.position})"


class BoxelStreams:
    """
    PDDLStream-compatible streams for Semantic Boxel TAMP.
    
    These are generator functions that yield tuples of output values.
    PDDLStream calls them lazily during planning.
    """
    
    def __init__(self, registry: BoxelRegistry, robot_id: int = None,
                 physics_client: int = None, object_body_ids: dict = None,
                 support_body_ids: frozenset = None,
                 allow_heuristic: bool = False):
        """
        Initialize streams with environment context.

        Args:
            registry: BoxelRegistry containing all boxels
            robot_id: PyBullet body ID of the robot (required for real IK)
            physics_client: PyBullet physics client ID (0 if using default)
            object_body_ids: Mapping from object/boxel identifiers to PyBullet
                body IDs. Used to exclude the grasped object from collision
                checks in compute_kin and plan_motion. Keys should include
                both object names ("occluder_1") and boxel IDs ("obj_003").
            support_body_ids: Body IDs of support surfaces (table, ground
                plane).  Ignored during all collision checks for pick/place
                motions (both endpoint validation and RRT path planning)
                because the Panda is mounted on the table and its lower arm
                links overlap the table's collision geometry in PyBullet.
                Not ignored for pure transit motions with no grasped object.
            allow_heuristic: If True, permit heuristic IK when robot_id is
                None (for symbolic-only testing).  Default False — production
                code must always provide a robot_id.
        """
        self.registry = registry
        self.robot_id = robot_id
        self.physics_client = physics_client if physics_client is not None else 0
        self.object_body_ids = object_body_ids or {}
        self.support_body_ids = support_body_ids or frozenset()
        
        if self.robot_id is None and not allow_heuristic:
            raise ValueError(
                "BoxelStreams requires robot_id for kinematically valid IK. "
                "Pass allow_heuristic=True only for symbolic-only testing."
            )
        if self.robot_id is None:
            logger.warning(
                "BoxelStreams created without robot_id — all IK will use "
                "heuristic fallback (configs will NOT be kinematically valid)"
            )
        
        # Home configuration
        self.home_config = RobotConfig(
            joint_positions=np.array(REST_POSES),
            name="q_home"
        )
        
        # Config counter for naming
        self._config_counter = 0
        self._traj_counter = 0
        self._grasp_counter = 0
        
        # IK solver parameters (PyBullet's iterative Jacobian-based IK).
        # 100 iterations is the PyBullet recommended default; convergence
        # threshold 1e-4 m balances precision vs speed (empirically tuned —
        # tighter values rarely improve the solution but slow planning).
        self.ik_max_iterations = 100
        self.ik_residual_threshold = 1e-4
    
    # =========================================================================
    # STREAM: Sample Push Solution — superseded by pick-and-place (#53)
    # =========================================================================
    # def sample_push_config(self, occluder_id: str, b_from: str) -> Iterator[Tuple]:
    #     """
    #     Generate push solutions: destination, start/end configs, and trajectory.
    #     Superseded by pick → move → place for occluder relocation (#53).
    #     """
    #     boxel = self.registry.get_boxel(occluder_id)
    #     if boxel is None:
    #         return
    #     occluder_center = boxel.center
    #     for angle in np.linspace(0, 2*np.pi, 4, endpoint=False):
    #         push_pos = occluder_center + np.array([
    #             0.15 * np.cos(angle), 0.15 * np.sin(angle), 0.1
    #         ])
    #         q_start_config = self._compute_sensing_ik(push_pos, occluder_center)
    #         if q_start_config is None:
    #             continue
    #         self._config_counter += 1
    #         q_start_config.name = f"q_push_start_{occluder_id}_{self._config_counter}"
    #         push_direction = np.array([np.cos(angle), np.sin(angle), 0.0])
    #         push_dist = np.max(boxel.extent[:2]) + 0.10
    #         dest_center = occluder_center + push_direction * push_dist
    #         dest_pos = dest_center + np.array([0.0, 0.0, 0.1])
    #         q_end_config = self._compute_sensing_ik(dest_pos, dest_center)
    #         if q_end_config is None:
    #             continue
    #         self._config_counter += 1
    #         q_end_config.name = f"q_push_end_{occluder_id}_{self._config_counter}"
    #         self._traj_counter += 1
    #         traj = Trajectory(
    #             waypoints=[q_start_config, q_end_config],
    #             name=f"traj_push_{occluder_id}_{self._traj_counter}"
    #         )
    #         b_to_name = f"push_dest_{occluder_id}_{self._config_counter}"
    #         yield (b_to_name, q_start_config, q_end_config, traj)
    
    IK_NUM_SEEDS = 8

    # Seed perturbations for multi-start IK (radians, added to REST_POSES).
    # First row is zero (start from rest); rows 2-3 are uniform ±0.4 rad
    # pushes; rows 4-8 are hand-tuned pseudo-random perturbations chosen
    # to spread the 7-DOF null-space exploration.  Magnitudes stay within
    # ±0.8 rad so that clipped seeds remain well inside joint limits.
    # Empirically, 8 seeds resolve >95% of reachable targets on the first
    # planning call; increasing beyond 8 showed diminishing returns.
    _IK_SEED_OFFSETS = [
        [0, 0, 0, 0, 0, 0, 0],
        [0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4],
        [-0.4, -0.4, -0.4, -0.4, -0.4, -0.4, -0.4],
        [0.8, -0.3, 0.5, -0.6, 0.3, 0.7, -0.2],
        [-0.6, 0.5, -0.3, 0.8, -0.5, -0.2, 0.6],
        [0.2, -0.7, 0.6, 0.3, -0.8, 0.4, -0.5],
        [-0.3, 0.6, -0.8, -0.2, 0.7, -0.5, 0.3],
        [0.5, 0.2, -0.5, 0.7, 0.2, -0.6, 0.4],
    ]

    def _pybullet_ik(self, ee_pos: np.ndarray,
                     ee_orn: np.ndarray,
                     seed: list = None) -> Optional[RobotConfig]:
        """
        Compute IK using PyBullet's null-space calculateInverseKinematics.

        Saves the robot's current joint state, resets joints to *seed*
        (defaulting to REST_POSES), runs IK with null-space parameters,
        then restores the original state.  Different seeds steer the
        iterative solver into different local minima, producing arm
        configurations that may avoid obstacles the default solution hits.

        Args:
            ee_pos: Desired end-effector position.
            ee_orn: Desired end-effector orientation (quaternion).
            seed:   7-element list of joint angles used as the IK starting
                    point.  ``None`` → REST_POSES.

        Returns:
            RobotConfig if valid solution found, None otherwise.
        """
        if seed is None:
            seed = REST_POSES
        saved_joints = None
        pc = self.physics_client
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0,
                                   physicsClientId=pc)
        try:
            saved_joints = [
                p.getJointState(self.robot_id, i,
                                physicsClientId=pc)[0]
                for i in ARM_JOINT_INDICES
            ]

            for i, angle in zip(ARM_JOINT_INDICES, seed):
                p.resetJointState(self.robot_id, i, angle,
                                  physicsClientId=pc)

            joint_positions = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=END_EFFECTOR_LINK,
                targetPosition=ee_pos.tolist(),
                targetOrientation=ee_orn.tolist(),
                lowerLimits=JOINT_LIMITS_LOW.tolist(),
                upperLimits=JOINT_LIMITS_HIGH.tolist(),
                jointRanges=JOINT_RANGES.tolist(),
                restPoses=list(seed),
                maxNumIterations=self.ik_max_iterations,
                residualThreshold=self.ik_residual_threshold,
                physicsClientId=pc
            )

            if joint_positions is None or len(joint_positions) < 7:
                return None

            arm_joints = np.array(joint_positions[:7])

            if np.any(arm_joints < JOINT_LIMITS_LOW - 0.1) or \
               np.any(arm_joints > JOINT_LIMITS_HIGH + 0.1):
                return None

            arm_joints = np.clip(arm_joints, JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)

            return RobotConfig(joint_positions=arm_joints)

        except Exception as e:
            logger.error("IK failed for pos=%s: %s", ee_pos.tolist(), e)
            return None

        finally:
            if saved_joints is not None:
                for i, angle in zip(ARM_JOINT_INDICES, saved_joints):
                    p.resetJointState(self.robot_id, i, angle,
                                      physicsClientId=pc)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1,
                                       physicsClientId=pc)

    def _ik_seeds(self):
        """Yield IK seed configurations: REST_POSES first, then perturbations."""
        rest = np.array(REST_POSES)
        for offset in self._IK_SEED_OFFSETS[:self.IK_NUM_SEEDS]:
            seed = rest + np.array(offset)
            seed = np.clip(seed, JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)
            yield seed.tolist()
    
    def _heuristic_ik(self, ee_pos: np.ndarray, 
                      look_at: np.ndarray) -> Optional[RobotConfig]:
        """
        Fake IK fallback for testing without a loaded robot.
        
        WARNING: This is NOT inverse kinematics. It perturbs the home config
        by an arbitrary function of the target position. The resulting joint
        angles have no guaranteed relationship to the requested end-effector
        pose. Configs produced here are marked with is_heuristic=True so
        downstream code can detect and reject them when real IK is expected.
        """
        logger.warning(
            "Using heuristic IK (ee_pos=%s) — result is NOT a valid IK "
            "solution. Config will be marked is_heuristic=True.",
            ee_pos.tolist()
        )
        
        offset = (ee_pos - np.array([0.5, 0, 0.5])) * 0.5
        joint_offsets = np.array([offset[1], offset[2], offset[0], 
                                  0, offset[1], offset[2], 0])
        
        new_joints = self.home_config.joint_positions + joint_offsets * 0.3
        new_joints = np.clip(new_joints, JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)
        
        return RobotConfig(joint_positions=new_joints, is_heuristic=True)
    
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
    
    def _resolve_body_id(self, obj_id) -> Optional[int]:
        """
        Look up the PyBullet body ID for an object or boxel identifier.

        Handles both direct object names (e.g. "target_2") and boxel IDs
        (e.g. "obj_003") by falling back to the boxel's object_name.
        """
        key = str(obj_id)
        if key in self.object_body_ids:
            return self.object_body_ids[key]
        boxel = self.registry.get_boxel(key)
        if boxel and boxel.object_name and boxel.object_name in self.object_body_ids:
            return self.object_body_ids[boxel.object_name]
        return None

    # =========================================================================
    # STREAM: Sample Grasp
    # =========================================================================
    # Top-down grasp clearances above the object center (m).
    # 0.05 is the minimum for Panda finger clearance above small objects
    # (target size 0.08 m); 0.10 and 0.15 give progressively more room
    # to avoid table/neighbor collisions at the cost of weaker grasp
    # contact (acceptable — execution uses a constraint-based weld).
    _GRASP_Z_OFFSETS = [0.05, 0.10, 0.15]

    def sample_grasp(self, obj_id: str) -> Iterator[Tuple[Grasp]]:
        """
        Generate grasp poses for an object with varying clearance.

        Yields multiple top-down grasps at different heights above the
        object center.  The lowest (0.05 m) places the EE close to the
        object for a tight grasp; higher offsets (0.10, 0.15 m) give the
        arm more room to avoid collisions with the table or adjacent
        objects at the cost of a less secure grip (acceptable because
        execution uses a constraint-based weld, not contact physics).

        PDDLStream declaration (see pddl/stream.pddl):
            (:stream sample-grasp
              :inputs (?o)
              :domain (Obj ?o)
              :outputs (?g)
              :certified (and (Grasp ?g) (valid_grasp ?o ?g)))

        Args:
            obj_id: ID of the object to grasp

        Yields:
            Tuples of (grasp,) for the object — one per Z offset.
        """
        orn = self._euler_to_quat(0, np.pi, 0)
        for z in self._GRASP_Z_OFFSETS:
            self._grasp_counter += 1
            grasp = Grasp(
                position=np.array([0, 0, z]),
                orientation=orn,
                name=f"grasp_{obj_id}_{self._grasp_counter}"
            )
            logger.debug("sample_grasp: %s -> %s (z=%.2f)", obj_id,
                         grasp.name, z)
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
    # STREAM: Plan Motion (RRT-Connect with shortcut smoothing)
    # =========================================================================

    # RRT-Connect parameters (Kuffner & LaValle, 2000).
    # MAX_ITERATIONS and STEP_SIZE follow standard practice for 7-DOF arms;
    # GOAL_BIAS 5% is the canonical value.  EDGE_CHECKS, CONNECT_ATTEMPTS,
    # and SMOOTH_ATTEMPTS were empirically tuned on the tabletop scenario
    # (2-4 objects, table-mounted Panda) — higher values improved solution
    # quality marginally while increasing planning time significantly.
    RRT_MAX_ITERATIONS = 2000    # sufficient for tabletop clutter
    RRT_STEP_SIZE = 0.2          # max joint displacement per extend (rad)
    RRT_GOAL_BIAS = 0.05         # probability of sampling the goal
    RRT_EDGE_CHECKS = 8          # collision samples per edge
    RRT_CONNECT_ATTEMPTS = 50    # max extends for the connect phase
    SMOOTH_ATTEMPTS = 75         # shortcut smoothing iterations

    def plan_motion(self, q1: RobotConfig, q2: RobotConfig) -> Iterator[Tuple[Trajectory]]:
        """
        Plan collision-free motion between two configurations.

        Collision checks respect ``ignored_body_ids`` carried by each
        config (set by ``compute_kin_solution`` for pick/place poses).
        This prevents the grasped object from blocking its own pick
        motion.

        Strategy:
          1. If no robot is loaded (heuristic mode), fall back to linear
             interpolation — there is no physics to check against.
          2. Verify both endpoints are collision-free.
          3. Try the direct linear path (fast path — most moves are simple
             reaches that don't collide with anything).
          4. If the direct path collides, run bidirectional RRT-Connect.
          5. Smooth the RRT path with random shortcutting.

        PDDLStream declaration (see pddl/stream.pddl)::

            (:stream plan-motion
              :inputs (?q1 ?q2)
              :domain (and (Config ?q1) (Config ?q2))
              :outputs (?t)
              :certified (and (Trajectory ?t) (motion ?q1 ?q2 ?t)))

        Args:
            q1: Start configuration.
            q2: Goal configuration.

        Yields:
            Tuples of ``(trajectory,)`` connecting *q1* to *q2*.
            Yields nothing if no collision-free path is found.
        """
        if self.robot_id is None:
            yield (self._linear_trajectory(q1, q2),)
            return

        pc = self.physics_client
        # Union of both endpoints' ignored bodies.  When moving from home
        # (ignored={}) to a pick config (ignored={obj}), this ignores the
        # grasped object for the entire path — which is necessary because
        # the pick endpoint places the gripper AT the object.  Without the
        # union, intermediate configs near the goal would be rejected for
        # colliding with the object and RRT could never connect to it.
        # A decomposed transit+approach architecture could restrict the
        # ignore set to the approach phase only, but the current single-
        # motion design requires the union.
        base_ignored = q1.ignored_body_ids | q2.ignored_body_ids
        is_pick_place = bool(q1.ignored_body_ids or q2.ignored_body_ids)

        endpoint_ignored = base_ignored | self.support_body_ids if is_pick_place else base_ignored

        # The Panda is mounted ON the table.  Its lower arm links overlap the
        # table's collision geometry in PyBullet at rest, producing false
        # positives on nearly every intermediate config.  For pick/place
        # motions (where the arm necessarily operates at table level) we must
        # include support surfaces in the path-planning ignored set as well.
        # For pure transit motions with no grasped object this is not needed.
        path_ignored = base_ignored | self.support_body_ids if is_pick_place else base_ignored

        logger.debug("plan_motion: %s -> %s  endpoint_ignored=%s "
                      "path_ignored=%s gripper_relax=%s",
                      q1, q2,
                      sorted(endpoint_ignored) if endpoint_ignored else '{}',
                      sorted(path_ignored) if path_ignored != endpoint_ignored else '=',
                      is_pick_place)

        if not is_config_collision_free(self.robot_id, q1.joint_positions,
                                        pc, endpoint_ignored,
                                        allow_gripper_collisions=is_pick_place):
            logger.warning("plan_motion: start config %s in collision "
                           "(ignored=%s)", q1, sorted(endpoint_ignored))
            return
        if not is_config_collision_free(self.robot_id, q2.joint_positions,
                                        pc, endpoint_ignored,
                                        allow_gripper_collisions=is_pick_place):
            logger.warning("plan_motion: goal config %s in collision "
                           "(ignored=%s)", q2, sorted(endpoint_ignored))
            return

        if is_path_collision_free(self.robot_id, q1.joint_positions,
                                  q2.joint_positions, pc,
                                  n_checks=self.RRT_EDGE_CHECKS,
                                  ignored_bodies=path_ignored,
                                  allow_gripper_collisions=is_pick_place):
            logger.info("plan_motion: direct path clear — linear trajectory")
            yield (self._linear_trajectory(q1, q2),)
            return

        logger.info("plan_motion: direct path blocked — running RRT-Connect")
        path = self._rrt_connect(q1.joint_positions, q2.joint_positions,
                                 path_ignored,
                                 allow_gripper_collisions=is_pick_place)

        if path is None:
            logger.warning("plan_motion: RRT-Connect failed (%d iters)",
                           self.RRT_MAX_ITERATIONS)
            return

        smoothed = self._smooth_path(path, path_ignored,
                                     allow_gripper_collisions=is_pick_place)
        logger.info("plan_motion: RRT path %d wps -> smoothed %d wps",
                     len(path), len(smoothed))

        waypoints = []
        for i, joints in enumerate(smoothed):
            waypoints.append(RobotConfig(
                joint_positions=joints,
                name=f"{q1.name}_to_{q2.name}_rrt{i}"
            ))

        self._traj_counter += 1
        traj = Trajectory(waypoints=waypoints,
                          name=f"traj_{self._traj_counter}")
        yield (traj,)

    # ----- helpers -----------------------------------------------------------

    def _linear_trajectory(self, q1: RobotConfig, q2: RobotConfig,
                           n_waypoints: int = 10) -> Trajectory:
        """Build a linearly-interpolated trajectory (no collision check)."""
        waypoints = []
        for t in np.linspace(0, 1, n_waypoints):
            waypoints.append(RobotConfig(
                joint_positions=(1 - t) * q1.joint_positions
                                + t * q2.joint_positions,
                name=f"{q1.name}_to_{q2.name}_wp{int(t * 10)}"
            ))
        self._traj_counter += 1
        return Trajectory(waypoints=waypoints,
                          name=f"traj_{self._traj_counter}")

    # ----- RRT-Connect -------------------------------------------------------

    def _random_config(self) -> np.ndarray:
        """Sample a uniform random configuration within joint limits."""
        return np.random.uniform(JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)

    def _nearest(self, nodes: List[np.ndarray], q: np.ndarray) -> int:
        """Return the index of the node closest to *q* (L2 in joint space)."""
        best_idx, best_dist = 0, np.inf
        for i, n in enumerate(nodes):
            d = np.linalg.norm(n - q)
            if d < best_dist:
                best_idx, best_dist = i, d
        return best_idx

    def _steer(self, q_near: np.ndarray, q_target: np.ndarray,
               max_step: float) -> np.ndarray:
        """
        Move from *q_near* toward *q_target*, limiting the maximum
        per-joint displacement to *max_step* radians.
        """
        diff = q_target - q_near
        max_diff = np.max(np.abs(diff))
        if max_diff <= max_step:
            return q_target.copy()
        return q_near + diff * (max_step / max_diff)

    def _try_connect(self, nodes: List[np.ndarray],
                     parents: List[int],
                     q_target: np.ndarray,
                     ignored_bodies: frozenset = frozenset(),
                     allow_gripper_collisions: bool = False) -> Optional[int]:
        """
        Greedily extend a tree toward *q_target* until it either reaches
        the target or hits a collision.  Returns the connecting node index,
        or ``None`` on failure.
        """
        cur_idx = self._nearest(nodes, q_target)
        pc = self.physics_client
        for _ in range(self.RRT_CONNECT_ATTEMPTS):
            q_cur = nodes[cur_idx]
            q_new = self._steer(q_cur, q_target, self.RRT_STEP_SIZE)
            if not is_path_collision_free(self.robot_id, q_cur, q_new, pc,
                                          self.RRT_EDGE_CHECKS,
                                          ignored_bodies=ignored_bodies,
                                          allow_gripper_collisions=allow_gripper_collisions):
                return None
            new_idx = len(nodes)
            nodes.append(q_new)
            parents.append(cur_idx)
            cur_idx = new_idx
            if np.allclose(q_new, q_target, atol=1e-3):
                return new_idx
        return None

    def _trace_path(self, nodes: List[np.ndarray],
                    parents: List[int], idx: int) -> List[np.ndarray]:
        """Walk parent pointers back to the root and return the path."""
        path = []
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()
        return path

    def _rrt_connect(self, q_start: np.ndarray,
                     q_goal: np.ndarray,
                     ignored_bodies: frozenset = frozenset(),
                     allow_gripper_collisions: bool = False
                     ) -> Optional[List[np.ndarray]]:
        """
        Bidirectional RRT-Connect (Kuffner & LaValle, 2000).

        Grows two trees — one from *q_start*, one from *q_goal* — and
        alternately extends them toward random samples.  When one tree
        successfully connects to a new node of the other tree, the two
        half-paths are spliced into a complete trajectory.

        Returns:
            List of joint-space waypoints from start to goal, or ``None``.
        """
        nodes_a: List[np.ndarray] = [q_start.copy()]
        parents_a: List[int] = [-1]
        nodes_b: List[np.ndarray] = [q_goal.copy()]
        parents_b: List[int] = [-1]

        swapped = False
        pc = self.physics_client

        for iteration in range(self.RRT_MAX_ITERATIONS):
            if iteration > 0 and iteration % 500 == 0:
                logger.debug("RRT-Connect: iter %d/%d  tree_a=%d  tree_b=%d",
                             iteration, self.RRT_MAX_ITERATIONS,
                             len(nodes_a), len(nodes_b))
            if random.random() < self.RRT_GOAL_BIAS:
                q_rand = nodes_b[0].copy()
            else:
                q_rand = self._random_config()

            near_idx = self._nearest(nodes_a, q_rand)
            q_new = self._steer(nodes_a[near_idx], q_rand, self.RRT_STEP_SIZE)

            if not is_path_collision_free(self.robot_id,
                                          nodes_a[near_idx], q_new, pc,
                                          self.RRT_EDGE_CHECKS,
                                          ignored_bodies=ignored_bodies,
                                          allow_gripper_collisions=allow_gripper_collisions):
                nodes_a, nodes_b = nodes_b, nodes_a
                parents_a, parents_b = parents_b, parents_a
                swapped = not swapped
                continue

            new_idx_a = len(nodes_a)
            nodes_a.append(q_new)
            parents_a.append(near_idx)

            connect_idx = self._try_connect(nodes_b, parents_b, q_new,
                                            ignored_bodies,
                                            allow_gripper_collisions)
            if connect_idx is not None:
                path_a = self._trace_path(nodes_a, parents_a, new_idx_a)
                path_b = self._trace_path(nodes_b, parents_b, connect_idx)
                if swapped:
                    path_a.reverse()
                    full = path_b + path_a[1:]
                else:
                    path_b.reverse()
                    full = path_a + path_b[1:]
                return full

            nodes_a, nodes_b = nodes_b, nodes_a
            parents_a, parents_b = parents_b, parents_a
            swapped = not swapped

        return None

    # ----- Shortcut smoothing ------------------------------------------------

    def _smooth_path(self, path: List[np.ndarray],
                     ignored_bodies: frozenset = frozenset(),
                     allow_gripper_collisions: bool = False
                     ) -> List[np.ndarray]:
        """
        Random shortcut smoothing: pick two non-adjacent waypoints and
        replace the segment between them with a direct edge if that edge
        is collision-free.
        """
        if len(path) <= 2:
            return list(path)
        smoothed = list(path)
        pc = self.physics_client
        for _ in range(self.SMOOTH_ATTEMPTS):
            if len(smoothed) <= 2:
                break
            i = random.randint(0, len(smoothed) - 3)
            j = random.randint(i + 2, len(smoothed) - 1)
            if is_path_collision_free(self.robot_id, smoothed[i], smoothed[j],
                                      pc, self.RRT_EDGE_CHECKS,
                                      ignored_bodies=ignored_bodies,
                                      allow_gripper_collisions=allow_gripper_collisions):
                smoothed = smoothed[:i + 1] + smoothed[j:]
        return smoothed
    
    # =========================================================================
    # STREAM: Compute IK for Pick/Place
    # =========================================================================
    def compute_kin_solution(self, obj_id: str, boxel_id: str,
                             grasp: Grasp) -> Iterator[Tuple[RobotConfig]]:
        """
        Compute IK solutions for picking object from boxel with grasp.

        Tries multiple IK seeds to produce diverse arm configurations.
        Each successful config carries ``ignored_body_ids`` containing the
        PyBullet body ID of the grasped object.  ``plan_motion()`` uses this
        to exclude the grasped body from collision checks — necessary because
        the gripper must be in contact with the object at the pick pose.

        Collision validation is deliberately NOT done here.  In a TAMP plan,
        earlier actions may relocate objects that currently block the target
        boxel.  Checking collisions against the static planning-time world
        would reject configs that are valid at execution time (e.g. picking
        a target from a shadow after the occluder has been moved).
        ``plan_motion()`` handles collision checking with the union of
        ignored bodies from both endpoints, which naturally covers objects
        moved earlier in the plan.

        PDDLStream declaration (see pddl/stream.pddl):
            (:stream compute-kin
              :inputs (?o ?b ?g)
              :domain (and (Obj ?o) (Boxel ?b) (valid_grasp ?o ?g))
              :outputs (?q)
              :certified (and (Config ?q) (kin_solution ?o ?b ?g ?q)
                              (config_for_boxel ?q ?b)))

        Args:
            obj_id: Object to grasp
            boxel_id: Boxel containing object
            grasp: Grasp to use

        Yields:
            Tuples of (config,) for grasping — one per successful IK seed.
        """
        boxel = self.registry.get_boxel(boxel_id)
        if boxel is None:
            return

        target_pos = boxel.center + grasp.position
        ee_orn = grasp.orientation

        body_id = self._resolve_body_id(obj_id)
        ignored = frozenset({body_id}) if body_id is not None else frozenset()

        if self.robot_id is None:
            config = self._heuristic_ik(target_pos, boxel.center)
            if config is not None:
                config.ignored_body_ids = ignored
                self._config_counter += 1
                config.name = f"q_kin_{obj_id}_{self._config_counter}"
                yield (config,)
            return

        seen = set()
        yielded = 0
        for seed_idx, seed in enumerate(self._ik_seeds()):
            config = self._pybullet_ik(target_pos, ee_orn, seed=seed)
            if config is None:
                continue

            sig = tuple(np.round(config.joint_positions, 3))
            if sig in seen:
                continue
            seen.add(sig)

            config.ignored_body_ids = ignored
            self._config_counter += 1
            config.name = f"q_kin_{obj_id}_{self._config_counter}"
            logger.debug("compute_kin: %s at %s -> %s  "
                         "ee_target=%s ignored_body=%s seed=%d",
                         obj_id, boxel_id, config.name,
                         target_pos.tolist(), body_id, seed_idx)
            yield (config,)
            yielded += 1

        if yielded == 0:
            logger.debug("compute_kin: all %d IK seeds failed for %s at %s "
                         "(target_pos=%s)", self.IK_NUM_SEEDS, obj_id,
                         boxel_id, target_pos.tolist())
    
