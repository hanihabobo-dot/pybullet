"""
Shared Franka Panda robot constants and low-level control utilities.

All joint limits, rest poses, and link indices for the Panda arm are defined
here once. Every module that needs robot parameters imports from this file.
"""

import logging
import numpy as np
import pybullet as p

logger = logging.getLogger(__name__)


# =============================================================================
# Franka Panda Constants (from the Franka Emika Panda datasheet)
# =============================================================================

ARM_JOINT_INDICES = [0, 1, 2, 3, 4, 5, 6]
FINGER_JOINTS = [9, 10]  # panda_finger_joint1, panda_finger_joint2
END_EFFECTOR_LINK = 11   # panda_grasptarget

JOINT_LIMITS_LOW = np.array([
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973
])
JOINT_LIMITS_HIGH = np.array([
    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973
])
JOINT_RANGES = JOINT_LIMITS_HIGH - JOINT_LIMITS_LOW

REST_POSES = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]


# =============================================================================
# Low-level control utilities
# =============================================================================

# =============================================================================
# Collision checking for motion planning
# =============================================================================

# Self-collision pairs to ignore: adjacent links in the Panda kinematic chain
# plus finger/hand pairs that naturally overlap.
_PANDA_IGNORED_SELF_PAIRS = frozenset({
    (-1, 0), (0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6),
    (6, 7), (7, 8), (8, 9), (8, 10), (9, 10), (9, 11), (10, 11),
})


_PANDA_GRIPPER_LINKS = frozenset({6, 7, 8, 9, 10, 11})


def is_config_collision_free(robot_id: int, joint_positions,
                              physics_client: int = 0,
                              ignored_bodies=None,
                              allow_gripper_collisions: bool = False,
                              log_collisions: bool = True,
                              _rendering_managed: bool = False) -> bool:
    """
    Check whether a 7-DOF arm configuration is collision-free.

    Saves the robot's current joint state, sets the query configuration,
    runs broadphase + narrowphase collision detection, then restores the
    original state.  Contacts are filtered:

    * Self-contacts between adjacent / structurally overlapping links
      (defined in ``_PANDA_IGNORED_SELF_PAIRS``) are ignored.
    * The base link (-1) is excluded — it is fixed and rests on the
      mounting surface.
    * Bodies listed in *ignored_bodies* (e.g. a held object) are skipped.
    * When *allow_gripper_collisions* is True, collisions between the
      gripper links (7-11: wrist flange, hand, fingers, grasptarget)
      and non-robot bodies are also ignored.  Use this for pick/place
      endpoint checks where the gripper must enter cluttered space.

    Args:
        robot_id:                 PyBullet body ID of the robot.
        joint_positions:          Sequence of 7 target joint angles.
        physics_client:           PyBullet physics client ID.
        ignored_bodies:           Optional set/frozenset of body IDs to skip.
        allow_gripper_collisions: If True, exempt gripper links from
                                  environment collision reporting.
        log_collisions:           If True, log the first collision found
                                  at DEBUG level.
        _rendering_managed:       If True, caller already disabled rendering;
                                  skip the enable/disable toggle here.

    Returns:
        True if the configuration has no disallowed contacts.
    """
    if ignored_bodies is None:
        ignored_bodies = frozenset()

    saved = [p.getJointState(robot_id, i, physicsClientId=physics_client)[0]
             for i in ARM_JOINT_INDICES]
    if not _rendering_managed:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0,
                                   physicsClientId=physics_client)
    try:
        for i, angle in zip(ARM_JOINT_INDICES, joint_positions):
            p.resetJointState(robot_id, i, angle,
                              physicsClientId=physics_client)

        p.performCollisionDetection(physicsClientId=physics_client)
        contacts = p.getContactPoints(bodyA=robot_id,
                                      physicsClientId=physics_client)

        for c in contacts:
            body_a, body_b, link_a, link_b = c[1], c[2], c[3], c[4]

            if body_a == robot_id and body_b == robot_id:
                pair = (min(link_a, link_b), max(link_a, link_b))
                if pair not in _PANDA_IGNORED_SELF_PAIRS:
                    if log_collisions:
                        logger.debug("collision: self-contact links (%d, %d)",
                                     link_a, link_b)
                    return False
                continue

            other = body_b if body_a == robot_id else body_a
            robot_link = link_a if body_a == robot_id else link_b

            if other in ignored_bodies:
                continue
            if robot_link == -1:
                continue
            if allow_gripper_collisions and robot_link in _PANDA_GRIPPER_LINKS:
                continue

            if log_collisions:
                logger.debug("collision: robot link %d <-> body %d (link %d)",
                             robot_link,
                             other,
                             link_b if body_a == robot_id else link_a)
            return False

        return True
    finally:
        for i, angle in zip(ARM_JOINT_INDICES, saved):
            p.resetJointState(robot_id, i, angle,
                              physicsClientId=physics_client)
        if not _rendering_managed:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1,
                                       physicsClientId=physics_client)


def is_path_collision_free(robot_id: int, q_start, q_end,
                            physics_client: int = 0, n_checks: int = 8,
                            ignored_bodies=None,
                            allow_gripper_collisions: bool = False) -> bool:
    """
    Check a straight-line joint-space path for collisions.

    Evaluates *n_checks* evenly-spaced configurations (including the
    endpoints) along the linear interpolation from *q_start* to *q_end*.

    Args:
        robot_id:        PyBullet body ID of the robot.
        q_start:         Start joint positions (array-like, length 7).
        q_end:           End joint positions (array-like, length 7).
        physics_client:  PyBullet physics client ID.
        n_checks:        Number of intermediate configurations to test.
        ignored_bodies:  Optional set/frozenset of body IDs to skip.
        allow_gripper_collisions: If True, exempt gripper/wrist links
            from environment collision reporting (same as in
            is_config_collision_free).

    Returns:
        True if every sampled configuration is collision-free.
    """
    q_s = np.asarray(q_start, dtype=float)
    q_e = np.asarray(q_end, dtype=float)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0,
                               physicsClientId=physics_client)
    try:
        for t in np.linspace(0.0, 1.0, n_checks):
            q = (1.0 - t) * q_s + t * q_e
            if not is_config_collision_free(robot_id, q, physics_client,
                                            ignored_bodies,
                                            allow_gripper_collisions=allow_gripper_collisions,
                                            log_collisions=False,
                                            _rendering_managed=True):
                return False
        return True
    finally:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1,
                                   physicsClientId=physics_client)


# =============================================================================
# Inverse kinematics
# =============================================================================

def solve_ik(robot_id: int, target_pos: np.ndarray,
             target_orn=None, physics_client: int = 0):
    """
    Null-space IK with rest-pose seed for consistent results.

    Saves the robot's current joint state, resets to REST_POSES to give
    the iterative solver a deterministic seed, runs IK with null-space
    bias, then restores the original state.  This makes results
    independent of where execution left the arm — critical for
    replanning after the robot has moved.

    Applies the same validation as ``BoxelStreams._pybullet_ik()`` in
    streams.py: null-check, joint-limit check (0.1 rad tolerance), and
    clipping.  Returns ``None`` on failure so callers can handle it.

    Args:
        robot_id: PyBullet body ID of the robot.
        target_pos: Desired end-effector position [x, y, z].
        target_orn: Desired orientation as quaternion [x, y, z, w] or
                    any sequence accepted by PyBullet.
                    Defaults to gripper pointing straight down.
        physics_client: PyBullet physics client ID.

    Returns:
        Array of 7 joint angles, or ``None`` if IK failed.
    """
    if target_orn is None:
        target_orn = p.getQuaternionFromEuler([0, np.pi, 0])

    orn_list = (target_orn.tolist() if isinstance(target_orn, np.ndarray)
                else list(target_orn))

    saved = [p.getJointState(robot_id, i,
                             physicsClientId=physics_client)[0]
             for i in ARM_JOINT_INDICES]
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0,
                               physicsClientId=physics_client)
    try:
        for i, angle in zip(ARM_JOINT_INDICES, REST_POSES):
            p.resetJointState(robot_id, i, angle,
                              physicsClientId=physics_client)

        joint_positions = p.calculateInverseKinematics(
            robot_id, END_EFFECTOR_LINK,
            target_pos.tolist(), orn_list,
            lowerLimits=JOINT_LIMITS_LOW.tolist(),
            upperLimits=JOINT_LIMITS_HIGH.tolist(),
            jointRanges=JOINT_RANGES.tolist(),
            restPoses=REST_POSES,
            maxNumIterations=100,
            residualThreshold=1e-4,
            physicsClientId=physics_client,
        )

        if joint_positions is None or len(joint_positions) < 7:
            return None

        arm_joints = np.array(joint_positions[:7])

        if np.any(arm_joints < JOINT_LIMITS_LOW - 0.1) or \
           np.any(arm_joints > JOINT_LIMITS_HIGH + 0.1):
            return None

        return np.clip(arm_joints, JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)

    except Exception as e:
        logger.warning("solve_ik failed for pos=%s: %s", target_pos.tolist(), e)
        return None
    finally:
        for i, angle in zip(ARM_JOINT_INDICES, saved):
            p.resetJointState(robot_id, i, angle,
                              physicsClientId=physics_client)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1,
                                   physicsClientId=physics_client)


def move_robot_smooth(robot_id: int, target_joints, gui: bool = True,
                      steps: int = 60):
    """
    Smoothly interpolate joint positions from the current state to
    *target_joints*.

    Args:
        robot_id: PyBullet body ID of the robot.
        target_joints: Sequence of 7 target joint angles.
        gui: If True, sleep between steps for real-time visualisation.
        steps: Number of interpolation steps.
    """
    import time
    current = [p.getJointState(robot_id, i)[0] for i in range(7)]
    for t in range(steps):
        alpha = (t + 1) / steps
        interp = [(1 - alpha) * c + alpha * tgt
                   for c, tgt in zip(current, target_joints)]
        for i in range(7):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL,
                                    targetPosition=interp[i], force=240)
        p.stepSimulation()
        if gui:
            time.sleep(1 / 120)


def open_gripper(robot_id: int, gui: bool = True):
    """Open the Panda gripper (finger width ~0.08 m)."""
    import time
    for _ in range(30):
        p.setJointMotorControl2(robot_id, FINGER_JOINTS[0],
                                p.POSITION_CONTROL,
                                targetPosition=0.04, force=50)
        p.setJointMotorControl2(robot_id, FINGER_JOINTS[1],
                                p.POSITION_CONTROL,
                                targetPosition=0.04, force=50)
        p.stepSimulation()
        if gui:
            time.sleep(1 / 120)


def close_gripper(robot_id: int, gui: bool = True):
    """Close the Panda gripper."""
    import time
    for _ in range(30):
        p.setJointMotorControl2(robot_id, FINGER_JOINTS[0],
                                p.POSITION_CONTROL,
                                targetPosition=0.01, force=50)
        p.setJointMotorControl2(robot_id, FINGER_JOINTS[1],
                                p.POSITION_CONTROL,
                                targetPosition=0.01, force=50)
        p.stepSimulation()
        if gui:
            time.sleep(1 / 120)
