"""
Shared Franka Panda robot constants and low-level control utilities.

All joint limits, rest poses, and link indices for the Panda arm are defined
here once. Every module that needs robot parameters imports from this file.
"""

import numpy as np
import pybullet as p


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

def solve_ik(robot_id: int, target_pos: np.ndarray,
             target_orn=None) -> np.ndarray:
    """
    Null-space IK with rest-pose seed for consistent results.

    Saves the robot's current joint state, resets to REST_POSES to give
    the iterative solver a deterministic seed, runs IK with null-space
    bias, then restores the original state.  This makes results
    independent of where execution left the arm — critical for
    replanning after the robot has moved.

    Uses the same null-space parameters and seed-reset strategy as
    BoxelStreams._pybullet_ik() in streams.py, so execution IK
    matches planning IK.

    Args:
        robot_id: PyBullet body ID of the robot.
        target_pos: Desired end-effector position [x, y, z].
        target_orn: Desired orientation as quaternion [x, y, z, w] or
                    any sequence accepted by PyBullet.
                    Defaults to gripper pointing straight down.

    Returns:
        Array of 7 joint angles.
    """
    if target_orn is None:
        target_orn = p.getQuaternionFromEuler([0, np.pi, 0])

    orn_list = (target_orn.tolist() if isinstance(target_orn, np.ndarray)
                else list(target_orn))

    saved = [p.getJointState(robot_id, i)[0] for i in ARM_JOINT_INDICES]
    try:
        for i, angle in zip(ARM_JOINT_INDICES, REST_POSES):
            p.resetJointState(robot_id, i, angle)

        joints = p.calculateInverseKinematics(
            robot_id, END_EFFECTOR_LINK,
            target_pos.tolist(), orn_list,
            lowerLimits=JOINT_LIMITS_LOW.tolist(),
            upperLimits=JOINT_LIMITS_HIGH.tolist(),
            jointRanges=JOINT_RANGES.tolist(),
            restPoses=REST_POSES,
            maxNumIterations=100,
            residualThreshold=1e-4,
        )[:7]
        return np.array(joints)
    finally:
        for i, angle in zip(ARM_JOINT_INDICES, saved):
            p.resetJointState(robot_id, i, angle)


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
