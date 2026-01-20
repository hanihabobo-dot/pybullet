#!/usr/bin/env python3
"""
GUI Demo: Watch the robot execute the plan in PyBullet.

Run from Windows:
    pybullet_env\\Scripts\\python.exe run_gui_demo.py
"""

import time
import numpy as np
import pybullet as p
import pybullet_data

from boxel_data import BoxelRegistry, BoxelType
from streams import BoxelStreams


def main():
    print("="*50)
    print("GUI DEMO: Robot Execution")
    print("="*50)
    
    # Setup PyBullet with GUI
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)
    
    # Camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0, 0.3]
    )
    
    # Load robot
    robot_id = p.loadURDF('franka_panda/panda.urdf', [0, 0, 0], useFixedBase=True)
    
    # Load table
    table_id = p.loadURDF('table/table.urdf', [0.5, 0, 0], useFixedBase=True)
    
    # Add a target object (red cube)
    target_pos = [0.5, 0.2, 0.65]
    target_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03], rgbaColor=[1, 0, 0, 1])
    target_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
    target_body = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=target_col, 
                                     baseVisualShapeIndex=target_id, basePosition=target_pos)
    
    # Set rest pose
    rest_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    for i in range(7):
        p.resetJointState(robot_id, i, rest_pose[i])
    
    # Open gripper
    p.resetJointState(robot_id, 9, 0.04)
    p.resetJointState(robot_id, 10, 0.04)
    
    print("\nRobot ready. Starting in 2 seconds...")
    time.sleep(2)
    
    # Load registry and generate configs
    registry = BoxelRegistry.load_from_json('boxel_data.json')
    streams = BoxelStreams(registry, robot_id=robot_id, physics_client=physics_client)
    
    shadows = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    print(f"Found {len(shadows)} shadow regions")
    
    # Hard-coded plan (from PDDLStream output):
    # 1. move to sensing config
    # 2. sense shadow
    # 3. move to pick config  
    # 4. pick object
    
    print("\n--- Executing Plan ---")
    
    # Step 1: Move to sensing position
    print("\n1. Moving to sensing position...")
    sensing_configs = list(streams.sample_sensing_config(shadows[0]))
    if sensing_configs:
        sensing_config = sensing_configs[0][0]
        move_robot(robot_id, sensing_config.joint_positions)
    
    time.sleep(0.5)
    
    # Step 2: Sense (just pause and "observe")
    print("2. Sensing shadow region... FOUND target!")
    time.sleep(1.0)
    
    # Step 3: Move to pick position (above target)
    print("3. Moving to pick position...")
    pick_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.15]
    pick_orn = p.getQuaternionFromEuler([0, np.pi, 0])
    
    pick_joints = p.calculateInverseKinematics(
        robot_id, 11, pick_pos, pick_orn,
        lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
        upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
        jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
        restPoses=rest_pose
    )[:7]
    move_robot(robot_id, pick_joints)
    
    time.sleep(0.5)
    
    # Step 4: Lower to grasp
    print("4. Lowering to grasp...")
    grasp_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.05]
    grasp_joints = p.calculateInverseKinematics(
        robot_id, 11, grasp_pos, pick_orn,
        lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
        upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
        jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
        restPoses=rest_pose
    )[:7]
    move_robot(robot_id, grasp_joints)
    
    # Step 5: Close gripper
    print("5. Closing gripper...")
    close_gripper(robot_id)
    
    # Attach object (create constraint)
    constraint = p.createConstraint(
        robot_id, 11, target_body, -1,
        p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0]
    )
    
    time.sleep(0.5)
    
    # Step 6: Lift
    print("6. Lifting object...")
    lift_joints = list(grasp_joints)
    lift_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.3]
    lift_joints = p.calculateInverseKinematics(
        robot_id, 11, lift_pos, pick_orn,
        lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
        upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
        jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
        restPoses=rest_pose
    )[:7]
    move_robot(robot_id, lift_joints)
    
    print("\n" + "="*50)
    print("SUCCESS! Object picked up.")
    print("="*50)
    print("\nDemo complete. Window stays open for 10 seconds...")
    time.sleep(10)
    
    p.disconnect()


def move_robot(robot_id, target_joints, steps=100):
    """Smoothly move robot to target joint configuration."""
    current = [p.getJointState(robot_id, i)[0] for i in range(7)]
    
    for t in range(steps):
        alpha = (t + 1) / steps
        interp = [(1 - alpha) * c + alpha * tgt for c, tgt in zip(current, target_joints)]
        
        for i in range(7):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, 
                                    targetPosition=interp[i], force=240)
        
        p.stepSimulation()
        time.sleep(1/240)


def close_gripper(robot_id):
    """Close the gripper."""
    for _ in range(100):
        p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.stepSimulation()
        time.sleep(1/240)


if __name__ == "__main__":
    main()
