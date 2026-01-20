#!/usr/bin/env python3
"""
Full TAMP Demo: Boxel Visualization + Robot Execution

This combines:
1. BoxelTestEnv scene setup with objects, shadows, free space
2. Full boxel visualization  
3. TAMP plan execution with the robot

Run from Windows:
    pybullet_env\\Scripts\\python.exe run_tamp_demo.py
"""

import numpy as np
import pybullet as p
import time

from boxel_test_env import BoxelTestEnv
from boxel_data import BoxelRegistry, BoxelType, create_boxel_registry_from_boxels
from streams import BoxelStreams
from cell_merger import merge_free_space_cells


# Settings
PHASE_WAIT_SECONDS = 1.0
ENABLE_FREE_SPACE = True


def main():
    print("=" * 60)
    print("TAMP Demo: Boxel Visualization + Robot Execution")
    print("=" * 60)
    
    # =========================================================
    # PHASE 1: Setup Environment & Generate Boxels
    # =========================================================
    print("\n--- Phase 1: Environment Setup ---")
    env = BoxelTestEnv(gui=True)
    
    # Get robot ID from environment
    robot_id = env.objects["robot"].object_id
    print(f"Robot ID: {robot_id}")
    
    # Let objects settle
    print("Letting objects settle...")
    for _ in range(100):
        env.step_simulation()
        time.sleep(1.0 / 240.0)
    
    # Get camera observation with boxels
    print("Capturing camera observation...")
    obs = env.get_camera_observation()
    
    print(f"  Visible objects: {obs.visible_objects}")
    print(f"  Boxels generated: {len(obs.boxels) if obs.boxels else 0}")
    
    # =========================================================
    # PHASE 2: Visualize Boxels
    # =========================================================
    print("\n--- Phase 2: Boxel Visualization ---")
    
    all_known = obs.boxels
    obj_boxels = [b for b in all_known if not b.is_shadow]
    shadow_boxels = [b for b in all_known if b.is_shadow]
    
    print(f"  Object boxels: {len(obj_boxels)}")
    print(f"  Shadow boxels: {len(shadow_boxels)}")
    
    # Set camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0.0, env.table_surface_height]
    )
    
    # Show objects
    print("\nShowing object boxels...")
    env.draw_boxels(obj_boxels, duration=0)
    wait_with_sim(env, PHASE_WAIT_SECONDS)
    
    # Show shadows
    print("Showing shadow boxels...")
    env.draw_boxels(all_known, duration=0)
    wait_with_sim(env, PHASE_WAIT_SECONDS)
    
    # Generate free space
    if ENABLE_FREE_SPACE:
        print("Generating free space...")
        free_boxels = env.generate_free_space(all_known, visualize=False)
        print(f"  Generated {len(free_boxels)} free space boxels")
        
        print("Merging free space...")
        merged_free = merge_free_space_cells(free_boxels)
        print(f"  Merged: {len(free_boxels)} -> {len(merged_free)} boxels")
        
        env.draw_boxels(all_known + merged_free, duration=0, 
                       clear_previous=True)
        free_boxels = merged_free
    else:
        free_boxels = []
    
    wait_with_sim(env, PHASE_WAIT_SECONDS)
    
    # =========================================================
    # PHASE 3: Create Registry & Setup TAMP
    # =========================================================
    print("\n--- Phase 3: TAMP Setup ---")
    
    all_final_boxels = all_known + free_boxels
    registry = create_boxel_registry_from_boxels(all_final_boxels, env.table_surface_height)
    registry.save_to_json("boxel_data.json")
    print(f"  Saved {len(registry.boxels)} boxels to registry")
    
    # Get shadow IDs for planning
    shadow_ids = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    print(f"  Shadow regions: {shadow_ids}")
    
    # Create streams with real IK
    streams = BoxelStreams(registry, robot_id=robot_id, physics_client=0)
    
    # =========================================================
    # PHASE 4: Execute TAMP Plan
    # =========================================================
    print("\n--- Phase 4: Robot Execution ---")
    print("\nSimulating TAMP plan execution:")
    print("  1. Move to sensing position")
    print("  2. Sense shadow region")
    print("  3. Move to pick position")
    print("  4. Pick object")
    
    print("\nStarting robot execution in 2 seconds...")
    time.sleep(2)
    
    # Step 1: Move to sensing config for first shadow
    print("\n[1/4] Moving to sensing position...")
    if shadow_ids:
        sensing_configs = list(streams.sample_sensing_config(shadow_ids[0]))
        if sensing_configs:
            sensing_config = sensing_configs[0][0]
            move_robot_smooth(robot_id, sensing_config.joint_positions)
            print(f"      Reached sensing config: {sensing_config.name}")
    
    wait_with_sim(env, 1.0)
    
    # Step 2: Sense (simulate observation)
    print("\n[2/4] Sensing shadow region...")
    print("      Observation: Target FOUND!")
    time.sleep(1.0)
    
    # Step 3: Move to pick position
    print("\n[3/4] Moving to pick position...")
    
    # Find target object position (use first visible target)
    target_name = None
    target_pos = None
    for name, info in env.objects.items():
        if name.startswith("target") and info.is_visible:
            target_name = name
            target_pos = info.position
            break
    
    if target_pos is not None:
        print(f"      Target: {target_name} at {target_pos}")
        
        # Compute pick config
        pick_pos = target_pos + np.array([0, 0, 0.15])  # Above target
        pick_orn = p.getQuaternionFromEuler([0, np.pi, 0])
        
        rest_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        pick_joints = p.calculateInverseKinematics(
            robot_id, 11, pick_pos.tolist(), pick_orn,
            lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
            upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
            jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
            restPoses=rest_pose
        )[:7]
        
        move_robot_smooth(robot_id, pick_joints)
        
        # Lower to grasp
        grasp_pos = target_pos + np.array([0, 0, 0.05])
        grasp_joints = p.calculateInverseKinematics(
            robot_id, 11, grasp_pos.tolist(), pick_orn,
            lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
            upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
            jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
            restPoses=rest_pose
        )[:7]
        move_robot_smooth(robot_id, grasp_joints)
    
    wait_with_sim(env, 0.5)
    
    # Step 4: Pick (close gripper)
    print("\n[4/4] Picking object...")
    close_gripper(robot_id)
    
    # Attach object
    if target_name:
        target_id = env.objects[target_name].object_id
        constraint = p.createConstraint(
            robot_id, 11, target_id, -1,
            p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0]
        )
    
    # Lift
    print("      Lifting object...")
    if target_pos is not None:
        lift_pos = target_pos + np.array([0, 0, 0.3])
        lift_joints = p.calculateInverseKinematics(
            robot_id, 11, lift_pos.tolist(), pick_orn,
            lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
            upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
            jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
            restPoses=rest_pose
        )[:7]
        move_robot_smooth(robot_id, lift_joints)
    
    # =========================================================
    # PHASE 5: Done
    # =========================================================
    print("\n" + "=" * 60)
    print("TAMP DEMO COMPLETE!")
    print("  - Generated semantic boxels (objects, shadows, free space)")
    print("  - Executed sense-move-pick plan")
    print("=" * 60)
    
    print("\nDemo complete. Window stays open for 15 seconds...")
    print("Use mouse to rotate view (Ctrl+Left click)")
    
    for _ in range(int(240 * 15)):
        env.step_simulation()
        time.sleep(1.0/240.0)
    
    env.close()


def wait_with_sim(env, seconds):
    """Wait while stepping simulation."""
    for _ in range(int(240 * seconds)):
        env.step_simulation()
        time.sleep(1.0/240.0)


def move_robot_smooth(robot_id, target_joints, steps=150):
    """Smoothly move robot to target configuration."""
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
    """Close gripper."""
    for _ in range(100):
        p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.stepSimulation()
        time.sleep(1/240)


if __name__ == "__main__":
    main()
