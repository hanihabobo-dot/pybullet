#!/usr/bin/env python3
"""
Full TAMP Demo: Boxel Visualization + Robot Execution

This demonstrates:
1. BoxelTestEnv scene setup with objects, shadows, free space
2. Full boxel visualization  
3. Hidden object search with replanning

The target is randomly hidden in one of the shadow regions.
The robot must sense shadows to find it, then pick it up.

Run from Windows:
    pybullet_env\\Scripts\\python.exe run_tamp_demo.py
    
Note: This uses a mock planner for visualization. 
For real PDDLStream planning, run test_full_pipeline.py from WSL.
"""

import numpy as np
import pybullet as p
import time
import random

from boxel_test_env import BoxelTestEnv
from boxel_data import BoxelRegistry, BoxelType, create_boxel_registry_from_boxels
from streams import BoxelStreams
from cell_merger import merge_free_space_cells


# Settings
PHASE_WAIT_SECONDS = 1.0
ENABLE_FREE_SPACE = True


def main():
    print("=" * 60)
    print("TAMP Demo: Hidden Object Search with Replanning")
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
        
        env.draw_boxels(all_known + merged_free, duration=0, clear_previous=True)
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
    # PHASE 4: Hidden Object Scenario
    # =========================================================
    print("\n--- Phase 4: Hidden Object Search ---")
    
    # Get all target objects (both visible and hidden)
    all_targets = [name for name in env.objects.keys() if name.startswith("target")]
    print(f"  All targets in scene: {all_targets}")
    
    # Randomly select which target to search for
    target_name = random.choice(all_targets)
    target_info = env.objects[target_name]
    target_pos = np.array(target_info.position)
    
    # Randomly decide which shadow the target is "hidden" in
    # (For demo purposes - in reality, we'd use the actual occlusion)
    hidden_shadow_idx = random.randint(0, len(shadow_ids) - 1)
    hidden_shadow_id = shadow_ids[hidden_shadow_idx]
    
    print(f"\n  *** SCENARIO ***")
    print(f"  Target: {target_name}")
    print(f"  Hidden in: {hidden_shadow_id} (shadow {hidden_shadow_idx + 1} of {len(shadow_ids)})")
    print(f"  Robot must search {hidden_shadow_idx + 1} shadow(s) to find it!")
    
    # =========================================================
    # PHASE 5: Execute Search with Replanning
    # =========================================================
    print("\n--- Phase 5: Executing Search Plan ---")
    print("\nStarting robot execution in 2 seconds...")
    time.sleep(2)
    
    # Search shadows one by one (simulating optimistic planning + replanning)
    found = False
    search_count = 0
    
    for i, shadow_id in enumerate(shadow_ids):
        search_count += 1
        print(f"\n[Plan {search_count}] Searching shadow {i + 1}/{len(shadow_ids)}: {shadow_id}")
        
        # Step 1: Move to sensing config
        print(f"  -> Moving to sensing position...")
        sensing_configs = list(streams.sample_sensing_config(shadow_id))
        if sensing_configs:
            sensing_config = sensing_configs[0][0]
            move_robot_smooth(robot_id, sensing_config.joint_positions)
        
        wait_with_sim(env, 0.5)
        
        # Step 2: Sense
        print(f"  -> Sensing {shadow_id}...")
        time.sleep(0.5)
        
        # Check if this is the shadow where target is hidden
        if shadow_id == hidden_shadow_id:
            print(f"  -> *** TARGET FOUND in {shadow_id}! ***")
            found = True
            
            # Move to pick position
            print(f"  -> Moving to pick position...")
            pick_pos = target_pos + np.array([0, 0, 0.15])
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
            
            # Pick
            print(f"  -> Picking {target_name}...")
            close_gripper(robot_id)
            
            # Attach object
            target_id = env.objects[target_name].object_id
            constraint = p.createConstraint(
                robot_id, 11, target_id, -1,
                p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0]
            )
            
            # Lift
            print(f"  -> Lifting {target_name}...")
            lift_pos = target_pos + np.array([0, 0, 0.3])
            lift_joints = p.calculateInverseKinematics(
                robot_id, 11, lift_pos.tolist(), pick_orn,
                lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
                upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
                jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
                restPoses=rest_pose
            )[:7]
            move_robot_smooth(robot_id, lift_joints)
            
            break
        else:
            print(f"  -> Target NOT in {shadow_id}. Replanning...")
            wait_with_sim(env, 0.3)
    
    # =========================================================
    # PHASE 6: Done
    # =========================================================
    print("\n" + "=" * 60)
    if found:
        print(f"SUCCESS! Found {target_name} after searching {search_count} shadow(s)")
        print(f"  - {search_count - 1} replan(s) were needed")
    else:
        print("FAILED: Target not found in any shadow")
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
