#!/usr/bin/env python3
"""
Full Pipeline Test: PDDLStream Planning + PyBullet Execution

This uses the REAL PDDLStream planner (not mock) with GUI visualization.

Run from WSL:
    source wsl_env/bin/activate
    export PYTHONPATH=/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib
    python3 test_full_pipeline.py

Or with no GUI (for testing):
    python3 test_full_pipeline.py --no-gui
"""

import sys
import os
import argparse
import random
import time

# Add pddlstream to path (for WSL)
PDDLSTREAM_PATH = '/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib'
if os.path.exists(PDDLSTREAM_PATH):
    sys.path.insert(0, PDDLSTREAM_PATH)

import numpy as np
import pybullet as p
import pybullet_data

from boxel_test_env import BoxelTestEnv
from boxel_data import BoxelRegistry, BoxelType, create_boxel_registry_from_boxels
from streams import BoxelStreams
from cell_merger import merge_free_space_cells
from pddlstream_planner import PDDLStreamPlanner


def main(gui=True):
    print("=" * 60)
    print("FULL PIPELINE: Real PDDLStream + PyBullet Execution")
    print("=" * 60)
    
    # =========================================================
    # PHASE 1: Setup Environment (same as run_demo.py)
    # =========================================================
    print("\n--- Phase 1: Environment Setup ---")
    env = BoxelTestEnv(gui=gui)
    
    robot_id = env.objects["robot"].object_id
    print(f"Robot ID: {robot_id}")
    
    # Let objects settle
    print("Letting objects settle...")
    for _ in range(100):
        env.step_simulation()
        time.sleep(1.0 / 240.0) if gui else None
    
    # Get camera observation with boxels
    print("Capturing camera observation...")
    obs = env.get_camera_observation()
    
    print(f"  Visible objects: {obs.visible_objects}")
    print(f"  Boxels generated: {len(obs.boxels) if obs.boxels else 0}")
    
    # =========================================================
    # PHASE 2: Visualize Boxels (if GUI)
    # =========================================================
    if gui:
        print("\n--- Phase 2: Boxel Visualization ---")
        
        all_known = obs.boxels
        obj_boxels = [b for b in all_known if not b.is_shadow]
        shadow_boxels = [b for b in all_known if b.is_shadow]
        
        print(f"  Object boxels: {len(obj_boxels)}")
        print(f"  Shadow boxels: {len(shadow_boxels)}")
        
        # Set camera view (same as run_demo.py)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0.5, 0.0, env.table_surface_height]
        )
        
        # Show boxels
        env.draw_boxels(all_known, duration=0)
        
        # Generate and show free space
        print("Generating free space...")
        free_boxels = env.generate_free_space(all_known, visualize=False)
        merged_free = merge_free_space_cells(free_boxels)
        print(f"  Merged: {len(free_boxels)} -> {len(merged_free)} boxels")
        
        env.draw_boxels(all_known + merged_free, duration=0, clear_previous=True)
        all_known = all_known + merged_free
        
        for _ in range(int(240 * 1.0)):
            env.step_simulation()
            time.sleep(1.0/240.0)
    else:
        all_known = obs.boxels
        free_boxels = env.generate_free_space(all_known, visualize=False)
        merged_free = merge_free_space_cells(free_boxels)
        all_known = all_known + merged_free
    
    # =========================================================
    # PHASE 3: Create Registry
    # =========================================================
    print("\n--- Phase 3: Creating BoxelRegistry ---")
    registry = create_boxel_registry_from_boxels(all_known, env.table_surface_height)
    registry.save_to_json("boxel_data.json")
    
    shadows = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    print(f"  Saved {len(registry.boxels)} boxels, {len(shadows)} shadows")
    
    # =========================================================
    # PHASE 4: Select Random Target & Hidden Location
    # =========================================================
    print("\n--- Phase 4: Hidden Object Scenario ---")
    
    all_targets = [name for name in env.objects.keys() if name.startswith("target")]
    target_name = random.choice(all_targets)
    target_info = env.objects[target_name]
    target_pos = np.array(target_info.position)
    
    # Randomly hide in a shadow
    hidden_shadow_idx = random.randint(0, len(shadows) - 1)
    hidden_shadow_id = shadows[hidden_shadow_idx]
    
    print(f"  Target: {target_name}")
    print(f"  Hidden in: {hidden_shadow_id} (shadow {hidden_shadow_idx + 1} of {len(shadows)})")
    
    # =========================================================
    # PHASE 5: PDDLStream Planning (REAL PLANNER)
    # =========================================================
    print("\n--- Phase 5: PDDLStream Planning ---")
    
    streams = BoxelStreams(registry, robot_id=robot_id, physics_client=0)
    planner = PDDLStreamPlanner(registry, robot_id=robot_id)
    
    print(f"Planning to find and hold {target_name}...")
    
    plan = planner.plan(
        target_objects=[target_name],
        goal=f'(holding {target_name})',
        max_time=60.0,
        verbose=True
    )
    
    if plan is None:
        print("ERROR: No plan found!")
        env.close()
        return False
    
    print(f"\nPlan found with {len(plan)} actions:")
    for i, action in enumerate(plan):
        print(f"  {i+1}. {action[0]} {' '.join(str(a) for a in action[1:])}")
    
    # =========================================================
    # PHASE 6: Execute Plan with Robot
    # =========================================================
    print("\n--- Phase 6: Executing Plan ---")
    
    if gui:
        print("Starting execution in 2 seconds...")
        time.sleep(2)
    
    # Execute each action
    for i, action in enumerate(plan):
        action_name = action[0]
        params = action[1:]
        
        print(f"\n[{i+1}/{len(plan)}] {action_name}")
        
        if action_name == 'move':
            q1, q2, traj = params
            print(f"  Moving from {q1} to {q2}...")
            
            # Get target config from streams
            target_config = None
            if hasattr(q2, 'joint_positions'):
                target_config = q2.joint_positions
            else:
                # Try to generate config
                for shadow_id in shadows:
                    configs = list(streams.sample_sensing_config(shadow_id))
                    for cfg, in configs:
                        if cfg.name == str(q2):
                            target_config = cfg.joint_positions
                            break
                    if target_config is not None:
                        break
            
            if target_config is not None:
                move_robot_smooth(robot_id, target_config, gui)
            
        elif action_name == 'sense_boxel':
            obj, boxel_id, config = params
            print(f"  Sensing {boxel_id} for {obj}...")
            
            # Check oracle
            if str(boxel_id) == hidden_shadow_id:
                print(f"  -> TARGET FOUND!")
            else:
                print(f"  -> Not here, would replan...")
            
            if gui:
                time.sleep(0.5)
                
        elif action_name == 'pick':
            obj, boxel_id, grasp, config = params
            print(f"  Picking {obj} from {boxel_id}...")
            
            # Move to pick position
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
            move_robot_smooth(robot_id, pick_joints, gui)
            
            # Lower and grasp
            grasp_pos = target_pos + np.array([0, 0, 0.05])
            grasp_joints = p.calculateInverseKinematics(
                robot_id, 11, grasp_pos.tolist(), pick_orn,
                lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
                upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
                jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
                restPoses=rest_pose
            )[:7]
            move_robot_smooth(robot_id, grasp_joints, gui)
            
            # Close gripper
            close_gripper(robot_id, gui)
            
            # Attach object
            target_id = env.objects[target_name].object_id
            p.createConstraint(robot_id, 11, target_id, -1,
                              p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
            
            # Lift
            lift_pos = target_pos + np.array([0, 0, 0.3])
            lift_joints = p.calculateInverseKinematics(
                robot_id, 11, lift_pos.tolist(), pick_orn,
                lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
                upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
                jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
                restPoses=rest_pose
            )[:7]
            move_robot_smooth(robot_id, lift_joints, gui)
            print(f"  -> {obj} picked up!")
    
    # =========================================================
    # PHASE 7: Done
    # =========================================================
    print("\n" + "=" * 60)
    print(f"SUCCESS! Real PDDLStream planned and executed!")
    print(f"  Target: {target_name}")
    print(f"  Plan length: {len(plan)} actions")
    print("=" * 60)
    
    if gui:
        print("\nDemo complete. Window stays open for 15 seconds...")
        print("Use mouse to rotate view (Ctrl+Left click)")
        for _ in range(int(240 * 15)):
            env.step_simulation()
            time.sleep(1.0/240.0)
    
    env.close()
    return True


def move_robot_smooth(robot_id, target_joints, gui=True, steps=150):
    """Smoothly move robot to target configuration."""
    current = [p.getJointState(robot_id, i)[0] for i in range(7)]
    
    for t in range(steps):
        alpha = (t + 1) / steps
        interp = [(1 - alpha) * c + alpha * tgt for c, tgt in zip(current, target_joints)]
        
        for i in range(7):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL,
                                    targetPosition=interp[i], force=240)
        
        p.stepSimulation()
        if gui:
            time.sleep(1/240)


def close_gripper(robot_id, gui=True):
    """Close gripper."""
    for _ in range(100):
        p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.stepSimulation()
        if gui:
            time.sleep(1/240)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Full PDDLStream Pipeline Test')
    parser.add_argument('--no-gui', action='store_true', help='Run without GUI')
    args = parser.parse_args()
    
    success = main(gui=not args.no_gui)
    sys.exit(0 if success else 1)
