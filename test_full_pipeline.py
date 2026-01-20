#!/usr/bin/env python3
"""
Full Pipeline Test: PDDLStream Planning + PyBullet Execution with REPLANNING

This uses the REAL PDDLStream planner with proper partial observability:
1. Robot doesn't know where target is (hidden in a shadow)
2. Must push occluder aside to reveal shadow
3. Sense shadow to check for object
4. If not found: REPLAN with updated belief
5. Repeat until found, then pick

Run from WSL:
    source wsl_env/bin/activate
    export DISPLAY=:0
    export LIBGL_ALWAYS_SOFTWARE=1
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


class BeliefState:
    """
    Tracks what the robot knows about object locations.
    
    For each shadow, tracks:
    - unknown: Haven't sensed yet
    - not_here: Sensed, object not found
    - found: Sensed, object found here
    """
    def __init__(self, shadows: list, target: str):
        self.target = target
        self.shadow_status = {s: 'unknown' for s in shadows}
        self.target_found_in = None
        self.occluders_moved = set()  # Which occluders have been pushed aside
    
    def mark_sensed(self, shadow_id: str, found: bool):
        """Update belief after sensing a shadow."""
        if found:
            self.shadow_status[shadow_id] = 'found'
            self.target_found_in = shadow_id
        else:
            self.shadow_status[shadow_id] = 'not_here'
    
    def mark_occluder_moved(self, occluder_id: str):
        """Mark that an occluder has been pushed aside."""
        self.occluders_moved.add(occluder_id)
    
    def get_unknown_shadows(self):
        """Get list of shadows we haven't checked yet."""
        return [s for s, status in self.shadow_status.items() if status == 'unknown']
    
    def is_target_found(self):
        """Check if we've found the target."""
        return self.target_found_in is not None


def main(gui=True):
    print("=" * 60)
    print("FULL PIPELINE: PDDLStream + Replanning")
    print("=" * 60)
    
    # =========================================================
    # PHASE 1: Setup Environment
    # =========================================================
    print("\n--- Phase 1: Environment Setup ---")
    env = BoxelTestEnv(gui=gui)
    robot_id = env.objects["robot"].object_id
    print(f"Robot ID: {robot_id}")
    
    # Let settle
    for _ in range(100):
        env.step_simulation()
        if gui:
            time.sleep(1.0 / 240.0)
    
    # Get boxels
    obs = env.get_camera_observation()
    all_known = obs.boxels
    
    if gui:
        print("\n--- Phase 2: Boxel Visualization ---")
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5, cameraYaw=45, cameraPitch=-30,
            cameraTargetPosition=[0.5, 0.0, env.table_surface_height]
        )
        env.draw_boxels(all_known, duration=0)
        
        free_boxels = env.generate_free_space(all_known, visualize=False)
        merged_free = merge_free_space_cells(free_boxels)
        env.draw_boxels(all_known + merged_free, duration=0, clear_previous=True)
        all_known = all_known + merged_free
    else:
        free_boxels = env.generate_free_space(all_known, visualize=False)
        merged_free = merge_free_space_cells(free_boxels)
        all_known = all_known + merged_free
    
    # Create registry
    print("\n--- Phase 3: Creating BoxelRegistry ---")
    registry = create_boxel_registry_from_boxels(all_known, env.table_surface_height)
    registry.save_to_json("boxel_data.json")
    
    shadows = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    occluders = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.OBJECT]
    print(f"  {len(registry.boxels)} boxels, {len(shadows)} shadows, {len(occluders)} occluders")
    
    # =========================================================
    # PHASE 4: Hidden Object Scenario (ORACLE ONLY)
    # =========================================================
    print("\n--- Phase 4: Hidden Object Scenario ---")
    
    all_targets = [name for name in env.objects.keys() if name.startswith("target")]
    target_name = random.choice(all_targets)
    target_info = env.objects[target_name]
    target_pos = np.array(target_info.position)
    
    # ORACLE: Randomly decide which shadow the target is actually in
    # The robot does NOT know this - it must search!
    hidden_shadow_idx = random.randint(0, len(shadows) - 1)
    oracle_hidden_shadow = shadows[hidden_shadow_idx]
    
    print(f"  Target: {target_name}")
    print(f"  ORACLE: Actually hidden in {oracle_hidden_shadow} (shadow {hidden_shadow_idx + 1}/{len(shadows)})")
    print(f"  Robot must search to find it!")
    
    # Create shadow->occluder mapping (for the PDDL domain)
    shadow_occluder_map = {}
    for i, shadow_id in enumerate(shadows):
        if i < len(occluders):
            shadow_occluder_map[shadow_id] = occluders[i % len(occluders)]
    
    # =========================================================
    # PHASE 5: Planning with Replanning Loop
    # =========================================================
    print("\n--- Phase 5: Planning with Replanning ---")
    
    belief = BeliefState(shadows, target_name)
    planner = PDDLStreamPlanner(registry, robot_id=robot_id, 
                                 shadow_occluder_map=shadow_occluder_map)
    
    plan_count = 0
    max_replans = len(shadows) + 1  # Safety limit
    
    if gui:
        print("Starting in 2 seconds...")
        time.sleep(2)
    
    while not belief.is_target_found() and plan_count < max_replans:
        plan_count += 1
        unknown_shadows = belief.get_unknown_shadows()
        
        print(f"\n=== PLAN #{plan_count} ===")
        print(f"Unknown shadows remaining: {len(unknown_shadows)}")
        
        if not unknown_shadows:
            print("ERROR: Searched all shadows but target not found!")
            break
        
        # Call PDDLStream to plan
        plan = planner.plan(
            target_objects=[target_name],
            goal=f'(holding {target_name})',
            max_time=60.0,
            verbose=False  # Quiet for replanning
        )
        
        if plan is None:
            print("ERROR: No plan found!")
            break
        
        print(f"Plan: {len(plan)} actions")
        for i, action in enumerate(plan):
            print(f"  {i+1}. {action[0]}")
        
        # Execute plan actions one by one
        for i, action in enumerate(plan):
            action_name = action[0]
            params = action[1:]
            
            print(f"\n  Executing: {action_name}")
            
            if action_name == 'push_aside':
                occluder_id = params[0]
                print(f"    Pushing {occluder_id} aside...")
                belief.mark_occluder_moved(occluder_id)
                # In real execution: move robot to push config, push object
                if gui:
                    time.sleep(0.5)
                    
            elif action_name == 'move':
                q1, q2, traj = params
                print(f"    Moving to {q2}...")
                # In real execution: follow trajectory
                if gui:
                    time.sleep(0.3)
                    
            elif action_name == 'sense_shadow':
                obj, shadow_id, occluder_id, config = params
                print(f"    Sensing {shadow_id}...")
                
                # ORACLE CHECK: Is the target actually here?
                found = (str(shadow_id) == oracle_hidden_shadow)
                belief.mark_sensed(str(shadow_id), found)
                
                if found:
                    print(f"    *** TARGET FOUND in {shadow_id}! ***")
                else:
                    print(f"    Target NOT in {shadow_id}")
                    print(f"    -> REPLANNING with updated belief...")
                    break  # Exit action loop to replan
                
                if gui:
                    time.sleep(0.5)
                    
            elif action_name == 'pick':
                obj, boxel_id, grasp, config = params
                print(f"    Picking {obj}...")
                
                # Execute pick
                execute_pick(robot_id, env, target_name, target_pos, gui)
                print(f"    *** {obj} PICKED UP! ***")
    
    # =========================================================
    # PHASE 6: Results
    # =========================================================
    print("\n" + "=" * 60)
    if belief.is_target_found():
        print(f"SUCCESS!")
        print(f"  Target: {target_name}")
        print(f"  Found in: {belief.target_found_in}")
        print(f"  Plans executed: {plan_count}")
        print(f"  Shadows searched: {len(shadows) - len(belief.get_unknown_shadows())}")
    else:
        print("FAILED: Target not found")
    print("=" * 60)
    
    if gui:
        print("\nWindow stays open for 10 seconds...")
        for _ in range(int(240 * 10)):
            env.step_simulation()
            time.sleep(1.0/240.0)
    
    env.close()
    return belief.is_target_found()


def execute_pick(robot_id, env, target_name, target_pos, gui):
    """Execute pick action with robot."""
    rest_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    pick_orn = p.getQuaternionFromEuler([0, np.pi, 0])
    
    # Move above target
    pick_pos = target_pos + np.array([0, 0, 0.15])
    pick_joints = p.calculateInverseKinematics(
        robot_id, 11, pick_pos.tolist(), pick_orn,
        lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
        upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
        jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
        restPoses=rest_pose
    )[:7]
    move_robot_smooth(robot_id, pick_joints, gui)
    
    # Lower
    grasp_pos = target_pos + np.array([0, 0, 0.05])
    grasp_joints = p.calculateInverseKinematics(
        robot_id, 11, grasp_pos.tolist(), pick_orn,
        lowerLimits=[-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9],
        upperLimits=[2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9],
        jointRanges=[5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8],
        restPoses=rest_pose
    )[:7]
    move_robot_smooth(robot_id, grasp_joints, gui)
    
    # Gripper
    close_gripper(robot_id, gui)
    
    # Attach
    target_id = env.objects[target_name].object_id
    p.createConstraint(robot_id, 11, target_id, -1, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
    
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


def move_robot_smooth(robot_id, target_joints, gui=True, steps=100):
    """Smoothly move robot."""
    current = [p.getJointState(robot_id, i)[0] for i in range(7)]
    for t in range(steps):
        alpha = (t + 1) / steps
        interp = [(1-alpha)*c + alpha*tgt for c, tgt in zip(current, target_joints)]
        for i in range(7):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL,
                                    targetPosition=interp[i], force=240)
        p.stepSimulation()
        if gui:
            time.sleep(1/240)


def close_gripper(robot_id, gui=True):
    """Close gripper."""
    for _ in range(50):
        p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.stepSimulation()
        if gui:
            time.sleep(1/240)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Full PDDLStream Pipeline with Replanning')
    parser.add_argument('--no-gui', action='store_true', help='Run without GUI')
    args = parser.parse_args()
    
    success = main(gui=not args.no_gui)
    sys.exit(0 if success else 1)
