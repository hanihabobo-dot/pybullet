#!/usr/bin/env python3
"""
Full Pipeline Test: PDDLStream Planning + PyBullet Execution

Run from WSL:
    source wsl_env/bin/activate
    export PYTHONPATH=/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib
    python3 test_full_pipeline.py
"""

import sys
import os

# Add pddlstream to path (for WSL)
PDDLSTREAM_PATH = '/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib'
if os.path.exists(PDDLSTREAM_PATH):
    sys.path.insert(0, PDDLSTREAM_PATH)

import numpy as np
import pybullet as p
import pybullet_data

from boxel_data import BoxelRegistry, BoxelType
from streams import BoxelStreams
from executor import BoxelExecutor
from pddlstream_planner import PDDLStreamPlanner


def setup_pybullet():
    """Initialize PyBullet environment."""
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    
    robot_id = p.loadURDF('franka_panda/panda.urdf', [0, 0, 0], useFixedBase=True)
    
    # Set rest pose
    rest_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    for i in range(7):
        p.resetJointState(robot_id, i, rest_pose[i])
    
    return physics_client, robot_id


def main():
    print("="*60)
    print("FULL PIPELINE TEST")
    print("PDDLStream Planning + PyBullet Execution")
    print("="*60)
    
    # Setup PyBullet
    print("\n1. Setting up PyBullet...")
    physics_client, robot_id = setup_pybullet()
    print(f"   Robot loaded: ID={robot_id}")
    
    # Load boxel registry
    print("\n2. Loading boxel registry...")
    registry = BoxelRegistry.load_from_json('boxel_data.json')
    shadows = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    print(f"   Loaded {len(registry.boxels)} boxels, {len(shadows)} shadows")
    
    # Create streams with real IK
    print("\n3. Creating streams with real IK...")
    streams = BoxelStreams(registry, robot_id=robot_id, physics_client=physics_client)
    
    # Pre-generate configs for planner
    print("\n4. Pre-generating robot configurations...")
    config_lookup = {
        'q_home': np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
    }
    
    for shadow_id in shadows:
        configs = list(streams.sample_sensing_config(shadow_id))
        for cfg, in configs[:2]:  # Keep first 2 configs per shadow
            config_lookup[cfg.name] = cfg.joint_positions
    print(f"   Generated {len(config_lookup)} configurations")
    
    # Plan using PDDLStream
    print("\n5. Planning with PDDLStream...")
    planner = PDDLStreamPlanner(registry, robot_id=robot_id)
    
    plan = planner.plan(
        target_objects=['target_1'],
        goal='(holding target_1)',
        max_time=60.0,
        verbose=False
    )
    
    if plan is None:
        print("   ERROR: No plan found!")
        p.disconnect()
        return False
    
    print(f"   Plan found with {len(plan)} actions:")
    for i, action in enumerate(plan):
        print(f"      {i+1}. {action[0]} {' '.join(action[1:])}")
    
    # Update config_lookup with any configs from the plan
    for action in plan:
        for param in action[1:]:
            if param.startswith('q_') and param not in config_lookup:
                # Generate this config
                if 'sense' in param:
                    shadow_id = param.replace('q_sense_', '').rsplit('_', 1)[0]
                    configs = list(streams.sample_sensing_config(shadow_id))
                    if configs:
                        config_lookup[param] = configs[0][0].joint_positions
                elif 'pick' in param:
                    # Use a reasonable pick config
                    config_lookup[param] = np.array([0.5, -0.5, 0, -2.0, 0, 1.5, 0.785])
    
    # Execute with real robot control
    print("\n6. Executing plan with PyBullet robot...")
    
    # Oracle: target is in first shadow (optimistic case)
    oracle_locations = {'target_1': shadows[0]}
    
    executor = BoxelExecutor(
        registry=registry,
        oracle_locations=oracle_locations,
        robot_id=robot_id,
        physics_client=physics_client,
        config_lookup=config_lookup,
        execute_real=True,
        verbose=True
    )
    
    # Execute each action
    from executor import ActionResult, ExecutionState
    from problem_generator import ProblemGenerator
    
    problem = ProblemGenerator(registry).generate_problem(
        target_objects=['target_1'],
        goal='(holding target_1)'
    )
    
    state = ExecutionState(
        problem=problem,
        current_config='q_home'
    )
    
    success = True
    for action in plan:
        result = executor._execute_action(action, state, ['target_1'])
        if result == ActionResult.FAILURE:
            success = False
            break
        elif result == ActionResult.SENSING_MISMATCH:
            print("   Sensing mismatch - would replan in full system")
            # Continue anyway for this test
    
    # Check final robot state
    print("\n7. Checking final state...")
    final_joints = np.array([p.getJointState(robot_id, i)[0] for i in range(7)])
    print(f"   Final joints: {np.round(final_joints, 2)}")
    print(f"   Holding: {state.holding}")
    
    p.disconnect()
    
    print("\n" + "="*60)
    if success and state.holding == 'target_1':
        print("FULL PIPELINE TEST: SUCCESS!")
    else:
        print("FULL PIPELINE TEST: Completed with issues")
    print("="*60)
    
    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
