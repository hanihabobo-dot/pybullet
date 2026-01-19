#!/usr/bin/env python3
"""Test PyBullet robot control in executor."""

import pybullet as p
import pybullet_data
import numpy as np
from executor import BoxelExecutor
from boxel_data import BoxelRegistry, BoxelType
from streams import BoxelStreams


def main():
    # Setup PyBullet
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    robot_id = p.loadURDF('franka_panda/panda.urdf', [0, 0, 0], useFixedBase=True)
    
    # Set rest pose
    rest_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    for i in range(7):
        p.resetJointState(robot_id, i, rest_pose[i])
    
    # Load registry
    registry = BoxelRegistry.load_from_json('boxel_data.json')
    shadows = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    print(f"Found {len(shadows)} shadows")
    
    # Generate sensing config
    streams = BoxelStreams(registry, robot_id=robot_id, physics_client=physics_client)
    configs = list(streams.sample_sensing_config(shadows[0]))
    print(f"Generated {len(configs)} configs for {shadows[0]}")
    
    if not configs:
        print("ERROR: No configs generated")
        return
    
    sensing_config = configs[0][0]
    print(f"Using config: {sensing_config.name}")
    
    # Create executor
    config_lookup = {
        'q_home': np.array(rest_pose),
        sensing_config.name: sensing_config.joint_positions
    }
    
    executor = BoxelExecutor(
        registry=registry,
        oracle_locations={'target_1': shadows[-1]},
        robot_id=robot_id,
        physics_client=physics_client,
        config_lookup=config_lookup,
        execute_real=True,
        verbose=True
    )
    
    # Test motion
    print("\nMoving robot...")
    executor._move_robot_to_config(sensing_config.joint_positions)
    
    # Check result
    final = np.array([p.getJointState(robot_id, i)[0] for i in range(7)])
    error = np.max(np.abs(final - sensing_config.joint_positions))
    print(f"Max joint error: {error:.4f} rad")
    print(f"Result: {'SUCCESS' if error < 0.1 else 'FAILED'}")
    
    p.disconnect()


if __name__ == "__main__":
    main()
