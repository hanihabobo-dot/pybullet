#!/usr/bin/env python3
"""
Demo script for the Boxel Test Environment.

This script demonstrates the Semantic Boxel system by:
1. Creating a PyBullet environment with objects
2. Detecting visible objects using oracle perception
3. Generating boxels for objects and their occlusion shadows
4. Discretizing free space using octree subdivision
5. Visualizing all boxels in the GUI
"""

import numpy as np
import pybullet as p
import time

from boxel_env import BoxelTestEnv
from visualization import save_point_cloud_to_ply


def main():
    """Main demo function."""
    print("=" * 60)
    print("Boxel Test Environment - Demo")
    print("=" * 60)
    
    # Create environment
    env = BoxelTestEnv(gui=True)
    
    # Let objects settle
    print("\nLetting objects settle...")
    for _ in range(100):
        env.step_simulation()
        time.sleep(1.0 / 240.0)
    
    # Get camera observation
    print("\nCapturing camera observation...")
    obs = env.get_camera_observation()
    
    print(f"\nCamera Observation Results:")
    print(f"  RGB image shape: {obs.rgb_image.shape}")
    print(f"  Depth image shape: {obs.depth_image.shape}")
    print(f"  Point cloud shape: {obs.point_cloud.shape}")
    print(f"  Visible objects: {obs.visible_objects}")
    print(f"  Total objects detected: {len(obs.object_poses)}")
    print(f"  Boxels generated: {len(obs.boxels) if obs.boxels else 0}")
    
    # Save point clouds
    save_point_cloud_to_ply(obs.point_cloud, "scene_point_cloud.ply")
    
    # Test oracle detection
    print("\nOracle Detection Results:")
    visible, poses = env.oracle_detect_objects(check_occlusion=True)
    print(f"  Visible objects: {visible}")
    print(f"  Total objects: {list(poses.keys())}")
    
    for obj_name in visible:
        pos, orn = poses[obj_name]
        print(f"    {obj_name}: position={pos}, orientation={orn}")
    
    # Test point cloud generation
    print("\nTesting point cloud generation...")
    target_pc = env.get_object_point_cloud("target_1")
    if target_pc is not None:
        print(f"  Generated point cloud for target_1: {target_pc.shape} points")
        save_point_cloud_to_ply(target_pc, "target_1_point_cloud.ply")
    
    # Run visualization
    print("\nRunning visualization sequence...")
    print("(Close the PyBullet window or press Ctrl+C to exit)")
    
    try:
        all_known = obs.boxels
        obj_boxels = [b for b in all_known if not b.is_shadow]
        
        # Debug output
        print("\n=== DEBUG: All Generated Boxels ===")
        for i, b in enumerate(all_known):
            boxel_type = "SHADOW" if b.is_shadow else "OBJECT"
            print(f"  [{i}] {boxel_type}: {b.object_name}")
            print(f"       center: {b.center}")
            print(f"       extent: {b.extent}")
            if np.any(b.extent < 0.01):
                print(f"       *** WARNING: Very thin boxel! ***")
            if np.any(b.extent > 1.0):
                print(f"       *** WARNING: Very large extent! ***")
        print("=== END DEBUG ===\n")
        
        # Timing constants
        PHASE_WAIT_SECONDS = 0  # Set to 0 for fast, 1.0 for normal
        BOXEL_FILL_OPACITY = 0.05 # Opacity for filled boxels (0.0 = invisible, 1.0 = solid)
        FINAL_HOLD_SECONDS = 600  # Increased for keyboard navigation
        ENABLE_FREE_SPACE = True
        
        # Set camera view using the new method
        env.set_debug_camera(distance=1.5, yaw=45, pitch=-30,
                            target=np.array([0.5, 0.0, env.table_surface_height]))
        
        print("\n=== KEYBOARD CONTROLS ===")
        print("  W/Up Arrow:    Move forward")
        print("  S/Down Arrow:  Move backward")
        print("  A/Left Arrow:  Move left")
        print("  D/Right Arrow: Move right")
        print("  Q:             Move up")
        print("  E:             Move down")
        print("  R:             Zoom in")
        print("  F:             Zoom out")
        print("  Mouse:         Ctrl+Left click to rotate")
        print("=========================\n")
        
        # Phase 1: Show Objects
        print("Phase 1: Objects")
        env.draw_boxels(obj_boxels, duration=0, fill_opacity=BOXEL_FILL_OPACITY)
        for _ in range(int(240 * PHASE_WAIT_SECONDS)): 
            env.step_simulation()
            env.handle_keyboard_camera()
            time.sleep(1.0/240.0)
            
        # Phase 2: Show Shadows
        print("Phase 2: Shadows")
        env.draw_boxels(all_known, duration=0, fill_opacity=BOXEL_FILL_OPACITY)
        for _ in range(int(240 * PHASE_WAIT_SECONDS)):
            env.step_simulation()
            env.handle_keyboard_camera()
            time.sleep(1.0/240.0)
            
        # Phase 3: Generate Free Space
        print("Phase 3: Free Space Discretization")
        env.clear_all_debug_items()
        env.draw_boxels(all_known, duration=0, clear_previous=True, fill_opacity=BOXEL_FILL_OPACITY)
        if ENABLE_FREE_SPACE:
            free_boxels = env.generate_free_space(all_known, visualize=False)
            print(f"  Generated {len(free_boxels)} free space boxels")
            env.draw_boxels(all_known + free_boxels, duration=0, clear_previous=True, fill_opacity=BOXEL_FILL_OPACITY)
        
        # Phase 3.5: Merge Free Space Cells
        print("Phase 3.5: Merging Free Space Cells")
        if ENABLE_FREE_SPACE and free_boxels:
            merged_free_boxels = env.merge_free_space(free_boxels)
            print(f"  Merged: {len(free_boxels)} -> {len(merged_free_boxels)} boxels")
            env.draw_boxels(all_known + merged_free_boxels, duration=0, clear_previous=True, fill_opacity=BOXEL_FILL_OPACITY)
            free_boxels = merged_free_boxels  # Use merged boxels going forward
        
        # Phase 4: Hold Result
        print("Phase 4: Hold Result")
        for _ in range(int(240 * PHASE_WAIT_SECONDS * 3)):
            env.step_simulation()
            env.handle_keyboard_camera()
            time.sleep(1.0/240.0)
            
        # Phase 5: Keep window open with keyboard navigation
        print(f"Visualization complete. Use WASD/Arrow keys to navigate. Window stays open for {FINAL_HOLD_SECONDS} seconds...")
        for _ in range(int(240 * FINAL_HOLD_SECONDS)):
            env.step_simulation()
            env.handle_keyboard_camera()
            time.sleep(1.0/240.0)
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    # Cleanup
    env.close()
    print("\nEnvironment closed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()
