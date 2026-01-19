#!/usr/bin/env python3
"""
Hidden Object Scenario - Full TAMP Demo with Partial Observability.

This script demonstrates the complete Semantic Boxel TAMP system:
1. Creates a PyBullet scene with occluding objects
2. Generates boxels (objects, shadows, free space)
3. Runs the executor with optimistic planning + replanning
4. Robot senses shadow boxels until target is found
5. Robot picks up the target object

The scenario showcases:
- Know-If fluents for partial observability
- Optimistic planning (assume sensing will succeed)
- Replanning when sensing reveals object is not in expected location
- Belief propagation (finding object eliminates other possibilities)
"""

import numpy as np
import time
import argparse
import random
from typing import List, Optional

from boxel_env import BoxelTestEnv
from boxel_data import BoxelRegistry, BoxelType, create_boxel_registry_from_boxels
from executor import BoxelExecutor


def setup_scene(gui: bool = True, settle_time: float = 0.5) -> tuple:
    """
    Set up the PyBullet scene and generate boxels.
    
    Args:
        gui: Whether to show the PyBullet GUI
        settle_time: Time in seconds to let objects settle
        
    Returns:
        Tuple of (env, registry, shadow_boxel_ids)
    """
    print("\n" + "="*70)
    print("SCENE SETUP")
    print("="*70)
    
    # Create environment
    env = BoxelTestEnv(gui=gui)
    
    # Let objects settle
    print("\nLetting objects settle...")
    steps = int(240 * settle_time)
    for _ in range(steps):
        env.step_simulation()
        if gui:
            time.sleep(1.0 / 240.0)
    
    # Get camera observation (generates object and shadow boxels)
    print("Capturing camera observation and generating boxels...")
    obs = env.get_camera_observation()
    
    print(f"\nObservation Results:")
    print(f"  Visible objects: {obs.visible_objects}")
    print(f"  Boxels generated: {len(obs.boxels) if obs.boxels else 0}")
    
    # Generate free space
    print("\nGenerating free space...")
    all_known = obs.boxels
    free_boxels = env.generate_free_space(all_known, visualize=False)
    print(f"  Free space cells: {len(free_boxels)}")
    
    # Merge free space
    merged_free = env.merge_free_space(free_boxels)
    print(f"  After merging: {len(merged_free)}")
    
    # Create registry
    all_boxels = all_known + merged_free
    registry = create_boxel_registry_from_boxels(all_boxels, env.table_surface_height)
    
    # Find shadow boxels
    shadow_ids = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    
    print(f"\nFinal boxel counts:")
    print(f"  Objects: {len([b for b in registry.boxels.values() if b.boxel_type == BoxelType.OBJECT])}")
    print(f"  Shadows: {len(shadow_ids)}")
    print(f"  Free space: {len([b for b in registry.boxels.values() if b.boxel_type == BoxelType.FREE_SPACE])}")
    
    return env, registry, shadow_ids


def run_scenario(env: BoxelTestEnv,
                 registry: BoxelRegistry, 
                 shadow_ids: List[str],
                 target_shadow: Optional[str] = None,
                 verbose: bool = True) -> bool:
    """
    Run the hidden object search scenario.
    
    Args:
        env: PyBullet environment
        registry: BoxelRegistry with scene boxels
        shadow_ids: List of shadow boxel IDs
        target_shadow: Which shadow contains the target (None = random)
        verbose: Print detailed output
        
    Returns:
        True if target was found and picked
    """
    print("\n" + "="*70)
    print("HIDDEN OBJECT SCENARIO")
    print("="*70)
    
    if not shadow_ids:
        print("ERROR: No shadow boxels found! Cannot hide object.")
        return False
    
    # Choose where to hide the target
    if target_shadow is None:
        target_shadow = random.choice(shadow_ids)
    elif target_shadow not in shadow_ids:
        print(f"WARNING: {target_shadow} not in shadow list. Using random.")
        target_shadow = random.choice(shadow_ids)
    
    print(f"\n{'='*50}")
    print(f"GROUND TRUTH: target_1 is hidden in {target_shadow}")
    print(f"{'='*50}")
    print(f"\nAvailable shadows to search: {shadow_ids}")
    print(f"Robot must sense shadows to find the target.\n")
    
    # Oracle knows the ground truth
    oracle_locations = {
        "target_1": target_shadow
    }
    
    # Create executor
    executor = BoxelExecutor(
        registry=registry,
        oracle_locations=oracle_locations,
        verbose=verbose
    )
    
    # Run execution
    success = executor.execute(
        target_objects=["target_1"],
        goal="(holding target_1)",
        max_replans=len(shadow_ids) + 2  # At most, search all shadows + buffer
    )
    
    return success


def visualize_result(env: BoxelTestEnv, 
                     registry: BoxelRegistry,
                     hold_seconds: float = 10.0):
    """
    Visualize the final scene with all boxels.
    
    Args:
        env: PyBullet environment
        registry: BoxelRegistry with scene boxels
        hold_seconds: Time to keep visualization open
    """
    print("\n" + "="*70)
    print("VISUALIZATION")
    print("="*70)
    
    # Convert registry boxels to visualization format
    from boxel_types import Boxel as VisBxl
    vis_boxels = []
    for b in registry.boxels.values():
        vis_boxel = VisBxl(
            center=np.array(b.center),
            extent=np.array(b.extent),
            object_name=b.id,
            color=b.color,
            is_shadow=(b.boxel_type == BoxelType.SHADOW)
        )
        vis_boxels.append(vis_boxel)
    
    # Draw boxels
    env.clear_all_debug_items()
    env.draw_boxels(vis_boxels, duration=0, fill_opacity=0.1)
    
    # Set nice camera angle
    env.set_debug_camera(distance=1.5, yaw=45, pitch=-30,
                        target=np.array([0.5, 0.0, env.table_surface_height]))
    
    print(f"\nVisualization active for {hold_seconds} seconds...")
    print("  Use WASD to navigate")
    print("  Press Ctrl+C to exit early")
    
    try:
        for _ in range(int(240 * hold_seconds)):
            env.step_simulation()
            env.handle_keyboard_camera()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        print("\nVisualization interrupted.")


def main():
    """Main entry point for hidden object scenario."""
    parser = argparse.ArgumentParser(
        description="Hidden Object TAMP Scenario with Semantic Boxels"
    )
    parser.add_argument(
        "--no-gui", 
        action="store_true",
        help="Run without PyBullet visualization"
    )
    parser.add_argument(
        "--target-shadow",
        type=str,
        default=None,
        help="Specify which shadow contains the target (e.g., 'shadow_001')"
    )
    parser.add_argument(
        "--visualize",
        type=float,
        default=5.0,
        help="Seconds to show visualization (0 = skip)"
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Reduce output verbosity"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility"
    )
    
    args = parser.parse_args()
    
    # Set random seed if specified
    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)
    
    print("\n" + "#"*70)
    print("#" + " "*20 + "SEMANTIC BOXEL TAMP DEMO" + " "*20 + "#")
    print("#" + " "*10 + "Hidden Object Search with Partial Observability" + " "*10 + "#")
    print("#"*70)
    
    # Setup scene
    gui = not args.no_gui
    env, registry, shadow_ids = setup_scene(gui=gui)
    
    try:
        # Run scenario
        success = run_scenario(
            env=env,
            registry=registry,
            shadow_ids=shadow_ids,
            target_shadow=args.target_shadow,
            verbose=not args.quiet
        )
        
        # Print final result
        print("\n" + "="*70)
        if success:
            print("SCENARIO COMPLETE: Target successfully found and picked!")
        else:
            print("SCENARIO FAILED: Could not find or pick target.")
        print("="*70)
        
        # Visualize if requested
        if gui and args.visualize > 0:
            visualize_result(env, registry, args.visualize)
            
    finally:
        env.close()
        print("\nEnvironment closed.")
    
    return 0 if success else 1


def quick_test():
    """
    Quick test without GUI - tests the REPLANNING loop.
    
    Hides target in the LAST shadow to force multiple sensing failures
    and replanning cycles before finding the object.
    
    Returns:
        True if test passed
    """
    print("\n=== QUICK TEST (No GUI) ===")
    print("Testing: Replanning loop with target in LAST shadow\n")
    
    try:
        env, registry, shadow_ids = setup_scene(gui=False, settle_time=0.2)
        
        if not shadow_ids:
            print("ERROR: No shadows generated")
            env.close()
            return False
        
        if len(shadow_ids) < 2:
            print("WARNING: Only 1 shadow - cannot test replanning")
        
        # Hide target in LAST shadow to force replanning!
        target_shadow = shadow_ids[-1]
        print(f"Hiding target in shadow {len(shadow_ids)-1} of {len(shadow_ids)}: {target_shadow}")
        print(f"This will require {len(shadow_ids)-1} replan(s) before finding target.\n")
        
        success = run_scenario(
            env=env,
            registry=registry,
            shadow_ids=shadow_ids,
            target_shadow=target_shadow,  # Use LAST shadow
            verbose=True
        )
        
        env.close()
        
        print(f"\n=== QUICK TEST {'PASSED' if success else 'FAILED'} ===")
        return success
        
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    import sys
    
    # Check for quick test mode
    if len(sys.argv) > 1 and sys.argv[1] == "--quick":
        success = quick_test()
        sys.exit(0 if success else 1)
    
    # Run full scenario
    sys.exit(main())
