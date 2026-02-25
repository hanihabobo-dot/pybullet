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
    
    def get_known_empty_shadows(self):
        """Get list of shadows we've checked and found empty."""
        return [s for s, status in self.shadow_status.items() if status == 'not_here']
    
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
    
    # Let settle (minimal)
    for _ in range(50):
        env.step_simulation()
    
    # =========================================================
    # PHASE 2: Boxel Calculation (fast, no visualization)
    # =========================================================
    print("\n--- Phase 2: Calculating Boxels ---")
    obs = env.get_camera_observation()
    all_known = obs.boxels
    free_boxels = env.generate_free_space(all_known, visualize=False)
    merged_free = merge_free_space_cells(free_boxels)
    all_boxels = all_known + merged_free
    print(f"  Calculated {len(all_boxels)} boxels")
    
    # =========================================================
    # PHASE 3: Create Registry
    # =========================================================
    print("\n--- Phase 3: Creating BoxelRegistry ---")
    registry = create_boxel_registry_from_boxels(all_boxels, env.table_surface_height)
    registry.save_to_json("boxel_data.json")
    
    shadows = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    occluders = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.OBJECT]
    print(f"  {len(registry.boxels)} boxels, {len(shadows)} shadows, {len(occluders)} occluders")
    
    # Visualize all boxels at once (after calculations complete)
    # if gui:
    #     p.resetDebugVisualizerCamera(
    #         cameraDistance=1.5, cameraYaw=45, cameraPitch=-30,
    #         cameraTargetPosition=[0.5, 0.0, env.table_surface_height]
    #     )
    #     env.draw_boxels(all_boxels, duration=0)
    
    # =========================================================
    # PHASE 4: Hidden Object Scenario (ORACLE ONLY)
    # =========================================================
    print("\n--- Phase 4: Hidden Object Scenario ---")
    
    all_targets = [name for name in env.objects.keys() if name.startswith("target")]
    
    # Determine which targets are actually inside shadow regions
    # by checking if each target's position falls within a shadow boxel's AABB.
    target_to_shadow = {}
    for tname in all_targets:
        tpos = np.array(env.objects[tname].position)
        for shadow_id in shadows:
            sb = registry.get_boxel(shadow_id)
            if sb and np.all(tpos >= sb.min_corner) and np.all(tpos <= sb.max_corner):
                target_to_shadow[tname] = shadow_id
                break
    
    if not target_to_shadow:
        print(f"  ERROR: No target is geometrically inside any shadow region.")
        print(f"  Cannot run hidden-object scenario.")
        env.close()
        return False
    
    target_name = random.choice(list(target_to_shadow.keys()))
    target_info = env.objects[target_name]
    target_pos = np.array(target_info.position)
    oracle_hidden_shadow = target_to_shadow[target_name]
    
    print(f"  Target: {target_name}")
    print(f"  ORACLE: Actually hidden in {oracle_hidden_shadow} (ground-truth AABB containment)")
    print(f"  Robot must search to find it!")
    
    # Create shadow->occluder mapping from the registry's ground-truth data.
    # Each shadow boxel knows which occluder created it (created_by_boxel_id).
    shadow_occluder_map = {}
    for shadow_id in shadows:
        shadow_boxel = registry.get_boxel(shadow_id)
        if shadow_boxel and shadow_boxel.created_by_boxel_id:
            shadow_occluder_map[shadow_id] = shadow_boxel.created_by_boxel_id
        else:
            print(f"  WARNING: Shadow {shadow_id} has no linked occluder — skipping")
    
    # Create mapping from boxel IDs to PyBullet object names and IDs
    # Boxel IDs are like "obj_000", PyBullet names are like "occluder_1"
    boxel_to_pybullet = {}
    for boxel in registry.boxels.values():
        if boxel.object_name and boxel.object_name in env.objects:
            boxel_to_pybullet[boxel.id] = {
                'name': boxel.object_name,
                'pybullet_id': env.objects[boxel.object_name].object_id,
                'position': np.array(env.objects[boxel.object_name].position)
            }
    
    print(f"  Boxel->PyBullet mapping: {len(boxel_to_pybullet)} objects")
    
    # =========================================================
    # PHASE 5: Planning with Replanning Loop
    # =========================================================
    print("\n--- Phase 5: Planning with Replanning ---")
    
    belief = BeliefState(shadows, target_name)
    planner = PDDLStreamPlanner(registry, robot_id=robot_id, 
                                 shadow_occluder_map=shadow_occluder_map)
    
    # Get boxel centers for robot motion targets
    boxel_centers = {b.id: b.center for b in registry.boxels.values()}
    
    plan_count = 0
    max_replans = len(shadows) + 1  # Safety limit
    
    while not belief.is_target_found() and plan_count < max_replans:
        plan_count += 1
        unknown_shadows = belief.get_unknown_shadows()
        known_empty = belief.get_known_empty_shadows()
        
        print(f"\n=== PLAN #{plan_count} ===")
        print(f"Unknown shadows remaining: {len(unknown_shadows)}")
        
        if not unknown_shadows:
            print("ERROR: Searched all shadows but target not found!")
            break
        
        # Call PDDLStream to plan with current belief state
        plan = planner.plan(
            target_objects=[target_name],
            goal=f'(holding {target_name})',
            known_empty_shadows=known_empty,
            moved_occluders=list(belief.occluders_moved),
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
                config_name = params[1]
                print(f"    Pushing {occluder_id} aside...")
                
                # Actually move the occluder in PyBullet
                if occluder_id in boxel_to_pybullet:
                    occ_info = boxel_to_pybullet[occluder_id]
                    occ_pybullet_id = occ_info['pybullet_id']
                    occ_pos = occ_info['position']
                    
                    # Move robot above occluder
                    move_robot_to_pos(robot_id, occ_pos + np.array([0, 0, 0.15]), gui)
                    
                    push_disp = compute_push_displacement(
                        env.camera_position, occluder_id, registry, boxel_to_pybullet
                    )
                    if push_disp is None:
                        print(f"    WARNING: No valid push direction for {occ_info['name']} "
                              f"— all directions fail bounds/collision checks. Replanning...")
                        break

                    new_pos = occ_pos + push_disp
                    p.resetBasePositionAndOrientation(
                        occ_pybullet_id, new_pos.tolist(), [0, 0, 0, 1]
                    )
                    
                    occ_info['position'] = new_pos
                    
                    for _ in range(30):
                        p.stepSimulation()
                    
                    belief.mark_occluder_moved(occluder_id)
                    
                    push_dir_xy = push_disp[:2] / (np.linalg.norm(push_disp[:2]) + 1e-8)
                    print(f"    -> Pushed {occ_info['name']} "
                          f"dir=[{push_dir_xy[0]:.2f},{push_dir_xy[1]:.2f}] "
                          f"dist={np.linalg.norm(push_disp[:2]):.3f}m "
                          f"from [{occ_pos[0]:.2f},{occ_pos[1]:.2f}] "
                          f"to [{new_pos[0]:.2f},{new_pos[1]:.2f}]")
                    
            elif action_name == 'move':
                q1, q2, dest_boxel_id, traj = params
                dest_boxel_id = str(dest_boxel_id)
                print(f"    Moving to boxel {dest_boxel_id} (config: {q2})...")
                
                # Determine base position from plan's boxel parameter.
                if dest_boxel_id in boxel_to_pybullet:
                    base_pos = boxel_to_pybullet[dest_boxel_id]['position']
                elif dest_boxel_id in boxel_centers:
                    base_pos = boxel_centers[dest_boxel_id]
                else:
                    print(f"    WARNING: Unknown boxel {dest_boxel_id}, skipping move")
                    continue
                
                # Approach offset depends on boxel type: shadow moves need
                # side clearance so the arm doesn't block camera rays;
                # occluder moves approach from the side for pushing.
                dest_boxel = registry.get_boxel(dest_boxel_id)
                if dest_boxel and dest_boxel.boxel_type == BoxelType.SHADOW:
                    offset = np.array([0, -0.3, 0.2])
                elif dest_boxel and dest_boxel.boxel_type == BoxelType.OBJECT:
                    offset = np.array([0, -0.2, 0.15])
                else:
                    offset = np.array([0, 0, 0.15])
                
                move_robot_to_pos(robot_id, base_pos + offset, gui)
                print(f"    -> Moved to {dest_boxel_id} (offset {offset.tolist()})")
                    
            elif action_name == 'sense_shadow':
                obj, shadow_id, occluder_id, config = params
                print(f"    Sensing {shadow_id}...")
                
                shadow_boxel = registry.get_boxel(str(shadow_id))
                target_pybullet_id = env.objects[target_name].object_id
                found = sense_shadow_raycasting(env.camera_position, shadow_boxel, target_pybullet_id)
                belief.mark_sensed(str(shadow_id), found)
                
                if found:
                    print(f"    *** TARGET FOUND in {shadow_id}! (ray-cast) ***")
                else:
                    print(f"    Target NOT in {shadow_id} (ray-cast: no hit)")
                    print(f"    -> REPLANNING with updated belief...")
                    break  # Exit action loop to replan
                    
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
        print("\nWindow stays open (press Ctrl+C to exit)...")
        try:
            while True:
                env.step_simulation()
        except KeyboardInterrupt:
            pass
    
    env.close()
    return belief.is_target_found()


# Robot IK parameters
REST_POSE = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
JOINT_LIMITS_LOW = [-2.9, -1.8, -2.9, -3.1, -2.9, -0.02, -2.9]
JOINT_LIMITS_HIGH = [2.9, 1.8, 2.9, -0.07, 2.9, 3.8, 2.9]
JOINT_RANGES = [5.8, 3.6, 5.8, 3.0, 5.8, 3.8, 5.8]
END_EFFECTOR_LINK = 11


def compute_ik(robot_id, target_pos, target_orn=None):
    """Compute IK solution for target position."""
    if target_orn is None:
        target_orn = p.getQuaternionFromEuler([0, np.pi, 0])  # Gripper pointing down
    
    joints = p.calculateInverseKinematics(
        robot_id, END_EFFECTOR_LINK, target_pos.tolist(), target_orn,
        lowerLimits=JOINT_LIMITS_LOW,
        upperLimits=JOINT_LIMITS_HIGH,
        jointRanges=JOINT_RANGES,
        restPoses=REST_POSE
    )[:7]
    return joints


def move_robot_to_pos(robot_id, target_pos, gui=True):
    """Move robot end-effector to target position."""
    joints = compute_ik(robot_id, target_pos)
    move_robot_smooth(robot_id, joints, gui)


def move_robot_smooth(robot_id, target_joints, gui=True, steps=60):
    """Smoothly move robot to target joint configuration."""
    import time
    current = [p.getJointState(robot_id, i)[0] for i in range(7)]
    for t in range(steps):
        alpha = (t + 1) / steps
        interp = [(1-alpha)*c + alpha*tgt for c, tgt in zip(current, target_joints)]
        for i in range(7):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL,
                                    targetPosition=interp[i], force=240)
        p.stepSimulation()
        if gui:
            time.sleep(1/120)  # Real-time visualization (120 Hz)


def close_gripper(robot_id, gui=True):
    """Close gripper."""
    import time
    for _ in range(30):
        p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, targetPosition=0.01, force=50)
        p.stepSimulation()
        if gui:
            time.sleep(1/120)


def sense_shadow_raycasting(camera_pos, shadow_boxel, target_pybullet_id):
    """
    Sense a shadow region using PyBullet ray-casting from the fixed camera.

    Once the occluder has been physically pushed aside, rays from the fixed
    scene camera can penetrate the shadow region to detect hidden objects.
    Casts rays into a grid of points spanning the shadow boxel's XY footprint
    at multiple Z levels.

    Args:
        camera_pos: Fixed camera position [x, y, z]
        shadow_boxel: BoxelData for the shadow region to sense
        target_pybullet_id: PyBullet body ID of the target object

    Returns:
        True if the target was detected in the shadow region
    """
    ray_origin = np.array(camera_pos)

    min_c = shadow_boxel.min_corner
    max_c = shadow_boxel.max_corner

    z_levels = [
        min_c[2] + 0.04,
        min_c[2] + (max_c[2] - min_c[2]) * 0.33,
        min_c[2] + (max_c[2] - min_c[2]) * 0.67,
    ]

    n = 7
    ray_froms = []
    ray_tos = []
    for z_target in z_levels:
        for xi in np.linspace(min_c[0], max_c[0], n):
            for yi in np.linspace(min_c[1], max_c[1], n):
                ray_froms.append(ray_origin.tolist())
                ray_tos.append([float(xi), float(yi), float(z_target)])

    results = p.rayTestBatch(ray_froms, ray_tos)
    for hit_obj_id, _link, _frac, _pos, _normal in results:
        if hit_obj_id == target_pybullet_id:
            return True

    return False


def execute_pick(robot_id, env, target_name, target_pos, gui):
    """Execute pick action with robot."""
    # Move above target
    move_robot_to_pos(robot_id, target_pos + np.array([0, 0, 0.15]), gui)
    
    # Lower to grasp
    move_robot_to_pos(robot_id, target_pos + np.array([0, 0, 0.05]), gui)
    
    # Close gripper
    close_gripper(robot_id, gui)
    
    # Attach object
    target_id = env.objects[target_name].object_id
    p.createConstraint(robot_id, END_EFFECTOR_LINK, target_id, -1, 
                       p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
    
    # Lift
    move_robot_to_pos(robot_id, target_pos + np.array([0, 0, 0.3]), gui)


def compute_push_displacement(camera_pos, occluder_id, registry, boxel_to_pybullet):
    """
    Compute the displacement vector to push an occluder out of the camera's
    line of sight to its shadow region(s).

    The push direction is perpendicular to the camera-to-shadow viewing line
    (in the XY plane). The distance ensures the occluder's AABB clears the
    viewing corridor. Between the two perpendicular candidates, the one that
    avoids collisions with other objects and stays within table bounds is chosen.

    Args:
        camera_pos: Camera position [x, y, z]
        occluder_id: Boxel ID of the occluder to push
        registry: BoxelRegistry with scene geometry
        boxel_to_pybullet: Dict mapping boxel IDs to PyBullet info

    Returns:
        np.ndarray or None: Push displacement vector [dx, dy, 0], or None if
        no valid direction exists (both perpendiculars fail bounds/collision).
    """
    occluder_boxel = registry.get_boxel(occluder_id)
    if occluder_boxel is None:
        print(f"    WARNING: compute_push_displacement — occluder '{occluder_id}' not found in registry")
        return None

    occ_extent = occluder_boxel.extent

    shadow_boxels = [registry.get_boxel(sid)
                     for sid in occluder_boxel.shadow_boxel_ids
                     if registry.get_boxel(sid) is not None]

    if not shadow_boxels:
        print(f"    WARNING: compute_push_displacement — no shadow boxels linked to occluder '{occluder_id}'")
        return None

    shadow_center = np.mean([sb.center for sb in shadow_boxels], axis=0)

    cam_to_shadow_xy = np.array([
        shadow_center[0] - camera_pos[0],
        shadow_center[1] - camera_pos[1]
    ])
    view_len = np.linalg.norm(cam_to_shadow_xy)
    if view_len < 1e-6:
        print(f"    WARNING: compute_push_displacement — degenerate camera-to-shadow direction for '{occluder_id}'")
        return None
    cam_to_shadow_xy /= view_len

    perp = np.array([-cam_to_shadow_xy[1], cam_to_shadow_xy[0]])

    half_width = abs(occ_extent[0] * perp[0]) + abs(occ_extent[1] * perp[1])
    push_dist = half_width + 0.10

    TABLE_X_MIN, TABLE_X_MAX = 0.0, 1.0
    TABLE_Y_MIN, TABLE_Y_MAX = -0.5, 0.5

    occ_pos_xy = occluder_boxel.center[:2]
    if occluder_id in boxel_to_pybullet:
        occ_pos_xy = np.array(boxel_to_pybullet[occluder_id]['position'][:2])

    for sign in [1.0, -1.0]:
        direction = sign * perp
        new_xy = occ_pos_xy + direction * push_dist

        if (new_xy[0] - occ_extent[0] < TABLE_X_MIN or
                new_xy[0] + occ_extent[0] > TABLE_X_MAX or
                new_xy[1] - occ_extent[1] < TABLE_Y_MIN or
                new_xy[1] + occ_extent[1] > TABLE_Y_MAX):
            continue

        collision = False
        for bid, binfo in boxel_to_pybullet.items():
            if bid == occluder_id:
                continue
            other_boxel = registry.get_boxel(bid)
            if other_boxel is None:
                continue
            other_pos = np.array(binfo['position'][:2])
            other_ext = other_boxel.extent
            if (abs(new_xy[0] - other_pos[0]) < occ_extent[0] + other_ext[0] and
                    abs(new_xy[1] - other_pos[1]) < occ_extent[1] + other_ext[1]):
                collision = True
                break

        if not collision:
            return np.array([direction[0] * push_dist, direction[1] * push_dist, 0.0])

    return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Full PDDLStream Pipeline with Replanning')
    parser.add_argument('--no-gui', action='store_true', help='Run without GUI')
    args = parser.parse_args()
    
    success = main(gui=not args.no_gui)
    sys.exit(0 if success else 1)
