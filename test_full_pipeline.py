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

PDDLStream path is added to sys.path via the hardcoded PDDLSTREAM_PATH constant below.
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

from boxel_env import BoxelTestEnv
from boxel_data import BoxelRegistry, BoxelType, create_boxel_registry_from_boxels
from cell_merger import merge_free_space_cells
from pddlstream_planner import PDDLStreamPlanner
from streams import RobotConfig
from robot_utils import (END_EFFECTOR_LINK, solve_ik,
                         move_robot_smooth, open_gripper, close_gripper)
from run_logger import RunLogger


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
        self.occluders_moved = {}  # {occluder_id: destination_boxel_id}
    
    def mark_sensed(self, shadow_id: str, found: bool):
        """Update belief after sensing a shadow."""
        if found:
            self.shadow_status[shadow_id] = 'found'
            self.target_found_in = shadow_id
        else:
            self.shadow_status[shadow_id] = 'not_here'
    
    def mark_occluder_moved(self, occluder_id: str, destination: str):
        """
        Mark that an occluder has been pushed to a new location.

        Args:
            occluder_id: Boxel ID of the occluder that was pushed
            destination: Symbolic boxel ID for the push destination (used
                by the planner to emit obj_at_boxel for the new location)
        """
        self.occluders_moved[occluder_id] = destination

    def get_unknown_shadows(self):
        """Get list of shadows we haven't checked yet."""
        return [s for s, status in self.shadow_status.items() if status == 'unknown']
    
    def get_known_empty_shadows(self):
        """Get list of shadows we've checked and found empty."""
        return [s for s, status in self.shadow_status.items() if status == 'not_here']
    
    def is_target_found(self):
        """Check if we've found the target."""
        return self.target_found_in is not None


def main(gui=True, run_logger=None):
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
    
    # Let settle: 50 steps at 240 Hz ≈ 0.2 s.  Enough for the loaded
    # Panda + cubes to reach static equilibrium after spawning.
    for _ in range(50):
        env.step_simulation()
    env.update_object_positions()
    
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
    if run_logger:
        run_logger.save_artefact("boxel_data.json")
    
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
    
    # Build bidirectional name→body-ID mapping so streams can exclude the
    # grasped object from collision checks during compute_kin / plan_motion.
    object_body_ids = {}
    for name, obj_info in env.objects.items():
        if name not in ("plane", "table", "robot"):
            object_body_ids[name] = obj_info.object_id
    for boxel in registry.boxels.values():
        if boxel.object_name and boxel.object_name in object_body_ids:
            object_body_ids[boxel.id] = object_body_ids[boxel.object_name]

    support_body_ids = frozenset({
        env.objects["plane"].object_id,
        env.objects["table"].object_id,
    })

    belief = BeliefState(shadows, target_name)
    planner = PDDLStreamPlanner(registry, robot_id=robot_id,
                                 shadow_occluder_map=shadow_occluder_map,
                                 physics_client=env.client_id,
                                 object_body_ids=object_body_ids,
                                 support_body_ids=support_body_ids)
    
    problem_path = planner.export_problem_pddl(
        target_objects=[target_name],
        goal=('holding', target_name)
    )
    print(f"  Exported initial problem to {problem_path}")
    if run_logger:
        run_logger.save_artefact(problem_path, "problem_initial.pddl")
    
    # Get boxel centers for robot motion targets
    boxel_centers = {b.id: b.center for b in registry.boxels.values()}
    
    plan_count = 0
    # Reactive replanning loop (see CODEBASE_AUDIT #61, PA-5):
    # The PDDL sense action is optimistic — it assumes the target will be
    # found.  When sensing reveals "not found" or "still blocked," the
    # execution loop breaks out and replans with updated belief state
    # (known_empty_shadows).  Each replan eliminates one shadow candidate,
    # so worst-case N replans for N shadows.  Budget allows multiple
    # occluder relocations per shadow (4 attempts * shadows + 1 final pick).
    max_replans = 4 * len(shadows) + 1
    grasp_constraint_id = None
    exit_reason = None
    current_config = planner.home_config
    
    while not belief.is_target_found() and plan_count < max_replans:
        plan_count += 1
        unknown_shadows = belief.get_unknown_shadows()
        known_empty = belief.get_known_empty_shadows()
        
        print(f"\n=== PLAN #{plan_count} ===")
        print(f"Unknown shadows remaining: {len(unknown_shadows)}")
        
        if not unknown_shadows:
            exit_reason = "all_searched"
            print("ERROR: Searched all shadows but target not found!")
            break
        
        # Call PDDLStream to plan with current belief state
        plan = planner.plan(
            target_objects=[target_name],
            goal=('holding', target_name),
            current_config=current_config,
            known_empty_shadows=known_empty,
            moved_occluders=dict(belief.occluders_moved),
            max_time=120.0,
            verbose=False  # Quiet for replanning
        )
        
        if plan is None:
            exit_reason = "planner_failed"
            print("ERROR: No plan found!")
            break
        
        print(f"Plan: {len(plan)} actions")
        for i, action in enumerate(plan):
            print(f"  {i+1}. {action[0]}")
        
        # Reject plans containing heuristic (kinematically invalid) configs
        for action in plan:
            for param in action[1:]:
                if isinstance(param, RobotConfig) and param.is_heuristic:
                    raise RuntimeError(
                        f"Plan contains heuristic config '{param.name}' — "
                        f"cannot execute kinematically invalid configurations. "
                        f"Ensure BoxelStreams has a valid robot_id."
                    )

        # Execute plan actions one by one
        for i, action in enumerate(plan):
            action_name = action[0]
            params = action[1:]
            
            print(f"\n  Executing: {action_name}")
            
            if action_name == 'move':
                q1, q2, dest_boxel_id, traj = params
                print(f"    Moving to {dest_boxel_id} ({len(traj.waypoints)} waypoints)...")

                for wp in traj.waypoints[1:]:
                    move_robot_smooth(robot_id, wp.joint_positions, gui,
                                      steps=30)
                current_config = q2
                print(f"    -> Arrived at {dest_boxel_id}")
                    
            elif action_name == 'sense':
                obj, shadow_id = params
                print(f"    Sensing {shadow_id} (fixed camera)...")
                
                shadow_boxel = registry.get_boxel(str(shadow_id))
                if shadow_boxel is None:
                    print(f"    WARNING: Shadow '{shadow_id}' not found in registry. Replanning...")
                    break

                target_pybullet_id = env.objects[target_name].object_id
                occluder_pybullet_id = None
                if shadow_boxel.created_by_boxel_id in boxel_to_pybullet:
                    occluder_pybullet_id = boxel_to_pybullet[shadow_boxel.created_by_boxel_id]['pybullet_id']
                shadow_occluder_id = shadow_boxel.created_by_boxel_id

                sense_outcome, blocked_fraction = sense_shadow_raycasting(
                    env.camera_position,
                    shadow_boxel,
                    target_pybullet_id,
                    occluder_pybullet_id
                )

                if sense_outcome == "found_target":
                    belief.mark_sensed(str(shadow_id), found=True)
                    print(f"    *** TARGET FOUND in {shadow_id}! (ray-cast) ***")
                elif sense_outcome == "clear_but_empty":
                    belief.mark_sensed(str(shadow_id), found=False)
                    print(f"    Target NOT in {shadow_id} (ray-cast: view clear but no target hit)")
                    print(f"    -> REPLANNING with updated belief...")
                    break  # Exit action loop to replan
                else:
                    print(f"    View to {shadow_id} still blocked "
                          f"({blocked_fraction:.0%} rays hit occluder).")
                    print(f"    -> REPLANNING without marking shadow empty...")
                    break  # Exit action loop to replan
                    
            elif action_name == 'pick':
                obj, boxel_id, grasp, config = params
                obj_str = str(obj)
                print(f"    Picking {obj_str} from {boxel_id}...")

                if obj_str in boxel_to_pybullet:
                    pick_obj_name = boxel_to_pybullet[obj_str]['name']
                    pick_pos = np.array(env.objects[pick_obj_name].position)
                elif obj_str == target_name:
                    pick_obj_name = target_name
                    pick_pos = np.array(env.objects[target_name].position)
                else:
                    print(f"    ERROR: Cannot resolve PyBullet object for '{obj_str}'")
                    break

                grasp_constraint_id, current_config = execute_pick(
                    robot_id, env, pick_obj_name, pick_pos,
                    grasp, config, gui)
                print(f"    *** {pick_obj_name} PICKED UP! ***")

            elif action_name == 'place':
                obj, boxel_id, grasp, config = params
                obj_str = str(obj)
                boxel_id_str = str(boxel_id)
                print(f"    Placing {obj_str} at {boxel_id_str}...")

                if boxel_id_str in boxel_centers:
                    place_pos = boxel_centers[boxel_id_str]
                elif boxel_id_str in boxel_to_pybullet:
                    place_pos = boxel_to_pybullet[boxel_id_str]['position']
                else:
                    print(f"    ERROR: Cannot resolve position for boxel '{boxel_id_str}'")
                    break

                current_config = execute_place(
                    robot_id, env, obj_str, place_pos, grasp, config,
                    grasp_constraint_id, gui)
                grasp_constraint_id = None

                env.update_object_positions()
                for bid, binfo in boxel_to_pybullet.items():
                    bname = binfo['name']
                    if bname in env.objects:
                        binfo['position'] = np.array(env.objects[bname].position)

                if obj_str in boxel_to_pybullet:
                    placed_obj_name = boxel_to_pybullet[obj_str]['name']
                    belief.mark_occluder_moved(obj_str, boxel_id_str)
                    print(f"    *** {placed_obj_name} PLACED at {boxel_id_str}! ***")
                else:
                    print(f"    *** {obj_str} PLACED at {boxel_id_str}! ***")
    
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
        remaining = belief.get_unknown_shadows()
        if exit_reason is None:
            exit_reason = "replan_limit"
        if exit_reason == "all_searched":
            print(f"FAILED: All {len(shadows)} shadows searched — target not found")
        elif exit_reason == "planner_failed":
            print(f"FAILED: Planner returned no plan "
                  f"({len(remaining)} unsearched shadows remaining)")
        else:
            print(f"FAILED: Replan limit reached ({max_replans}) with "
                  f"{len(remaining)} unsearched shadows remaining")
        print(f"  Plans executed: {plan_count}")
    print("=" * 60)
    
    if gui:
        import time
        print("\nWindow closing in 4 seconds...")
        end_time = time.time() + 4
        while time.time() < end_time:
            env.step_simulation()
            time.sleep(1.0 / 240.0)
    
    if grasp_constraint_id is not None:
        p.removeConstraint(grasp_constraint_id)
    
    env.close()
    return belief.is_target_found()


def sense_shadow_raycasting(camera_pos, shadow_boxel, target_pybullet_id, occluder_pybullet_id=None):
    """
    Sense a shadow region using PyBullet ray-casting from the fixed camera.

    Returns one of three outcomes:
      - found_target: at least one ray hits the target
      - clear_but_empty: no ray hits target and no ray hits the occluder
      - still_blocked: no ray hits target and at least one ray hits occluder

    This keeps visibility verification inside the sensing action (Phase 3):
    blocked sensing must not be treated as "target absent".

    Args:
        camera_pos: Fixed camera position [x, y, z]
        shadow_boxel: BoxelData for the shadow region to sense
        target_pybullet_id: PyBullet body ID of the target object
        occluder_pybullet_id: Optional PyBullet body ID of the occluder that
            geometrically blocks this shadow

    Returns:
        Tuple[str, float]:
          - outcome string in {"found_target", "clear_but_empty", "still_blocked"}
          - blocked_fraction (fraction of rays that hit occluder; 0 when unknown)
    """
    ray_origin = np.array(camera_pos)

    min_c = shadow_boxel.min_corner
    max_c = shadow_boxel.max_corner

    # Three Z slices through the shadow volume:
    # - Bottom slice at +0.04 m above min (half the target height 0.08 m,
    #   avoids hitting the table surface at min_z);
    # - Two interior slices at 33% and 67% of the shadow height.
    z_levels = [
        min_c[2] + 0.04,
        min_c[2] + (max_c[2] - min_c[2]) * 0.33,
        min_c[2] + (max_c[2] - min_c[2]) * 0.67,
    ]

    # 7×7 grid per Z slice = 147 total rays.  Empirically chosen:
    # 5×5 missed small targets at shadow edges; 9×9 doubled ray count
    # with negligible detection improvement.
    n = 7
    ray_froms = []
    ray_tos = []
    for z_target in z_levels:
        for xi in np.linspace(min_c[0], max_c[0], n):
            for yi in np.linspace(min_c[1], max_c[1], n):
                ray_froms.append(ray_origin.tolist())
                ray_tos.append([float(xi), float(yi), float(z_target)])

    results = p.rayTestBatch(ray_froms, ray_tos)
    occluder_hits = 0
    total_rays = len(results)

    for hit_obj_id, _link, _frac, _pos, _normal in results:
        if hit_obj_id == target_pybullet_id:
            return "found_target", 0.0
        if (occluder_pybullet_id is not None) and (hit_obj_id == occluder_pybullet_id):
            occluder_hits += 1

    if occluder_hits > 0:
        blocked_fraction = occluder_hits / total_rays if total_rays > 0 else 0.0
        return "still_blocked", blocked_fraction

    return "clear_but_empty", 0.0


def execute_pick(robot_id, env, obj_name, obj_pos, grasp, config, gui):
    """
    Execute pick action using the plan's grasp pose.

    All waypoints are derived from the grasp relative to the object's
    actual current position (obj_pos), not hardcoded offsets.  The
    contact waypoint re-derives IK from ``obj_pos + grasp.position``
    at execution time; if IK fails it falls back to the plan config.
    This handles any drift between the boxel center used during
    planning and the object's actual position.

    The constraint-based attachment (p.createConstraint) is an accepted
    simulation simplification — see audit #7 part B.

    Args:
        robot_id: PyBullet body ID of the robot
        env: BoxelTestEnv instance
        obj_name: Name key in env.objects (e.g. "target_1", "occluder_2")
        obj_pos: Current object position [x, y, z] (from PyBullet)
        grasp: Grasp object from the plan (position, orientation)
        config: RobotConfig from the plan's compute_kin_solution (fallback)
        gui: Whether GUI is active (for step_simulation timing)

    Returns:
        Tuple[int, RobotConfig]: PyBullet constraint ID for the grasp
        attachment, and a RobotConfig representing the robot's actual
        final joint configuration (lift position).
    """
    approach_height = 0.10
    lift_height = 0.25
    approach_dir = np.array([0.0, 0.0, 1.0])

    contact_ee = obj_pos + grasp.position
    approach_ee = contact_ee + approach_dir * approach_height
    lift_ee = contact_ee + approach_dir * lift_height

    pc = env.client_id
    approach_joints = solve_ik(robot_id, approach_ee, grasp.orientation, pc)
    lift_joints = solve_ik(robot_id, lift_ee, grasp.orientation, pc)

    if approach_joints is None or lift_joints is None:
        print(f"    WARNING: IK failed for pick waypoints of {obj_name}, "
              f"falling back to nearest valid waypoint")
        if lift_joints is None:
            lift_joints = approach_joints if approach_joints is not None \
                else config.joint_positions
        if approach_joints is None:
            approach_joints = config.joint_positions

    contact_joints = solve_ik(robot_id, contact_ee, grasp.orientation, pc)
    if contact_joints is None:
        contact_joints = config.joint_positions

    move_robot_smooth(robot_id, approach_joints, gui)
    open_gripper(robot_id, gui)

    move_robot_smooth(robot_id, contact_joints, gui)
    close_gripper(robot_id, gui)

    obj_id = env.objects[obj_name].object_id
    grasp_constraint_id = p.createConstraint(
        robot_id, END_EFFECTOR_LINK, obj_id, -1,
        p.JOINT_FIXED, [0, 0, 0], grasp.position.tolist(), [0, 0, 0]
    )

    move_robot_smooth(robot_id, lift_joints, gui)

    final_config = RobotConfig(joint_positions=np.asarray(lift_joints),
                               name="post_pick_lift")
    return grasp_constraint_id, final_config


def execute_place(robot_id, env, obj_name, place_pos, grasp, config,
                   grasp_constraint_id, gui):
    """
    Execute place action using the plan's grasp pose.

    Mirrors execute_pick() in reverse: approach above destination, lower
    to contact, release, retreat.  The contact waypoint re-derives IK
    from ``place_pos + grasp.position`` at execution time; falls back
    to the plan config if IK fails.

    Args:
        robot_id: PyBullet body ID of the robot
        env: BoxelTestEnv instance
        obj_name: Name of the object being placed (for logging)
        place_pos: Destination position [x, y, z] (boxel center)
        grasp: Grasp object from the plan (position, orientation)
        config: RobotConfig from the plan's compute_kin_solution (fallback)
        grasp_constraint_id: PyBullet constraint ID from execute_pick()
        gui: Whether GUI is active (for step_simulation timing)

    Returns:
        RobotConfig: The robot's actual final joint configuration
        (retreat position above the placement).
    """
    approach_height = 0.10
    retreat_height = 0.25
    approach_dir = np.array([0.0, 0.0, 1.0])

    contact_ee = place_pos + grasp.position
    approach_ee = contact_ee + approach_dir * approach_height
    retreat_ee = contact_ee + approach_dir * retreat_height

    pc = env.client_id
    approach_joints = solve_ik(robot_id, approach_ee, grasp.orientation, pc)
    retreat_joints = solve_ik(robot_id, retreat_ee, grasp.orientation, pc)

    if approach_joints is None or retreat_joints is None:
        print(f"    WARNING: IK failed for place waypoints of {obj_name}, "
              f"falling back to nearest valid waypoint")
        if retreat_joints is None:
            retreat_joints = approach_joints if approach_joints is not None \
                else config.joint_positions
        if approach_joints is None:
            approach_joints = config.joint_positions

    contact_joints = solve_ik(robot_id, contact_ee, grasp.orientation, pc)
    if contact_joints is None:
        contact_joints = config.joint_positions

    move_robot_smooth(robot_id, approach_joints, gui)

    move_robot_smooth(robot_id, contact_joints, gui)

    open_gripper(robot_id, gui)

    if grasp_constraint_id is not None:
        p.removeConstraint(grasp_constraint_id)

    # 30 steps ≈ 0.125 s — let the placed object settle before retreating.
    for _ in range(30):
        p.stepSimulation()

    move_robot_smooth(robot_id, retreat_joints, gui)

    return RobotConfig(joint_positions=np.asarray(retreat_joints),
                       name="post_place_retreat")


# compute_push_displacement() removed (#53): push superseded by pick-and-place.
# The function teleported occluders via p.resetBasePositionAndOrientation without
# involving the robot arm. Occluder relocation now uses pick → move → place.


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Full PDDLStream Pipeline with Replanning')
    parser.add_argument('--no-gui', action='store_true', help='Run without GUI')
    parser.add_argument('--log-level', choices=['quiet', 'normal', 'verbose'],
                        default='normal',
                        help='Console verbosity (log file always captures everything)')
    args = parser.parse_args()

    logger = RunLogger(verbosity=args.log_level)
    try:
        success = main(gui=not args.no_gui, run_logger=logger)
    finally:
        logger.close()
    sys.exit(0 if success else 1)
