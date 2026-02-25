#!/usr/bin/env python3
"""
Real PDDLStream Planner Integration for Semantic Boxel TAMP.

This module provides the interface between our Boxel TAMP system and
the actual PDDLStream solver with FastDownward backend.

Usage (from WSL):
    source wsl_env/bin/activate
    export PYTHONPATH=/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib
    python3 pddlstream_planner.py
"""

import sys
import os

# Add pddlstream to path (for WSL)
PDDLSTREAM_PATH = '/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib'
if PDDLSTREAM_PATH not in sys.path:
    sys.path.insert(0, PDDLSTREAM_PATH)

import numpy as np
from typing import List, Tuple, Optional, Dict, Any

from pddlstream.algorithms.meta import solve
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import read

from boxel_data import BoxelRegistry, BoxelType
from streams import BoxelStreams, RobotConfig


def read_pddl_file(filename: str) -> str:
    """Read PDDL file content."""
    pddl_dir = os.path.join(os.path.dirname(__file__), 'pddl')
    filepath = os.path.join(pddl_dir, filename)
    with open(filepath, 'r') as f:
        return f.read()


class PDDLStreamPlanner:
    """
    Real PDDLStream planner for Semantic Boxel TAMP.
    
    Uses PDDLStream with FastDownward for planning with continuous
    stream sampling for IK and motion planning.
    """
    
    def __init__(self, registry: BoxelRegistry, robot_id: int = None,
                 shadow_occluder_map: Dict[str, str] = None,
                 physics_client: int = None):
        """
        Initialize the planner.
        
        Args:
            registry: BoxelRegistry with scene boxels
            robot_id: PyBullet robot body ID (for real IK; None = heuristic fallback)
            shadow_occluder_map: Dict mapping shadow_id -> occluder_id
            physics_client: PyBullet physics client ID (None = default 0)
        """
        self.registry = registry
        self.robot_id = robot_id
        self.shadow_occluder_map = shadow_occluder_map or {}
        self._shadow_ids = [b.id for b in registry.boxels.values()
                            if b.boxel_type == BoxelType.SHADOW]

        self.domain_pddl = read_pddl_file('domain_pddlstream.pddl')
        self.stream_pddl = read_pddl_file('stream.pddl')

        self.streams = BoxelStreams(
            registry, robot_id=robot_id, physics_client=physics_client
        )
    
    def _init_stream_stats(self):
        """Reset per-plan stream call counters."""
        self._stream_stats = {
            'sample-push-config': {'calls': 0, 'results': 0, 'by_input': {}},
            'sample-sensing-config': {'calls': 0, 'results': 0, 'by_input': {}},
            'sample-grasp': {'calls': 0, 'results': 0, 'by_input': {}},
            'compute-kin': {'calls': 0, 'results': 0, 'by_input': {}},
            'plan-motion': {'calls': 0, 'results': 0},
        }

    def _logged(self, name, gen_fn):
        """Wrap a generator to count calls and results for diagnostics."""
        stats = self._stream_stats[name]
        def wrapper(*args):
            key = str(args[0]) if args else ''
            stats['calls'] += 1
            count = 0
            for result in gen_fn(*args):
                count += 1
                yield result
            stats['results'] += count
            if 'by_input' in stats:
                stats['by_input'][key] = stats['by_input'].get(key, 0) + count
        return wrapper

    def _print_stream_stats(self):
        """Print a summary of what each stream produced (for debugging)."""
        print("\n  Stream results:")
        for name, s in self._stream_stats.items():
            line = f"    {name}: {s['results']} results from {s['calls']} calls"
            if 'by_input' in s and s['by_input']:
                details = ', '.join(f"{k}={v}" for k, v in sorted(s['by_input'].items()))
                line += f"  [{details}]"
            print(line)

    def _get_stream_map(self) -> Dict[str, Any]:
        """
        Create the stream map connecting stream names to Python generators.
        
        Each stream delegates to BoxelStreams, which computes real IK via
        PyBullet (or heuristic fallback if no robot_id). The yielded values
        are RobotConfig / Grasp / Trajectory objects that carry actual joint
        angles, grasp poses, and trajectory waypoints.
        
        No caps — the incremental algorithm evaluates all streams eagerly
        and searches one fully-grounded problem. More results = richer
        search space = better chance of finding a plan.
        
        Returns:
            Dict mapping stream names to generator functions
        """
        return {
            'sample-push-config': from_gen_fn(self._logged('sample-push-config', self.streams.sample_push_config)),
            'sample-sensing-config': from_gen_fn(self._logged('sample-sensing-config', self.streams.sample_sensing_config)),
            'sample-grasp': from_gen_fn(self._logged('sample-grasp', self.streams.sample_grasp)),
            'plan-motion': from_gen_fn(self._logged('plan-motion', self.streams.plan_motion)),
            'compute-kin': from_gen_fn(self._logged('compute-kin', self.streams.compute_kin_solution)),
        }
    
    def create_problem(self, 
                       target_objects: List[str],
                       goal_expr: str,
                       current_config: Optional[RobotConfig] = None,
                       known_empty_shadows: List[str] = None,
                       moved_occluders: List[str] = None) -> PDDLProblem:
        """
        Create a PDDLStream problem from current state.
        
        Args:
            target_objects: Objects to reason about
            goal_expr: Goal expression like "(holding target_1)"
            current_config: Robot configuration (None = home pose with real joint angles)
            known_empty_shadows: Shadows we've already checked (not containing target)
            moved_occluders: Occluders that have been pushed aside
            
        Returns:
            PDDLProblem for PDDLStream solver
        """
        if current_config is None:
            current_config = self.streams.home_config
        known_empty_shadows = known_empty_shadows or []
        moved_occluders = moved_occluders or []
        
        # Build initial state as list of facts (tuples)
        init = []
        
        # Track occluders and their shadows
        occluders = []
        shadows = []
        
        # Add boxel facts with type predicates
        for boxel in self.registry.boxels.values():
            init.append(('Boxel', boxel.id))  # Type predicate
            
            if boxel.boxel_type == BoxelType.SHADOW:
                init.append(('is_shadow', boxel.id))
                shadows.append(boxel.id)
                
                # If we've searched this shadow and found nothing, mark as KNOWN NOT HERE
                if boxel.id in known_empty_shadows:
                    for obj in target_objects:
                        init.append(('obj_at_boxel_KIF', obj, boxel.id))
                # Otherwise still UNKNOWN - no KIF fact
                
            elif boxel.boxel_type == BoxelType.OBJECT:
                init.append(('is_occluder', boxel.id))
                # Only blocking if NOT moved yet
                if boxel.id not in moved_occluders:
                    init.append(('occluder_blocking', boxel.id))
                else:
                    init.append(('occluder_aside', boxel.id))
                occluders.append(boxel.id)
                # Known NOT in boxel for targets
                for obj in target_objects:
                    init.append(('obj_at_boxel_KIF', obj, boxel.id))
                    
            elif boxel.boxel_type == BoxelType.FREE_SPACE:
                init.append(('is_free_space', boxel.id))
                # Known NOT in boxel for targets
                for obj in target_objects:
                    init.append(('obj_at_boxel_KIF', obj, boxel.id))
        
        # Add casts_shadow relationships
        # Map shadows to their occluders based on provided map or registry data
        if self.shadow_occluder_map:
            for shadow_id, occluder_id in self.shadow_occluder_map.items():
                init.append(('casts_shadow', occluder_id, shadow_id))
        else:
            # Derive from registry's ground-truth created_by_boxel_id
            for shadow_id in shadows:
                shadow_boxel = self.registry.get_boxel(shadow_id)
                if shadow_boxel and shadow_boxel.created_by_boxel_id:
                    init.append(('casts_shadow', shadow_boxel.created_by_boxel_id, shadow_id))
        
        # Add target objects with type predicates
        for obj in target_objects:
            init.append(('Obj', obj))  # Type predicate
        
        # Robot state
        init.append(('Config', current_config))  # Type predicate
        init.append(('at_config', current_config))
        init.append(('handempty',))
        
        # Parse goal - simple for now
        # Assuming goal like "(holding target_1)"
        if 'holding' in goal_expr:
            obj_name = goal_expr.replace('(', '').replace(')', '').split()[1]
            goal = ('holding', obj_name)
        else:
            goal = goal_expr  # Use as-is
        
        # Create problem
        constant_map = {}  # No constants
        stream_map = self._get_stream_map()
        
        return PDDLProblem(
            self.domain_pddl,
            constant_map,
            self.stream_pddl,
            stream_map,
            init,
            goal
        )
    
    def plan(self,
             target_objects: List[str],
             goal: str,
             current_config: Optional[RobotConfig] = None,
             known_empty_shadows: List[str] = None,
             moved_occluders: List[str] = None,
             max_time: float = 30.0,
             verbose: bool = True) -> Optional[List[Tuple]]:
        """
        Generate a plan using PDDLStream.
        
        Args:
            target_objects: Objects to reason about
            goal: Goal expression
            current_config: Robot configuration (None = home pose)
            known_empty_shadows: Shadows already checked (empty)
            moved_occluders: Occluders already pushed aside
            max_time: Maximum planning time in seconds
            verbose: Print planning info
            
        Returns:
            List of action tuples, or None if planning fails
        """
        self._init_stream_stats()
        problem = self.create_problem(target_objects, goal, current_config,
                                      known_empty_shadows, moved_occluders)

        known_empty_shadows = known_empty_shadows or []
        moved_occluders_list = moved_occluders or []

        if verbose:
            unknown = [s for s in self._shadow_ids if s not in known_empty_shadows]
            print(f"  Init: {len(unknown)} unknown shadows, "
                  f"{len(known_empty_shadows)} checked empty, "
                  f"{len(moved_occluders_list)} occluders aside")
            for s in unknown:
                occ = self.shadow_occluder_map.get(s, '?')
                aside = 'aside' if occ in moved_occluders_list else 'blocking'
                print(f"    {s} <- {occ} ({aside})")

        import time
        t0 = time.time()
        solution = solve(
            problem,
            algorithm='incremental',
            max_time=max_time,
            unit_costs=True,
            verbose=False
        )
        elapsed = time.time() - t0

        plan, cost, certificate = solution

        if verbose:
            status = "SOLVED" if plan else "FAILED"
            print(f"  {status} in {elapsed:.1f}s (cost={cost})")
            self._print_stream_stats()

        if plan is None:
            return None
        
        # Convert plan to our action format
        actions = []
        for action in plan:
            action_name = action.name
            action_args = action.args
            actions.append((action_name,) + tuple(action_args))
        
        return actions


def test_planner():
    """Test the PDDLStream planner."""
    print("="*60)
    print("Testing PDDLStream Planner")
    print("="*60)
    
    # Load registry from JSON
    registry = BoxelRegistry.load_from_json('boxel_data.json')
    print(f"Loaded {len(registry.boxels)} boxels")
    
    # Count shadow and occluder boxels
    shadows = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    occluders = [b.id for b in registry.boxels.values() if b.boxel_type == BoxelType.OBJECT]
    print(f"Shadow boxels: {len(shadows)}")
    print(f"Occluder boxels: {len(occluders)}")
    
    # Create shadow->occluder mapping from registry ground-truth
    shadow_occluder_map = {}
    for shadow_id in shadows:
        shadow_boxel = registry.get_boxel(shadow_id)
        if shadow_boxel and shadow_boxel.created_by_boxel_id:
            shadow_occluder_map[shadow_id] = shadow_boxel.created_by_boxel_id
    print(f"Shadow-Occluder mapping: {shadow_occluder_map}")
    
    # Create planner
    planner = PDDLStreamPlanner(registry, shadow_occluder_map=shadow_occluder_map)
    
    # Plan
    print("\nPlanning to find and hold target_1...")
    plan = planner.plan(
        target_objects=['target_1'],
        goal='(holding target_1)',
        max_time=60.0,
        verbose=True
    )
    
    if plan:
        print("\n--- Generated Plan ---")
        for i, action in enumerate(plan):
            print(f"  {i+1}. {action}")
    else:
        print("\nNo plan found!")
    
    return plan is not None


if __name__ == "__main__":
    success = test_planner()
    sys.exit(0 if success else 1)
