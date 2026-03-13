#!/usr/bin/env python3
"""
Real PDDLStream Planner Integration for Semantic Boxel TAMP.

This module provides the interface between our Boxel TAMP system and
the actual PDDLStream solver with FastDownward backend.

Usage (from WSL):
    source wsl_env/bin/activate
    export PYTHONPATH=/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib
    python3 pddlstream_planner.py

PDDLStream path is resolved automatically via config.py
(PROJECT_ROOT/../pddlstream_lib). No manual PYTHONPATH export needed.
"""

import sys
import os

# Add pddlstream to path (for WSL)
PDDLSTREAM_PATH = '/mnt/c/Users/HaniAlassiriAlhabbou/git/pddlstream_lib'
if PDDLSTREAM_PATH not in sys.path:
    sys.path.insert(0, PDDLSTREAM_PATH)

import numpy as np
from typing import List, Tuple, Optional, Dict, Any, Union

from pddlstream.algorithms.meta import solve
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import read

from boxel_data import BoxelRegistry, BoxelType
from streams import BoxelStreams, RobotConfig, Trajectory, Grasp
from robot_utils import REST_POSES


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
                 physics_client: int = None,
                 object_body_ids: Dict[str, int] = None,
                 support_body_ids: frozenset = None):
        """
        Initialize the planner.

        Args:
            registry: BoxelRegistry with scene boxels
            robot_id: PyBullet robot body ID (required for real IK;
                without it, BoxelStreams falls back to heuristic IK)
            shadow_occluder_map: Dict mapping shadow_id -> occluder_id
            physics_client: PyBullet physics client ID (0 if default)
            object_body_ids: Mapping from object/boxel identifiers to
                PyBullet body IDs.  Passed to BoxelStreams so that
                compute_kin and plan_motion can exclude grasped objects
                from collision checks.
            support_body_ids: Body IDs of support surfaces (table, ground
                plane) ignored during pick/place endpoint checks.
        """
        self.registry = registry
        self.robot_id = robot_id
        self.shadow_occluder_map = shadow_occluder_map or {}

        self.streams = BoxelStreams(
            registry, robot_id=robot_id, physics_client=physics_client,
            object_body_ids=object_body_ids,
            support_body_ids=support_body_ids
        )
        self.home_config = self.streams.home_config
        
        # Load PDDL files (use PDDLStream-compatible untyped domain)
        self.domain_pddl = read_pddl_file('domain_pddlstream.pddl')
        self.stream_pddl = read_pddl_file('stream.pddl')
    
    def _get_stream_map(self) -> Dict[str, Any]:
        """
        Create the stream map connecting stream names to BoxelStreams generators.
        
        Each entry wraps a BoxelStreams method via PDDLStream's from_gen_fn.
        The streams produce real geometric objects (RobotConfig, Grasp,
        Trajectory) instead of placeholder strings.

        Returns:
            Dict mapping stream names to generator functions
        """
        return {
            'sample-grasp': from_gen_fn(self.streams.sample_grasp),
            'plan-motion': from_gen_fn(self.streams.plan_motion),
            'compute-kin': from_gen_fn(self.streams.compute_kin_solution),
        }
    
    def create_problem(self, 
                       target_objects: List[str],
                       goal: Tuple,
                       current_config: 'Union[RobotConfig, str]' = None,
                       known_empty_shadows: List[str] = None,
                       moved_occluders: Dict[str, str] = None) -> PDDLProblem:
        """
        Create a PDDLStream problem from current state.
        
        Args:
            target_objects: Objects to reason about
            goal: Goal as a tuple, e.g. ('holding', 'target_1').
                  Passed directly to PDDLStream — must match domain predicates.
            current_config: Current robot config (RobotConfig preferred;
                string accepted for backward compat, wrapped automatically)
            known_empty_shadows: Shadows we've already checked (not containing target)
            moved_occluders: Dict mapping occluder_id -> destination_boxel_id
            
        Returns:
            PDDLProblem for PDDLStream solver
        """
        if current_config is None:
            current_config = self.home_config
        init = self._build_init(target_objects, current_config,
                                known_empty_shadows, moved_occluders)
        
        constant_map = {}
        stream_map = self._get_stream_map()
        
        return PDDLProblem(
            self.domain_pddl,
            constant_map,
            self.stream_pddl,
            stream_map,
            init,
            goal
        )
    
    def export_problem_pddl(self,
                            target_objects: List[str],
                            goal: Tuple,
                            current_config: 'Union[RobotConfig, str]' = None,
                            known_empty_shadows: List[str] = None,
                            moved_occluders: Dict[str, str] = None,
                            filepath: str = "pddl/problem_debug.pddl") -> str:
        """
        Export the programmatically-built problem to a standalone PDDL file.

        Useful for debugging: inspect exactly what init state and goal the
        planner receives, without running PDDLStream. The output file matches
        domain_pddlstream.pddl's untyped format.

        Note: stream-certified predicates (push_solution, kin_solution, etc.)
        are populated at runtime by PDDLStream streams and will NOT appear in
        this file. The problem is therefore not solvable by a plain PDDL
        planner — it's a snapshot of the static init state for inspection.

        Args:
            target_objects: Objects to reason about
            goal: Goal as a tuple, e.g. ('holding', 'target_1')
            current_config: Current robot config (RobotConfig preferred)
            known_empty_shadows: Shadows already checked (empty)
            moved_occluders: Dict mapping occluder_id -> destination_boxel_id
            filepath: Output path (default: pddl/problem_debug.pddl)

        Returns:
            The filepath written to
        """
        if current_config is None:
            current_config = self.home_config
        known_empty_shadows = known_empty_shadows or []
        moved_occluders = moved_occluders or {}

        init = self._build_init(target_objects, current_config,
                                known_empty_shadows, moved_occluders)

        objects = set()
        for fact in init:
            for arg in fact[1:]:
                objects.add(str(arg))

        def format_fact(fact):
            if len(fact) == 1:
                return f"    ({fact[0]})"
            return f"    ({' '.join(str(a) for a in fact)})"

        def format_goal(g):
            if isinstance(g, str):
                return f"({g})"
            if g[0] == 'and':
                inner = ' '.join(format_goal(sub) for sub in g[1:])
                return f"(and {inner})"
            return f"({' '.join(str(a) for a in g)})"

        lines = [
            ";; Auto-generated problem file from PDDLStreamPlanner.export_problem_pddl()",
            ";; Static init state only — stream-certified facts are NOT included.",
            "",
            "(define (problem boxel-tamp-debug)",
            "  (:domain boxel-tamp)",
            "",
            "  (:objects",
        ]
        for obj in sorted(objects):
            lines.append(f"    {obj}")
        lines.append("  )")
        lines.append("")
        lines.append("  (:init")
        for fact in sorted(init, key=lambda f: (str(f[0]),) + tuple(str(a) for a in f[1:])):
            lines.append(format_fact(fact))
        lines.append("  )")
        lines.append("")
        lines.append(f"  (:goal {format_goal(goal)})")
        lines.append(")")
        lines.append("")

        output_path = os.path.join(os.path.dirname(__file__), filepath)
        with open(output_path, 'w') as f:
            f.write('\n'.join(lines))

        return output_path

    def _build_init(self,
                    target_objects: List[str],
                    current_config: 'Union[RobotConfig, str]' = None,
                    known_empty_shadows: List[str] = None,
                    moved_occluders: Dict[str, str] = None) -> List[Tuple]:
        """
        Build the init state as a list of fact tuples.

        Shared by create_problem() and export_problem_pddl() to guarantee
        they produce identical init states.

        Args:
            target_objects: Objects to reason about
            current_config: Current robot config (RobotConfig or string)
            known_empty_shadows: Shadows already checked (empty)
            moved_occluders: Dict mapping occluder_id -> destination_boxel_id
                for occluders that have been pushed aside

        Returns:
            List of fact tuples for the init state
        """
        if current_config is None:
            current_config = self.home_config
        known_empty_shadows = known_empty_shadows or []
        moved_occluders = moved_occluders or {}

        init = []
        shadows = []

        for boxel in self.registry.boxels.values():
            init.append(('Boxel', boxel.id))

            if boxel.boxel_type == BoxelType.SHADOW:
                init.append(('is_shadow', boxel.id))
                shadows.append(boxel.id)

                if boxel.id in known_empty_shadows:
                    for obj in target_objects:
                        init.append(('obj_at_boxel_KIF', obj, boxel.id))

            elif boxel.boxel_type == BoxelType.OBJECT:
                init.append(('is_object', boxel.id))
                init.append(('Obj', boxel.id))
                if boxel.id in moved_occluders:
                    dest = moved_occluders[boxel.id]
                    init.append(('Boxel', dest))
                    init.append(('obj_at_boxel', boxel.id, dest))
                    init.append(('obj_at_boxel_KIF', boxel.id, dest))
                else:
                    init.append(('obj_at_boxel', boxel.id, boxel.id))
                    init.append(('obj_at_boxel_KIF', boxel.id, boxel.id))
                for obj in target_objects:
                    init.append(('obj_at_boxel_KIF', obj, boxel.id))

            elif boxel.boxel_type == BoxelType.FREE_SPACE:
                init.append(('is_free_space', boxel.id))
                for obj in target_objects:
                    init.append(('obj_at_boxel_KIF', obj, boxel.id))

        # Static geometric facts: blocks_view_at(occ, occ_boxel, shadow).
        # Always added regardless of moved status — they describe geometry,
        # not current state. The derived predicate blocks_view combines these
        # with obj_at_boxel to determine actual view blockage.
        if self.shadow_occluder_map:
            for shadow_id, occluder_id in self.shadow_occluder_map.items():
                init.append(('blocks_view_at', occluder_id, occluder_id, shadow_id))
        else:
            for shadow_id in shadows:
                shadow_boxel = self.registry.get_boxel(shadow_id)
                if shadow_boxel and shadow_boxel.created_by_boxel_id:
                    occ_id = shadow_boxel.created_by_boxel_id
                    init.append(('blocks_view_at', occ_id, occ_id, shadow_id))

        for obj in target_objects:
            init.append(('Obj', obj))

        init.append(('Config', current_config))
        init.append(('at_config', current_config))
        init.append(('handempty',))

        return init

    def plan(self,
             target_objects: List[str],
             goal: Tuple,
             current_config: 'Union[RobotConfig, str]' = None,
             known_empty_shadows: List[str] = None,
             moved_occluders: Dict[str, str] = None,
             max_time: float = 30.0,
             verbose: bool = True) -> Optional[List[Tuple]]:
        """
        Generate a plan using PDDLStream.
        
        Args:
            target_objects: Objects to reason about
            goal: Goal as a tuple, e.g. ('holding', 'target_1')
            current_config: Current robot config (RobotConfig preferred)
            known_empty_shadows: Shadows already checked (empty)
            moved_occluders: Dict mapping occluder_id -> destination_boxel_id
            max_time: Maximum planning time in seconds
            verbose: Print planning info
            
        Returns:
            List of action tuples, or None if planning fails
        """
        if current_config is None:
            current_config = self.home_config
        problem = self.create_problem(target_objects, goal, current_config,
                                      known_empty_shadows, moved_occluders)
        
        if verbose:
            print(f"\n--- PDDLStream Planning ---")
            print(f"Goal: {goal}")
            print(f"Max time: {max_time}s")
        
        # Call PDDLStream solver
        solution = solve(
            problem,
            algorithm='adaptive',  # Best for TAMP problems
            max_time=max_time,
            verbose=verbose
        )
        
        plan, cost, certificate = solution
        
        if verbose:
            print_solution(solution)
        
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
    """Test the PDDLStream planner (uses heuristic IK without a robot)."""
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
    
    # Create planner (no robot_id: BoxelStreams will use heuristic IK)
    planner = PDDLStreamPlanner(registry, shadow_occluder_map=shadow_occluder_map)
    
    # Plan (uses planner.home_config as default current_config)
    print("\nPlanning to find and hold target_1...")
    plan = planner.plan(
        target_objects=['target_1'],
        goal=('holding', 'target_1'),
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
