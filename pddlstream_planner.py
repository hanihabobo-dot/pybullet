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
    
    def __init__(self, registry: BoxelRegistry, robot_id: int = None):
        """
        Initialize the planner.
        
        Args:
            registry: BoxelRegistry with scene boxels
            robot_id: PyBullet robot body ID (optional, for real IK)
        """
        self.registry = registry
        self.robot_id = robot_id
        
        # Load PDDL files (use PDDLStream-compatible untyped domain)
        self.domain_pddl = read_pddl_file('domain_pddlstream.pddl')
        self.stream_pddl = read_pddl_file('stream.pddl')
        
        # Counters for unique names
        self._config_count = 0
        self._grasp_count = 0
        self._traj_count = 0
    
    def _get_stream_map(self) -> Dict[str, Any]:
        """
        Create the stream map connecting stream names to Python generators.
        
        Returns:
            Dict mapping stream names to generator functions
        """
        return {
            'sample-sensing-config': from_gen_fn(self._gen_sensing_config),
            'sample-grasp': from_gen_fn(self._gen_grasp),
            'plan-motion': from_gen_fn(self._gen_motion),
            'compute-kin': from_gen_fn(self._gen_kin_solution),
        }
    
    def _gen_sensing_config(self, boxel_id: str):
        """Generator for sensing configurations."""
        boxel = self.registry.get_boxel(boxel_id)
        if boxel is None:
            return
        
        # Generate a few viewing configurations
        for i in range(3):
            self._config_count += 1
            config_name = f"q_sense_{boxel_id}_{self._config_count}"
            yield (config_name,)
    
    def _gen_grasp(self, obj_id: str):
        """Generator for grasp poses."""
        for i in range(2):
            self._grasp_count += 1
            grasp_name = f"grasp_{obj_id}_{self._grasp_count}"
            yield (grasp_name,)
    
    def _gen_motion(self, q1: str, q2: str):
        """Generator for motion plans."""
        self._traj_count += 1
        traj_name = f"traj_{self._traj_count}"
        yield (traj_name,)
    
    def _gen_kin_solution(self, obj_id: str, boxel_id: str, grasp: str):
        """Generator for kinematic solutions."""
        self._config_count += 1
        config_name = f"q_pick_{obj_id}_{self._config_count}"
        yield (config_name,)
    
    def create_problem(self, 
                       target_objects: List[str],
                       goal_expr: str,
                       current_config: str = "q_home") -> PDDLProblem:
        """
        Create a PDDLStream problem from current state.
        
        Args:
            target_objects: Objects to reason about
            goal_expr: Goal expression like "(holding target_1)"
            current_config: Current robot configuration name
            
        Returns:
            PDDLProblem for PDDLStream solver
        """
        # Build initial state as list of facts (tuples)
        init = []
        
        # Add boxel facts with type predicates
        for boxel in self.registry.boxels.values():
            init.append(('Boxel', boxel.id))  # Type predicate
            init.append(('semantic_zone', boxel.id))
            
            if boxel.boxel_type == BoxelType.SHADOW:
                init.append(('is_shadow', boxel.id))
                # Shadows start as UNKNOWN - no KIF fact
                
            elif boxel.boxel_type == BoxelType.OBJECT:
                init.append(('is_occluder', boxel.id))
                # Known NOT in boxel for targets
                for obj in target_objects:
                    init.append(('obj_at_boxel_KIF', obj, boxel.id))
                    # Note: NOT having obj_at_boxel means known not there
                    
            elif boxel.boxel_type == BoxelType.FREE_SPACE:
                # Known NOT in boxel for targets
                for obj in target_objects:
                    init.append(('obj_at_boxel_KIF', obj, boxel.id))
        
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
             current_config: str = "q_home",
             max_time: float = 30.0,
             verbose: bool = True) -> Optional[List[Tuple]]:
        """
        Generate a plan using PDDLStream.
        
        Args:
            target_objects: Objects to reason about
            goal: Goal expression
            current_config: Current robot config
            max_time: Maximum planning time in seconds
            verbose: Print planning info
            
        Returns:
            List of action tuples, or None if planning fails
        """
        problem = self.create_problem(target_objects, goal, current_config)
        
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
    """Test the PDDLStream planner."""
    print("="*60)
    print("Testing PDDLStream Planner")
    print("="*60)
    
    # Load registry from JSON
    registry = BoxelRegistry.load_from_json('boxel_data.json')
    print(f"Loaded {len(registry.boxels)} boxels")
    
    # Count shadow boxels
    shadows = [b for b in registry.boxels.values() if b.boxel_type == BoxelType.SHADOW]
    print(f"Shadow boxels: {len(shadows)}")
    
    # Create planner
    planner = PDDLStreamPlanner(registry)
    
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
