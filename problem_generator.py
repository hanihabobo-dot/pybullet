"""
PDDL Problem Generator for Semantic Boxel TAMP.

Converts BoxelRegistry state to PDDL problem format for PDDLStream.
Handles the Know-If fluent pattern for partial observability.
"""

from typing import List, Set, Optional
from dataclasses import dataclass

from boxel_data import BoxelRegistry, BoxelData, BoxelType


@dataclass
class PlanningProblem:
    """Container for a PDDL planning problem."""
    domain_name: str
    problem_name: str
    objects: List[str]
    init: Set[str]
    goal: str
    
    def to_pddl(self) -> str:
        """Convert to PDDL string format."""
        objects_str = "\n    ".join(self.objects)
        init_str = "\n    ".join(sorted(self.init))
        
        return f"""(define (problem {self.problem_name})
  (:domain {self.domain_name})
  
  (:objects
    {objects_str}
  )
  
  (:init
    {init_str}
  )
  
  (:goal
    {self.goal}
  )
)"""


class ProblemGenerator:
    """
    Generates PDDL problems from BoxelRegistry for PDDLStream.
    
    Handles the Know-If fluent pattern:
      - obj_at_boxel(?o, ?b)     = ground truth
      - obj_at_boxel_KIF(?o, ?b) = we know the value
    
    Initial epistemic state:
      - Object boxels: KIF=true (we see them), at=true if object there
      - Shadow boxels: KIF=false (unknown), at=unknown
      - After sensing: KIF=true, at=observation result
    """
    
    def __init__(self, registry: BoxelRegistry):
        """
        Initialize with boxel registry.
        
        Args:
            registry: BoxelRegistry with current scene state
        """
        self.registry = registry
    
    def generate_problem(self, 
                         target_objects: List[str],
                         goal: str,
                         hidden_object_locations: dict = None,
                         current_config: str = "q_home") -> PlanningProblem:
        """
        Generate a PDDL problem for finding and manipulating objects.
        
        Args:
            target_objects: List of object names to reason about
            goal: Goal specification string (e.g., "(holding target_1)")
            hidden_object_locations: Dict mapping object -> boxel_id for ground truth
                                     (used by executor, not visible to planner)
            current_config: Name of current robot configuration
            
        Returns:
            PlanningProblem ready for PDDLStream
        """
        objects = []
        init = set()
        
        # --- Add Boxels ---
        boxel_ids = []
        shadow_ids = []
        object_boxel_ids = []
        
        for boxel in self.registry.boxels.values():
            boxel_ids.append(boxel.id)
            objects.append(f"{boxel.id} - boxel")
            init.add(f"(semantic_zone {boxel.id})")
            
            if boxel.boxel_type == BoxelType.SHADOW:
                shadow_ids.append(boxel.id)
                init.add(f"(is_shadow {boxel.id})")
                
            elif boxel.boxel_type == BoxelType.OBJECT:
                object_boxel_ids.append(boxel.id)
                init.add(f"(is_object_boxel {boxel.id})")
                
                # Add occlusion relationships
                for shadow_id in boxel.shadow_boxel_ids:
                    init.add(f"(occludes {boxel.id} {shadow_id})")
        
        # --- Add Target Objects ---
        for obj_name in target_objects:
            objects.append(f"{obj_name} - obj")
            init.add(f"(obj_graspable {obj_name})")
            
            # Epistemic state for object boxels: we KNOW target is NOT there
            # (we can see the whole boxel and target isn't visible)
            for obj_boxel_id in object_boxel_ids:
                init.add(f"(obj_at_boxel_KIF {obj_name} {obj_boxel_id})")
                # Note: NOT adding obj_at_boxel means we know it's NOT there
            
            # Epistemic state for FREE SPACE boxels: we KNOW target is NOT there
            # (free space was observed to be empty during octree discretization)
            for boxel in self.registry.get_free_space_boxels():
                init.add(f"(obj_at_boxel_KIF {obj_name} {boxel.id})")
                # Note: NOT adding obj_at_boxel means we know it's NOT there
            
            # Epistemic state for shadow boxels: we DON'T KNOW
            # Don't add KIF for shadows -> unknown
            # The planner must sense to discover
        
        # --- Add Robot State ---
        objects.append(f"{current_config} - config")
        init.add(f"(at_config {current_config})")
        init.add("(handempty)")
        
        return PlanningProblem(
            domain_name="boxel-tamp",
            problem_name="boxel-sensing-problem",
            objects=objects,
            init=init,
            goal=goal
        )
    
    def update_after_sensing(self, problem: PlanningProblem,
                             obj_name: str, boxel_id: str, 
                             found: bool) -> PlanningProblem:
        """
        Update problem state after a sensing action.
        
        This is called by the executor when a sensing action is performed.
        
        Args:
            problem: Current planning problem
            obj_name: Object we were looking for
            boxel_id: Boxel we sensed
            found: Whether the object was found
            
        Returns:
            Updated PlanningProblem with new epistemic state
        """
        new_init = set(problem.init)
        
        # We now KNOW the value (add KIF)
        new_init.add(f"(obj_at_boxel_KIF {obj_name} {boxel_id})")
        
        if found:
            # Object IS in this boxel
            new_init.add(f"(obj_at_boxel {obj_name} {boxel_id})")
            new_init.add(f"(obj_pose_known {obj_name})")
            
            # Infer: object is NOT in any other SHADOW boxel
            # (closed-world: object can only be in one place)
            # Note: Object and free space boxels already have KIF=true from init
            for other_boxel in self.registry.get_shadow_boxels():
                if other_boxel.id != boxel_id:
                    new_init.add(f"(obj_at_boxel_KIF {obj_name} {other_boxel.id})")
                    # NOT adding obj_at_boxel = we know it's NOT there
        else:
            # Object is NOT in this boxel (KIF added, but no obj_at_boxel)
            pass  # KIF already added above
        
        return PlanningProblem(
            domain_name=problem.domain_name,
            problem_name=problem.problem_name,
            objects=problem.objects,
            init=new_init,
            goal=problem.goal
        )
    
    def print_epistemic_state(self, problem: PlanningProblem, 
                              target_objects: List[str]) -> None:
        """
        Print human-readable epistemic state for debugging.
        
        Args:
            problem: Current planning problem
            target_objects: Objects to show state for
        """
        print("\n" + "="*60)
        print("EPISTEMIC STATE (Know-If Fluents)")
        print("="*60)
        
        for obj_name in target_objects:
            print(f"\nObject: {obj_name}")
            print("-" * 40)
            
            for boxel in self.registry.boxels.values():
                kif_fact = f"(obj_at_boxel_KIF {obj_name} {boxel.id})"
                at_fact = f"(obj_at_boxel {obj_name} {boxel.id})"
                
                has_kif = kif_fact in problem.init
                has_at = at_fact in problem.init
                
                if has_kif and has_at:
                    status = "✓ KNOWN IN BOXEL"
                elif has_kif and not has_at:
                    status = "✗ KNOWN NOT IN BOXEL"
                else:
                    status = "? UNKNOWN"
                
                boxel_type = boxel.boxel_type.value
                print(f"  {boxel.id:15} ({boxel_type:10}): {status}")
        
        print("="*60 + "\n")


def create_test_problem(registry: BoxelRegistry) -> PlanningProblem:
    """
    Create a test problem for manual verification.
    
    Args:
        registry: BoxelRegistry from the scene
        
    Returns:
        PlanningProblem for testing
    """
    generator = ProblemGenerator(registry)
    
    problem = generator.generate_problem(
        target_objects=["target_1"],
        goal="(holding target_1)"
    )
    
    return problem
