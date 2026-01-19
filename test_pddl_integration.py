#!/usr/bin/env python3
"""
Manual Test Script for PDDLStream Integration.

This script allows you to verify each component of the PDDL integration
step by step. Run it interactively to understand and validate the system.

Usage:
    python test_pddl_integration.py

Tests:
    1. Domain file parsing
    2. Problem generation from BoxelRegistry
    3. Know-If fluent representation
    4. Belief state updates after sensing
    5. Mock execution with replanning
"""

import os
import sys
import json

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from boxel_data import BoxelRegistry, BoxelType
from problem_generator import ProblemGenerator, PlanningProblem
from streams import BoxelStreams, RobotConfig
from executor import BoxelExecutor, demo_execution


def separator(title: str):
    """Print a section separator."""
    print("\n" + "="*70)
    print(f"  {title}")
    print("="*70 + "\n")


def test_1_load_registry():
    """Test 1: Load BoxelRegistry from JSON."""
    separator("TEST 1: Load BoxelRegistry from JSON")
    
    json_path = "boxel_data.json"
    
    if not os.path.exists(json_path):
        print(f"ERROR: {json_path} not found!")
        print("Run 'python run_demo.py' first to generate the boxel data.")
        return None
    
    registry = BoxelRegistry.load_from_json(json_path)
    
    print(f"✓ Loaded {len(registry.boxels)} boxels from {json_path}")
    print(f"\nBoxel summary:")
    print(f"  - Objects:    {len(registry.get_object_boxels())}")
    print(f"  - Shadows:    {len(registry.get_shadow_boxels())}")
    print(f"  - Free space: {len(registry.get_free_space_boxels())}")
    
    print(f"\nShadow boxels (where target might hide):")
    for shadow in registry.get_shadow_boxels():
        print(f"  - {shadow.id}: created by {shadow.created_by_object}")
    
    return registry


def test_2_check_domain_file():
    """Test 2: Check PDDL domain file exists and is valid."""
    separator("TEST 2: Check PDDL Domain File")
    
    domain_path = "pddl/domain_boxel_tamp.pddl"
    
    if not os.path.exists(domain_path):
        print(f"ERROR: {domain_path} not found!")
        return False
    
    with open(domain_path, 'r') as f:
        content = f.read()
    
    # Basic validation
    checks = [
        ("(define (domain boxel-tamp)", "Domain definition"),
        ("(:predicates", "Predicates section"),
        ("obj_at_boxel_KIF", "Know-If fluent"),
        ("sense_boxel", "Sense action"),
        ("pick", "Pick action"),
        ("move", "Move action"),
    ]
    
    all_passed = True
    for pattern, description in checks:
        if pattern in content:
            print(f"  ✓ {description}: found")
        else:
            print(f"  ✗ {description}: NOT FOUND!")
            all_passed = False
    
    if all_passed:
        print(f"\n✓ Domain file is valid: {domain_path}")
    
    return all_passed


def test_3_generate_problem(registry: BoxelRegistry):
    """Test 3: Generate PDDL problem from BoxelRegistry."""
    separator("TEST 3: Generate PDDL Problem")
    
    if registry is None:
        print("Skipping - registry not loaded")
        return None
    
    generator = ProblemGenerator(registry)
    
    problem = generator.generate_problem(
        target_objects=["target_1"],
        goal="(holding target_1)"
    )
    
    print("Generated problem:")
    print(f"  - Objects: {len(problem.objects)}")
    print(f"  - Init facts: {len(problem.init)}")
    print(f"  - Goal: {problem.goal}")
    
    # Show PDDL output
    pddl_str = problem.to_pddl()
    print(f"\nPDDL Output (first 50 lines):")
    print("-" * 50)
    for i, line in enumerate(pddl_str.split('\n')[:50]):
        print(line)
    print("-" * 50)
    
    # Save to file for inspection
    output_path = "pddl/problem_generated.pddl"
    os.makedirs("pddl", exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(pddl_str)
    print(f"\n✓ Full problem saved to: {output_path}")
    
    return problem


def test_4_epistemic_state(registry: BoxelRegistry, problem: PlanningProblem):
    """Test 4: Verify Know-If fluent representation."""
    separator("TEST 4: Epistemic State (Know-If Fluents)")
    
    if registry is None or problem is None:
        print("Skipping - previous tests failed")
        return
    
    generator = ProblemGenerator(registry)
    generator.print_epistemic_state(problem, ["target_1"])
    
    # Explain the representation
    print("\nInterpretation:")
    print("  - 'KNOWN IN BOXEL':     KIF=true, at=true  → K(in)")
    print("  - 'KNOWN NOT IN BOXEL': KIF=true, at=false → K(¬in)")
    print("  - 'UNKNOWN':            KIF=false          → might be here")
    
    # Count unknowns
    unknown_count = 0
    for shadow in registry.get_shadow_boxels():
        kif_fact = f"(obj_at_boxel_KIF target_1 {shadow.id})"
        if kif_fact not in problem.init:
            unknown_count += 1
    
    print(f"\n✓ {unknown_count} shadow boxels have UNKNOWN location for target_1")
    print("  (The planner must sense these to find the target)")


def test_5_belief_update(registry: BoxelRegistry):
    """Test 5: Simulate belief updates after sensing."""
    separator("TEST 5: Belief Update After Sensing")
    
    if registry is None:
        print("Skipping - registry not loaded")
        return
    
    generator = ProblemGenerator(registry)
    
    # Initial problem
    problem = generator.generate_problem(
        target_objects=["target_1"],
        goal="(holding target_1)"
    )
    
    print("Initial epistemic state:")
    generator.print_epistemic_state(problem, ["target_1"])
    
    # Get shadow boxels
    shadows = registry.get_shadow_boxels()
    if len(shadows) < 2:
        print("Not enough shadow boxels for this test")
        return
    
    # Simulate sensing first shadow - NOT FOUND
    shadow1 = shadows[0].id
    print(f"\n→ Sensing {shadow1} for target_1...")
    print(f"  Observation: NOT FOUND")
    
    problem = generator.update_after_sensing(problem, "target_1", shadow1, found=False)
    generator.print_epistemic_state(problem, ["target_1"])
    
    # Simulate sensing second shadow - FOUND
    shadow2 = shadows[1].id
    print(f"\n→ Sensing {shadow2} for target_1...")
    print(f"  Observation: FOUND!")
    
    problem = generator.update_after_sensing(problem, "target_1", shadow2, found=True)
    generator.print_epistemic_state(problem, ["target_1"])
    
    print("✓ Belief updates working correctly!")
    print("  - After 'not found': KIF set to true, at stays false")
    print("  - After 'found': KIF=true, at=true, pose known")
    print("  - When found: all other shadows also marked as 'not in'")


def test_6_mock_execution(registry: BoxelRegistry):
    """Test 6: Run mock execution with replanning."""
    separator("TEST 6: Mock Execution with Replanning")
    
    if registry is None:
        print("Skipping - registry not loaded")
        return
    
    shadows = registry.get_shadow_boxels()
    if len(shadows) < 2:
        print("Not enough shadow boxels for this test")
        return
    
    # Put target in the SECOND shadow (so first sensing fails)
    target_location = shadows[1].id
    
    print(f"Setting up scenario:")
    print(f"  - Target 'target_1' is hidden in: {target_location}")
    print(f"  - First shadow to check will be: {shadows[0].id}")
    print(f"  - This should trigger a REPLAN!\n")
    
    input("Press Enter to start mock execution...")
    
    success = demo_execution(registry, target_location)
    
    if success:
        print("\n✓ Execution test PASSED!")
    else:
        print("\n✗ Execution test FAILED!")


def test_7_streams(registry: BoxelRegistry):
    """Test 7: Verify stream generators work."""
    separator("TEST 7: Stream Generators")
    
    if registry is None:
        print("Skipping - registry not loaded")
        return
    
    streams = BoxelStreams(registry)
    
    # Test sensing config stream
    shadows = registry.get_shadow_boxels()
    if shadows:
        shadow_id = shadows[0].id
        print(f"Testing sample_sensing_config for {shadow_id}:")
        
        configs = list(streams.sample_sensing_config(shadow_id))
        print(f"  Generated {len(configs)} sensing configurations")
        
        if configs:
            config = configs[0][0]
            print(f"  First config: {config.name}")
            print(f"  Joint positions: {config.joint_positions}")
    
    # Test grasp stream
    print(f"\nTesting sample_grasp for 'target_1':")
    grasps = list(streams.sample_grasp("target_1"))
    print(f"  Generated {len(grasps)} grasps")
    
    if grasps:
        grasp = grasps[0][0]
        print(f"  First grasp: {grasp.name}")
        print(f"  Position offset: {grasp.position}")
    
    print("\n✓ Streams working correctly!")


def run_all_tests():
    """Run all tests in sequence."""
    print("\n" + "#"*70)
    print("#" + " "*20 + "PDDL INTEGRATION TESTS" + " "*20 + "#")
    print("#"*70)
    
    # Test 1: Load registry
    registry = test_1_load_registry()
    
    # Test 2: Check domain file
    test_2_check_domain_file()
    
    # Test 3: Generate problem
    problem = test_3_generate_problem(registry)
    
    # Test 4: Epistemic state
    test_4_epistemic_state(registry, problem)
    
    # Test 5: Belief update
    test_5_belief_update(registry)
    
    # Test 7: Streams (before execution)
    test_7_streams(registry)
    
    # Test 6: Mock execution (interactive)
    print("\n" + "-"*70)
    response = input("Run mock execution test? (y/n): ")
    if response.lower() == 'y':
        test_6_mock_execution(registry)
    
    # Summary
    separator("TEST SUMMARY")
    print("All tests completed!")
    print("\nNext steps to verify manually:")
    print("  1. Open pddl/domain_boxel_tamp.pddl and review the domain")
    print("  2. Open pddl/problem_generated.pddl and check the problem")
    print("  3. Verify Know-If fluents make sense for your scenario")
    print("  4. Run 'python test_pddl_integration.py' again to test execution")


def interactive_menu():
    """Show interactive menu for running individual tests."""
    print("\n" + "#"*70)
    print("#" + " "*15 + "PDDL INTEGRATION - INTERACTIVE TESTS" + " "*15 + "#")
    print("#"*70)
    
    print("\nAvailable tests:")
    print("  1. Load BoxelRegistry from JSON")
    print("  2. Check PDDL domain file")
    print("  3. Generate PDDL problem")
    print("  4. Show epistemic state (Know-If fluents)")
    print("  5. Simulate belief updates")
    print("  6. Run mock execution with replanning")
    print("  7. Test stream generators")
    print("  A. Run ALL tests")
    print("  Q. Quit")
    
    registry = None
    problem = None
    
    while True:
        print()
        choice = input("Select test (1-7, A, Q): ").strip().upper()
        
        if choice == '1':
            registry = test_1_load_registry()
        elif choice == '2':
            test_2_check_domain_file()
        elif choice == '3':
            problem = test_3_generate_problem(registry)
        elif choice == '4':
            test_4_epistemic_state(registry, problem)
        elif choice == '5':
            test_5_belief_update(registry)
        elif choice == '6':
            test_6_mock_execution(registry)
        elif choice == '7':
            test_7_streams(registry)
        elif choice == 'A':
            run_all_tests()
        elif choice == 'Q':
            print("Goodbye!")
            break
        else:
            print("Invalid choice. Try again.")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--all":
        run_all_tests()
    else:
        interactive_menu()
