#!/usr/bin/env python3
"""
Configuration for Semantic Boxels TAMP project.

Sets up paths for PDDLStream and other external dependencies.
"""

import sys
import os
from pathlib import Path

# Project root
PROJECT_ROOT = Path(__file__).parent.resolve()

# PDDLStream location (relative to project root's parent)
PDDLSTREAM_PATH = PROJECT_ROOT.parent / "pddlstream_lib"

def setup_pddlstream():
    """
    Add PDDLStream to Python path if available.
    
    Returns:
        bool: True if PDDLStream is available, False otherwise.
    """
    if PDDLSTREAM_PATH.exists():
        pddlstream_str = str(PDDLSTREAM_PATH)
        if pddlstream_str not in sys.path:
            sys.path.insert(0, pddlstream_str)
        return True
    else:
        print(f"WARNING: PDDLStream not found at {PDDLSTREAM_PATH}")
        print("To install PDDLStream:")
        print(f"  cd {PROJECT_ROOT.parent}")
        print("  git clone https://github.com/caelan/pddlstream.git pddlstream_lib")
        print("  cd pddlstream_lib")
        print("  git submodule update --init downward")
        return False

def check_pddlstream_planner():
    """
    Check if FastDownward planner is built.
    
    Returns:
        bool: True if planner is ready, False if it needs building.
    """
    downward_builds = PDDLSTREAM_PATH / "downward" / "builds"
    if downward_builds.exists() and any(downward_builds.iterdir()):
        return True
    
    print("WARNING: FastDownward planner not built.")
    print("To build (requires CMake and C++ compiler):")
    print(f"  cd {PDDLSTREAM_PATH / 'downward'}")
    print("  python build.py")
    return False

# Auto-setup on import
PDDLSTREAM_AVAILABLE = setup_pddlstream()

if __name__ == "__main__":
    print(f"Project root: {PROJECT_ROOT}")
    print(f"PDDLStream path: {PDDLSTREAM_PATH}")
    print(f"PDDLStream available: {PDDLSTREAM_AVAILABLE}")
    
    if PDDLSTREAM_AVAILABLE:
        planner_ready = check_pddlstream_planner()
        print(f"FastDownward planner built: {planner_ready}")
        
        # Test import
        try:
            import pddlstream
            print(f"PDDLStream imported from: {pddlstream.__file__}")
        except ImportError as e:
            print(f"Failed to import pddlstream: {e}")
