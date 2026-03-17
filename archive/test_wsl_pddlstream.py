#!/usr/bin/env python3
"""Test PDDLStream import in WSL."""
import os
import sys
sys.path.insert(0, os.environ.get('PDDLSTREAM_PATH', '../pddlstream_lib'))

try:
    from pddlstream.algorithms.meta import solve
    print("SUCCESS: PDDLStream solve() imported!")
    print(f"solve function: {solve}")
except Exception as e:
    print(f"FAILED: {e}")
    import traceback
    traceback.print_exc()
