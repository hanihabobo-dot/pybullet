"""
Cell Merger for Convex Free Space Optimization.

This module merges adjacent free space boxels into larger convex regions,
reducing the total number of boxels while maintaining accurate space representation.
"""

import numpy as np
from typing import List, Tuple, Set, Optional
from dataclasses import dataclass

from boxel_types import Boxel


class CellMerger:
    """
    Merges adjacent free space cells into larger convex regions.
    
    Uses a greedy algorithm to iteratively merge pairs of adjacent boxels
    that share a common face and have compatible dimensions.
    """
    
    def __init__(self, tolerance: float = 1e-4):
        """
        Initialize the cell merger.
        
        Args:
            tolerance: Floating point tolerance for dimension comparisons
        """
        self.tolerance = tolerance
    
    def merge_free_space(self, free_boxels: List[Boxel], max_iterations: int = 100) -> List[Boxel]:
        """
        Merge adjacent free space boxels into larger convex regions.
        
        Args:
            free_boxels: List of free space boxels to merge
            max_iterations: Maximum number of merge passes to perform
            
        Returns:
            List of merged free space boxels (fewer, larger boxes)
        """
        if not free_boxels:
            return []
        
        current_boxels = list(free_boxels)
        
        for iteration in range(max_iterations):
            merged_boxels, num_merges = self._merge_pass(current_boxels)
            
            if num_merges == 0:
                # No more merges possible
                break
            
            current_boxels = merged_boxels
            print(f"  Merge iteration {iteration + 1}: {num_merges} merges, {len(current_boxels)} boxels remaining")
        
        print(f"  Merging complete: {len(free_boxels)} -> {len(current_boxels)} boxels")
        return current_boxels
    
    def _merge_pass(self, boxels: List[Boxel]) -> Tuple[List[Boxel], int]:
        """
        Perform one pass of merging adjacent boxels.
        
        Args:
            boxels: List of boxels to process
            
        Returns:
            Tuple of (merged boxels list, number of merges performed)
        """
        if len(boxels) <= 1:
            return boxels, 0
        
        merged = [False] * len(boxels)
        result = []
        num_merges = 0
        
        # Try to merge each pair of boxels
        for i in range(len(boxels)):
            if merged[i]:
                continue
            
            best_merge = None
            best_j = -1
            
            for j in range(i + 1, len(boxels)):
                if merged[j]:
                    continue
                
                merged_boxel = self._try_merge(boxels[i], boxels[j])
                if merged_boxel is not None:
                    # Prefer merges that create the most compact result
                    if best_merge is None:
                        best_merge = merged_boxel
                        best_j = j
                    else:
                        # Compare compactness (prefer merges along longest axis)
                        if self._merge_quality(merged_boxel) > self._merge_quality(best_merge):
                            best_merge = merged_boxel
                            best_j = j
            
            if best_merge is not None:
                result.append(best_merge)
                merged[i] = True
                merged[best_j] = True
                num_merges += 1
            else:
                result.append(boxels[i])
        
        return result, num_merges
    
    def _try_merge(self, boxel_a: Boxel, boxel_b: Boxel) -> Optional[Boxel]:
        """
        Try to merge two boxels if they share a common face.
        
        Two boxels can be merged if:
        1. They share a common face (adjacent with touching boundaries)
        2. The shared face has the same dimensions
        3. They are aligned on the non-merge axes
        
        Args:
            boxel_a: First boxel
            boxel_b: Second boxel
            
        Returns:
            Merged boxel if merge is possible, None otherwise
        """
        a_min, a_max = self._get_bounds(boxel_a)
        b_min, b_max = self._get_bounds(boxel_b)
        
        # Check each axis for potential merge
        for axis in range(3):
            other_axes = [i for i in range(3) if i != axis]
            
            # Check if boxels are aligned on the other two axes
            aligned = True
            for oa in other_axes:
                if not (self._approx_equal(a_min[oa], b_min[oa]) and 
                        self._approx_equal(a_max[oa], b_max[oa])):
                    aligned = False
                    break
            
            if not aligned:
                continue
            
            # Check if boxels are adjacent along this axis
            # Case 1: boxel_a is before boxel_b
            if self._approx_equal(a_max[axis], b_min[axis]):
                # Merge: new box spans from a_min to b_max on this axis
                new_min = a_min.copy()
                new_max = b_max.copy()
                return self._create_merged_boxel(new_min, new_max, boxel_a)
            
            # Case 2: boxel_b is before boxel_a
            if self._approx_equal(b_max[axis], a_min[axis]):
                # Merge: new box spans from b_min to a_max on this axis
                new_min = b_min.copy()
                new_max = a_max.copy()
                return self._create_merged_boxel(new_min, new_max, boxel_a)
        
        return None
    
    def _get_bounds(self, boxel: Boxel) -> Tuple[np.ndarray, np.ndarray]:
        """Get min and max bounds of a boxel."""
        return boxel.center - boxel.extent, boxel.center + boxel.extent
    
    def _approx_equal(self, a: float, b: float) -> bool:
        """Check if two floats are approximately equal."""
        return abs(a - b) < self.tolerance
    
    def _create_merged_boxel(self, min_pt: np.ndarray, max_pt: np.ndarray, 
                             template: Boxel) -> Boxel:
        """Create a new boxel from min/max bounds."""
        center = (min_pt + max_pt) / 2.0
        extent = (max_pt - min_pt) / 2.0
        
        return Boxel(
            center=center,
            extent=extent,
            object_name="free_space_merged",
            is_occluded=False,
            is_shadow=False
        )
    
    def _merge_quality(self, boxel: Boxel) -> float:
        """
        Calculate merge quality score (higher is better).
        
        Prefers more cubic shapes over elongated ones.
        """
        # Use ratio of min to max extent as quality metric
        min_ext = np.min(boxel.extent)
        max_ext = np.max(boxel.extent)
        if max_ext < self.tolerance:
            return 0.0
        return min_ext / max_ext


def merge_free_space_cells(free_boxels: List[Boxel], tolerance: float = 1e-4, 
                           max_iterations: int = 100) -> List[Boxel]:
    """
    Convenience function to merge free space cells.
    
    Args:
        free_boxels: List of free space boxels to merge
        tolerance: Floating point tolerance for dimension comparisons
        max_iterations: Maximum number of merge passes
        
    Returns:
        List of merged free space boxels
    """
    merger = CellMerger(tolerance=tolerance)
    return merger.merge_free_space(free_boxels, max_iterations=max_iterations)
