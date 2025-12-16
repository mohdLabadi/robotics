#!/usr/bin/env python

import numpy as np
import os
import rosunit
import unittest

from planning import search
from planning.problems import R2Problem


class TestRRTReuse(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        self.permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(self.permissible_region, check_resolution=0.5)

    def _test_rrt_reuse_helper(self, start, old_goal, new_goal, correct_path):
        tree_path = os.path.join(os.path.dirname(__file__), "share", "tree_save_test")

        np.random.seed(111)

        rrt = search.RRTPlanner(
            self.problem, 
            self.permissible_region, 
            save_tree=tree_path,
        )
        rrt.Plan(start, old_goal)

        np.random.seed(222) # Different seed to avoid exact same samples

        rrt = search.RRTPlanner(
            self.problem, 
            self.permissible_region, 
            reuse_tree=tree_path,
        )
        path = rrt.Plan(start, new_goal)
        
        self.assertTrue(
            np.allclose(path, correct_path),
            msg="RRT implementation is incorrect",
        )

    def test_rrt_reuse1(self):
        start = np.array([[1., 6.]])
        old_goal = np.array([[5., 1.]])
        new_goal = np.array([[0., 2.]])
        correct_path = np.array([[1., 6.],
                                [2., 1.],
                                [1., 3.],
                                [0., 2.]])
        
        self._test_rrt_reuse_helper(start, old_goal, new_goal, correct_path)
    
    def test_rrt_reuse2(self):
        start = np.array([[1., 6.]])
        old_goal = np.array([[5., 1.]])
        new_goal = np.array([[3., 3.]])
        correct_path = np.array([[1., 6.],
                                [2., 8.],
                                [4., 8.],
                                [4., 7.],
                                [4., 5.],
                                [3., 4.],
                                [3., 3.]])
        
        self._test_rrt_reuse_helper(start, old_goal, new_goal, correct_path)

if __name__ == "__main__":
    np.random.seed(111)
    rosunit.unitrun("planning", "test_rrt_reuse", TestRRTReuse, "--text")