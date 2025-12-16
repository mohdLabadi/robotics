#!/usr/bin/env python
from __future__ import division
import numpy as np
import rosunit
import unittest

from car_controller.controller import BaseController, compute_position_in_frame


class TestController(unittest.TestCase):
    def setUp(self):
        self.defaults = {
            "frequency": 50,
            "finish_threshold": 0.2,
            "exceed_threshold": 2,
            "distance_lookahead": 0.8,
            "min_speed": 0.5,
        }
        self.controller = BaseController(**self.defaults)

    def test_compute_position_in_frame(self):
        """Test coordinate frame transformations with various scenarios."""
        
        # Test 1: Basic transformation 
        p = np.array([2, 2, np.pi / 2])
        frame = np.array([1, 1, 0])
        p_in_frame = compute_position_in_frame(p, frame)
        self.assertEqual(p_in_frame.shape[0], 2, 
                        msg="compute_position_in_frame should produce a 2D vector")
        np.testing.assert_allclose(p_in_frame, [1, 1], atol=1e-10,
                                  err_msg="Basic transformation failed")

        # Test 2: Frame with 180-degree rotation
        p = np.array([2, 2, np.pi / 2])
        frame = np.array([1, 1, np.pi])
        p_in_frame = compute_position_in_frame(p, frame)
        np.testing.assert_allclose(p_in_frame, [-1, -1], atol=1e-10,
                                  err_msg="180-degree rotation transformation failed")

        # Test 3: 90-degree rotation
        p = np.array([1, 0, 0])
        frame = np.array([0, 0, np.pi/2])
        p_in_frame = compute_position_in_frame(p, frame)
        np.testing.assert_allclose(p_in_frame, [0, -1], atol=1e-10,
                                  err_msg="90-degree rotation transformation failed")

        # Test 4: 270-degree rotation
        p = np.array([1, 0, 0])
        frame = np.array([0, 0, 3*np.pi/2])
        p_in_frame = compute_position_in_frame(p, frame)
        np.testing.assert_allclose(p_in_frame, [0, 1], atol=1e-10,
                                  err_msg="270-degree rotation transformation failed")

        # Test 5: 45-degree rotation
        p = np.array([np.sqrt(2), 0, 0])
        frame = np.array([0, 0, np.pi/4])
        p_in_frame = compute_position_in_frame(p, frame)
        np.testing.assert_allclose(p_in_frame, [1,-1], atol=1e-10,
                                  err_msg="45-degree rotation transformation failed")

        # Test 6: Negative coordinates
        p = np.array([-1, -1, 0])
        frame = np.array([0, 0, -np.pi/4])
        p_in_frame = compute_position_in_frame(p, frame)
        np.testing.assert_allclose(p_in_frame, [0,-np.sqrt(2)], atol=1e-10,
                                  err_msg="Negative coordinates transformation failed")

        # Test 7: Zero position in non-zero frame
        p = np.array([0, 0, 0])
        frame = np.array([5, 3, np.pi/6])
        p_in_frame = compute_position_in_frame(p, frame)
        expected = np.array([-5*np.cos(np.pi/6) - 3*np.sin(np.pi/6), 
                            5*np.sin(np.pi/6) - 3*np.cos(np.pi/6)])
        np.testing.assert_allclose(p_in_frame, expected, atol=1e-10,
                                  err_msg="Zero position transformation failed")

        # Test 8: Small angles (numerical stability)
        p = np.array([1, 0, 0])
        frame = np.array([0, 0, 0.001])
        p_in_frame = compute_position_in_frame(p, frame)
        expected = np.array([np.cos(0.001), -np.sin(0.001)])
        np.testing.assert_allclose(p_in_frame, expected, atol=1e-10,
                                  err_msg="Small angle transformation failed")

        # Test 9: Large angles
        p = np.array([1, 0, 0])
        frame = np.array([0, 0, 2*np.pi + np.pi/4])
        p_in_frame = compute_position_in_frame(p, frame)
        expected = np.array([np.cos(np.pi/4), -np.sin(np.pi/4)])
        np.testing.assert_allclose(p_in_frame, expected, atol=1e-10,
                                  err_msg="Large angle transformation failed")

        # Test 10: Real-world car following scenario
        car_pos = np.array([10, 5, np.pi/6]) 
        waypoint = np.array([15, 8, np.pi/4]) 
        p_in_frame = compute_position_in_frame(car_pos, waypoint)

        # Test 11: Identity transformation (point in its own frame)
        p = np.array([3, 4, np.pi/3])
        frame = np.array([3, 4, np.pi/3])
        p_in_frame = compute_position_in_frame(p, frame)
        np.testing.assert_allclose(p_in_frame, [0, 0], atol=1e-10,
                                  err_msg="Identity transformation failed")

        # Test 12: Translation only (no rotation)
        p = np.array([5, 7, 0])
        frame = np.array([2, 3, 0])
        p_in_frame = compute_position_in_frame(p, frame)
        np.testing.assert_allclose(p_in_frame, [3, 4], atol=1e-10,
                                  err_msg="Translation-only transformation failed")

        # Test 13: Rotation only (no translation)
        p = np.array([1, 0, 0])
        frame = np.array([0, 0, np.pi/6])
        p_in_frame = compute_position_in_frame(p, frame)
        expected = np.array([np.cos(np.pi/6), -np.sin(np.pi/6)])
        np.testing.assert_allclose(p_in_frame, expected, atol=1e-10,
                                  err_msg="Rotation-only transformation failed")
        
        # Test 14: Complex transformation with both rotation and translation
        p = np.array([7, 3, np.pi/3])
        frame = np.array([2, 1, np.pi/4])
        p_in_frame = compute_position_in_frame(p, frame)
        expected = np.array([7*np.sqrt(2)/2, -3*np.sqrt(2)/2])
        np.testing.assert_allclose(p_in_frame, expected, atol=1e-10,
                                  err_msg="Complex transformation with both rotation and translation failed")

    def test_get_reference_index_on_path(self):
        straight_path_xytv = np.array([[x, 0, 0, 1] for x in range(10)], dtype=np.float)
        pose_index = 4
        pose = straight_path_xytv[pose_index, :3]
        reference_index = self.controller.get_reference_index(
            pose, straight_path_xytv, self.controller.distance_lookahead
        )

        self.assertGreater(
            reference_index,
            pose_index,
            msg="Reference index should be ahead of the current state",
        )

        distance = np.linalg.norm(straight_path_xytv[reference_index, :2] - pose[:2])
        # States on the straight path differ by a distance of 1, so the distance
        # to the reference state should be within 1 of the expected lookahead.
        self.assertLess(
            abs(distance - self.controller.distance_lookahead),
            1.0,
            msg="There is a reference index that would match the lookahead better",
        )

    def test_get_reference_index_near_path(self):
        straight_path_xytv = np.array([[x, 0, 0, 1] for x in range(10)], dtype=np.float)
        closest_index = 4
        pose = straight_path_xytv[closest_index, :3]
        pose += np.array([0.15, 0, 0])
        reference_index = self.controller.get_reference_index(
            pose, straight_path_xytv, self.controller.distance_lookahead
        )

        self.assertGreaterEqual(
            reference_index,
            closest_index,
            msg="Reference index should be ahead of the nearest state",
        )

        distance = np.linalg.norm(straight_path_xytv[reference_index, :2] - pose[:2])
        # States on the straight path differ by a distance of 1, so the distance
        # to the reference state should be within 1 of the expected lookahead.
        self.assertLess(
            abs(distance - self.controller.distance_lookahead),
            1.0,
            msg="There is a reference index that would match the lookahead better",
        )

    def test_get_reference_index_near_dense_path(self):
        dense_path_xytv = np.zeros((100, 4))
        dense_path_xytv[:, 0] = np.linspace(0, 20, 100)
        dense_path_xytv[:, 3] = 1
        closest_index = 30
        pose = dense_path_xytv[closest_index, :3]
        pose += np.array([0.01, 0, 0])
        reference_index = self.controller.get_reference_index(
            pose, dense_path_xytv, self.controller.distance_lookahead
        )

        self.assertGreaterEqual(
            reference_index,
            closest_index,
            msg="Reference index should be ahead of the nearest state",
        )

        distance = np.linalg.norm(dense_path_xytv[reference_index, :2] - pose[:2])
        # States on the dense path differ by a distance of ~0.2, so the distance
        # to the reference state should be within 0.2 of the expected lookahead.
        self.assertLess(
            abs(distance - self.controller.distance_lookahead),
            0.21,
            msg="There is a reference index that would match the lookahead better",
        )


if __name__ == "__main__":
    rosunit.unitrun("control", "test_controller", TestController, "--text")