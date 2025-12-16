"""Unit tests for the qdot_tracking method in the WidowX class."""

import unittest
import numpy as np
from unittest.mock import patch
from arm_kinematics.widowx import WidowX


class TestQdotTracking(unittest.TestCase):
    def setUp(self):
        """Create mocked WidowX instance without calling __init__"""
        self.wx = object.__new__(WidowX)
        # Inject necessary attributes.
        self.wx.Kp = 1.0
        self.wx.target_position_world_frame = np.zeros(3)
        self.wx.forward_kinematics = lambda: np.eye(4)  # 4x4 with zero translation

    def _run(self, J, v):
        """Helper function to run qdot_tracking with mocked Jacobian and vdes."""
        J = np.asarray(J, float)
        v = np.asarray(v, float)
        with patch.object(WidowX, "camera_pos_jacob_world_frame", return_value=J):
            # override method under test’s dependency
            self.wx.vdes_ee_world_frame = lambda v=v: v
            return self.wx.qdot_tracking()

    def test_equal_blocks(self):
        """Test the simple case where J has two equal blocks for exact solution."""
        # J = [I | I]
        J = np.hstack([np.eye(3), np.eye(3)])
        v = np.array([1.0, 2.0, -3.0])
        qdot = self._run(J, v)
        expected = 0.5 * np.concatenate([v, v])
        np.testing.assert_allclose(qdot, expected, atol=1e-12)
        np.testing.assert_allclose(J @ qdot, v, atol=1e-12)
        self.assertEqual(qdot.shape, (6,))

    def test_general_full_row_rank(self):
        """Test that solution solves the general full row-rank case."""
        J = np.array(
            [[1, 0, 2, 0, 0, 1], [0, 1, 0, 2, 1, 0], [1, 1, 0, 0, 2, 0]], float
        )
        v = np.array([0.3, -0.7, 1.1])
        qdot = self._run(J, v)
        np.testing.assert_allclose(J @ qdot, v, atol=1e-12)

    def test_zero_jacobian(self):
        """Test that solution can handle poorly-conditioned Jacobian."""
        J = np.zeros((3, 6))
        v = np.array([1.0, -2.0, 3.0])
        qdot = self._run(J, v)
        np.testing.assert_allclose(qdot, np.zeros(6), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
