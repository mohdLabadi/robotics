# tests/test_vdes_ee_world_frame.py
import unittest
import numpy as np

from arm_kinematics.widowx import WidowX

def make_T(pos_xyz):
    """Return a 4x4 homogeneous transform with identity rotation and given translation."""
    T = np.eye(4)
    T[:3, 3] = np.asarray(pos_xyz, dtype=float)
    return T

class TestVdesEEWorldFrame(unittest.TestCase):
    def setUp(self):
        """Create mocked WidowX instance without calling __init__"""
        self.wx = object.__new__(WidowX)

    def _configure(self, current_pos, target_pos, kp):
        """Helper function to mock the state of the WidowX instance."""
        self.wx.forward_kinematics = lambda: make_T(current_pos)
        # Inject gain and target
        self.wx.Kp = float(kp)
        self.wx.target_position_world_frame = np.asarray(target_pos, dtype=float)

    def test_basic_cases(self):
        """Test various cases of current position, target position, and gain."""
        cases = [
            # zero error
            ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0],  1.0, [0.0, 0.0, 0.0]),
            # positive error
            ([0.0, 0.0, 0.0], [0.4, 0.1, 0.2],  1.0, [0.4, 0.1, 0.2]),
            # negative error
            ([0.5, 0.5, 0.5], [0.2, 0.4, 0.1],  1.0, [-0.3, -0.1, -0.4]),
            # scalar gain not 1
            ([0.1, 0.0, 0.2], [0.3, 0.1, 0.5],  2.5, [0.5, 0.25, 0.75]),
            # zero gain
            ([0.4, 0.0, -0.1], [1.0, 1.0, 1.0], 0.0, [0.0, 0.0, 0.0]),
        ]
        for current, target, kp, expected in cases:
            with self.subTest(current=current, target=target, kp=kp):
                self._configure(current, target, kp)
                v = self.wx.vdes_ee_world_frame()
                np.testing.assert_allclose(v, np.asarray(expected), atol=1e-12)
                # shape invariant
                self.assertEqual(v.shape, (3,))

if __name__ == "__main__":
    unittest.main()
