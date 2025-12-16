"""Unit tests for the camera frame offset matrix in the WidowX class."""
import unittest
import numpy as np
from arm_kinematics.widowx import WidowX

def apply(T, P):
    """Helper function to apply a transform T to a set of points P."""
    P4 = np.c_[P, np.ones(len(P))]
    Q4 = (T @ P4.T).T
    return Q4[:, :3]

class TestCameraOffset(unittest.TestCase):
    def setUp(self):
        # Create mocked WidowX instance without calling __init__
        self.wx = object.__new__(WidowX)

    def test_hardcoded_point_mappings(self):
        """Test that the camera_offset transforms random 3D points correctly."""
        T = self.wx.camera_offset
        P = np.array([
            [0.0, 0.0, 0.0],
            [0.2, -0.1, 0.5],
            [-0.3, 0.4, 0.2],
            [0.1, 0.1, -0.2],
        ])
        expected = np.array([[-0.05,  0.  ,  0.05],
                [-0.55, -0.2 , -0.05],
                [-0.25,  0.3 ,  0.45],
                [ 0.15, -0.1 ,  0.15]])
        
        np.testing.assert_allclose(apply(T, P), expected, atol=1e-12)

    def test_is_rigid_transform(self):
        """Test that the camera_offset property returns a valid rigid transform."""
        M = self.wx.camera_offset
        R = M[:3, :3]
        t = M[:3, 3]
        self.assertTrue(np.allclose(R @ R.T, np.eye(3), atol=1e-12))
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=12)
        np.testing.assert_allclose(t, np.array([-0.05, 0.0, 0.05]), atol=1e-12)
        np.testing.assert_allclose(M[3], np.array([0, 0, 0, 1]), atol=1e-12)

    def test_applies_to_origin(self):
        """Test that the camera_offset property correctly transforms the origin as a homogeneous point."""
        M = self.wx.camera_offset
        p_ee = np.array([0.0, 0.0, 0.0, 1.0])
        p_cam = M @ p_ee
        np.testing.assert_allclose(p_cam, np.array([-0.05, 0.0, 0.05, 1.0]), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
