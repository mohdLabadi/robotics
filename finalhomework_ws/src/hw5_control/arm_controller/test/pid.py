#!/usr/bin/env python

import numpy as np
import os
import rosunit
import unittest

from arm_controller.pid_controller import PIDController


class TestPID(unittest.TestCase):
    def setUp(self):
        self.pid = PIDController(None)

    def test_pid(self):
        self.pid.set_refs(np.array([1, 1, 1, 1, 1, 1]))
        
        self.assertTrue(
            np.allclose(self.pid.get_error(), [-1,-1,-1,-1,-1,-1]),
            msg="Get_error implementation is incorrect",
        )

        self.pid.set_refs(np.array([0.1,0.2,0.3,0.4,0.5,0.6]))
        self.assertTrue(
            np.allclose(self.pid.get_control(), np.array([0.5005, 1.001 , 1.5015, 2.002 , 2.5025, 3.003 ])),
            msg="Get_control implementation is incorrect",
        )
        self.assertTrue(
            np.allclose(self.pid.p_error, np.array([-0.1, -0.2, -0.3, -0.4, -0.5, -0.6])),
            msg="Did not update p_error",
        )
        self.assertTrue(
            np.allclose(self.pid.i_error, np.array([-0.005, -0.01 , -0.015, -0.02 , -0.025, -0.03 ])),
            msg="Did not update i_error",
        )
        self.assertTrue(
            np.allclose(self.pid.d_error, np.array([ -2.,  -4.,  -6.,  -8., -10., -12.])),
            msg="Did not update d_error",
        )
        self.assertTrue(
            np.allclose(self.pid.get_control(), np.array([0.301, 0.602 , 0.903, 1.204 , 1.505, 1.806])),
            msg="Get_control implementation is incorrect",
        )

        

if __name__ == "__main__":
    rosunit.unitrun("arm_controller", "test_pid", TestPID, "--text")

