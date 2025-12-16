#!/usr/bin/env python
import numpy as np
import rosunit
import rospy
import rostest
import unittest

from cs4750 import utils as utils_cs
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

from car_final_project.security_bot import SecurityBot

class TestSurveillanceBot(unittest.TestCase):
    """
    Goal: test whether SecurityBot.is_goal_reached keeps the required tolerances.
    """

    def setUp(self):
        self.bot = SecurityBot()

    def test_goalreached_samepose(self):
        # Set a fake robot position
        self.bot.current_car_pose = utils_cs.particle_to_pose([0,1,np.pi/2])
        # Set a fake robot goal
        self.bot.current_goal = utils_cs.particle_to_pose([0,1,np.pi/2])
        goal_reached = self.bot.is_goal_reached
        self.assertTrue(
            goal_reached,
            msg="If car pose is identical to goal, is_goal_reached should be true."
        )
    
    def test_goalreached_diffxy(self):
        # Set a fake robot position
        self.bot.current_car_pose = utils_cs.particle_to_pose([0,1,np.pi/2])
        # Set a fake robot goal
        self.bot.current_goal = utils_cs.particle_to_pose([2,1,np.pi/2])
        goal_reached = self.bot.is_goal_reached
        self.assertFalse(
            goal_reached,
            msg="If car position is more than 1.0 away from goal position, is_goal_reached should be false."
        )
    
    def test_goalreached_difftheta(self):
        # Set a fake robot position
        self.bot.current_car_pose = utils_cs.particle_to_pose([0,1,0])
        # Set a fake robot goal
        self.bot.current_goal = utils_cs.particle_to_pose([0,1,1])
        goal_reached = self.bot.is_goal_reached
        self.assertFalse(
            goal_reached,
            msg="If car yaw is more than 1 from goal yaw, is_goal_reached should be false."
        )

if __name__ == "__main__":
    rospy.init_node('test_securitybot')
    rosunit.unitrun("car_final_project", "test_goalreached", TestSurveillanceBot, "--text")
