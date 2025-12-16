#!/usr/bin/env python
import rospy
import rostest
import unittest

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry


class TestParticleFilterPublishers(unittest.TestCase):
    def test_inferred_pose_publisher(self):
        print("Waiting for message on /car/particle_filter/inferred_pose...")
        msg = rospy.wait_for_message("/car/particle_filter/inferred_pose", PoseStamped, timeout=10)
        print("Received message on /car/particle_filter/inferred_pose")
        self.assertIsInstance(msg, PoseStamped)

    def test_particles_publisher(self):
        print("Waiting for message on /car/particle_filter/particles...")
        msg = rospy.wait_for_message("/car/particle_filter/particles", PoseArray, timeout=10)
        print("Received message on /car/particle_filter/particles")
        self.assertIsInstance(msg, PoseArray)

    def test_odom_publisher(self):
        print("Waiting for message on /car/particle_filter/odom...")
        msg = rospy.wait_for_message("/car/particle_filter/odom", Odometry, timeout=10)
        print("Received message on /car/particle_filter/odom")
        self.assertIsInstance(msg, Odometry)


if __name__ == "__main__":
    print("Starting test for particle filter publishers...")
    rospy.init_node("test_particle_filter_publishers")
    print("Node initialized, running tests...")
    rostest.rosrun("car_state_estimation", "test_particle_filter_publishers", TestParticleFilterPublishers)
