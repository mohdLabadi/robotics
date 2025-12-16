#!/usr/bin/env python
from __future__ import division
import matplotlib.pyplot as plt
import numpy as np
import rospy
import rostest
import unittest

from geometry_msgs.msg import PoseStamped

from cs4750 import utils

# Collector-specific imports
from cs4750.collector import SynchronizedMessageCollector
import message_filters
import rospy

try:
    import queue
except ImportError:
    import Queue as queue  # noqa


class SynchronizedMessageCollectorPF(SynchronizedMessageCollector):
    """Collect approximately synchronized messages published to a set of ROS topics, 
    but with special handling for this test to calculate running mean position error."""

    def __init__(self, topics, msg_types, queue_size=10, sync_tolerance=0.1):
        super().__init__(topics, msg_types, queue_size, sync_tolerance)
        self.running_sum_position_error = 0
        self.running_mean_position_error = []
        # Test-specific variables
        self.pos_threshold = 9
        self.test_failed = False

    def start(self, duration=None, max_msgs=-1):
        self.msgs = queue.Queue(maxsize=max_msgs)

        self.subscibers = []
        for topic, msg_type in zip(self._topics, self._msg_types):
            sub = message_filters.Subscriber(topic, msg_type)
            self.subscibers.append(sub)

        self.__ts = message_filters.ApproximateTimeSynchronizer(
            self.subscibers,
            self._queue_size,
            self._sync_tolerance,
            allow_headerless=True,
        )
        self.__ts.registerCallback(self._msg_callback)

        if duration:
            r = rospy.Rate(50)
            start_time = rospy.get_time()
            # Terminate simulation if the test fails.
            while not self.test_failed and (rospy.get_time() - start_time) < duration:
                r.sleep()
            return self.stop()
        
    def _msg_callback(self, *args):
        """
        Receives synchronized pairs of estimated + gt pose messages,
        and checks to see whether the running mean position error exceeds a certain threshold.
        """
        self.msgs.put(args)
        # Compute the current mean position error.
        ## Convert synchronized PoseStamped messages to state vectors
        msgs_list = list(self.msgs.queue)
        estimates, references = [
            [utils.pose_to_particle(m.pose) for m in ms] for ms in zip(*msgs_list)
        ]
        estimates = np.array(estimates)
        references = np.array(references)

        position_error = np.linalg.norm(estimates[-1] - references[-1])
        self.running_sum_position_error += position_error.item()
        mean_position_error = self.running_sum_position_error/len(msgs_list)
        # If running mean position error exceeds a threshold, test should fail.
        if mean_position_error >= self.pos_threshold:
            self.test_failed = True
        # Save current mean position error.
        self.running_mean_position_error.append(mean_position_error)

class TestParticleFilterServoing(unittest.TestCase):
    def setUp(self):
        pass

    def _test_particle_filter(self):
        reference_pose_topic = "gt_pose"
        plot = bool(rospy.get_param("~plot", False))

        collector = SynchronizedMessageCollectorPF(
            ["/estimated_point", reference_pose_topic],
            [PoseStamped, PoseStamped],
        )
        try:
            rospy.sleep(5)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # We expect time to jump back because the test restarts the bag
            pass
        msgs = collector.start(duration=15)
        self.assertFalse(collector.test_failed,
                         msg=f"The running mean position error exceeded {collector.pos_threshold} pixels.")
        self.assertGreaterEqual(
            len(msgs),
            50,
            msg="The test didn't receive enough messages to be able to compare "
            "the particle filter estimate with the ground truth.",
        )
        sentinel = object()

        # Convert synchronized PoseStamped messages to state vectors
        estimates, references = [
            [utils.pose_to_particle(m.pose) for m in ms] for ms in zip(*msgs)
        ]
        estimates = np.array(estimates)
        references = np.array(references)

        position_error = utils.estimation_error_pose_only(
            estimates, references
        )
        rospy.logwarn("Final mean position error: {}".format(np.mean(position_error)))

        plt.xlabel("Time")
        plt.ylabel("Error (Pixels)")
        plt.plot(position_error, label="Position error")
        plt.plot(collector.running_mean_position_error, c="r", label="Running mean position error")
        plt.legend()
        plt.savefig("./position_error.png")
        plt.show()
        if plot:
            plt.xlabel("x")
            plt.ylabel("y")
            plt.plot(estimates[:, 0], estimates[:, 1], c="r", label="Estimated State")
            plt.plot(references[:, 0], references[:, 1], c="g", label="Ground Truth")
            plt.legend()
            plt.gca().set_aspect(aspect=1.0)
            plt.show()
            plt.savefig("./figure.png")

        pos_threshold = 6
        self.assertLess(
            np.mean(position_error),
            pos_threshold,
            msg="The final mean position error should be less than {} pixels".format(
                pos_threshold
            ),
        )


if __name__ == "__main__":
    rospy.init_node("test_particle_filter_tracking")
    # The xml report will use the method's name, so we have to manually
    # mangle it with the bag name to ensure all entries appear
    setattr(
        TestParticleFilterServoing,
        "test_object_tracking_particle_filter",
        lambda self: self._test_particle_filter(),
    )
    rostest.rosrun(
        "arm_state_estimation", "test_object_tracking_particle_filter", TestParticleFilterServoing
    )
