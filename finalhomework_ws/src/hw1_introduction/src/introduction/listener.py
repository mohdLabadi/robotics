import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

# BEGIN QUESTION 4.3
# (import placed above)
# END QUESTION 4.3


def norm_python(data):
    """Compute the norm for each row of a numpy array using Python for loops.

    >>> data = np.array([[3, 4],
    ...                  [5, 12]])
    >>> norm_python(data)
    array([ 7., 17.])
    """
    n, d = data.shape
    norm = np.zeros(n)
    # BEGIN QUESTION 4.1
    for i in range(n):
        total = 0.0
        for j in range(d):
            total += np.absolute(data[i, j])
        norm[i] = total
    # END QUESTION 4.1
    return norm


def norm_numpy(data):
    """Compute the norm for each row of a numpy array using numpy functions.

    >>> data = np.array([[3, 4],
    ...                  [5, 12]])
    >>> norm_numpy(data)
    array([ 7., 17.])
    """
    # BEGIN QUESTION 4.2
    # Use numpy's vectorized L1 (Manhattan) norm
    norm = np.linalg.norm(data, ord=1, axis=1)
    # Alternatively: norm = np.sum(np.abs(data), axis=1)
    # END QUESTION 4.2
    return norm


class PoseListener:
    """Collect car poses."""

    def __init__(self, size=100):
        self.size = size
        self.done = False
        self.storage = []  # a list of (x, y) tuples
        # Create a subscriber for the car pose.
        # BEGIN QUESTION 4.3
        # Use a ROS param so you can set the exact topic name when launching.
        # Default '/car_pose' is a safe placeholder; course instructions ask you
        # to find the exact topic name (rostopic list) and use that when running.
        pose_topic = rospy.get_param("~pose_topic", "/car/car_pose")
        self.subscriber = rospy.Subscriber(pose_topic, PoseStamped, self.callback, queue_size=10)
        # END QUESTION 4.3

    def callback(self, msg):
        """Store the x and y coordinates of the car."""
        header = msg.header
        rospy.loginfo(
            "Received a new message with timestamp " + str(header.stamp.secs) + "(s)"
        )

        # Extract and store the x and y position from the message data
        # BEGIN QUESTION 4.4
        # msg is expected to be a geometry_msgs/PoseStamped
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.storage.append((x, y))
        # END QUESTION 4.4

        if len(self.storage) == self.size:
            self.done = True
            rospy.loginfo("Received enough samples, trying to unsubscribe")
            self.subscriber.unregister()
