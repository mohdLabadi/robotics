import json
import numpy as np

import rospy
from cs4750 import utils as utils_cs
from numpy.lib import utils

from geometry_msgs.msg import PoseStamped
import time
import rospkg


class SecurityBot:
    def __init__(self):
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=1)
        self.car_pose = rospy.Subscriber(
            '/car/car_pose', PoseStamped, self.pose_cb)
        self.current_car_pose = None
        self.current_goal = None
        self.goal_ctr = 0
        self.goal_list = []
        self.traversal_complete = False
        # Room variables
        self.room_check_order = []
        self.max_room_visits = {'A':2, 'B':2, 'C':2}
        self.current_room_visits = {'A':0, 'B':0, 'C':0}
        
        self.life = 100
        self.life_ori = self.life
        self.medicine = 0
        self.doctor_time = 200
        self.start_time = time.time()
        rospack = rospkg.RosPack()
        wp_path = rospack.get_path('car_final_project')
        self.submission_file = open(wp_path + "/submission/output.txt", 'w+')
        f = open(wp_path + '/scripts/sequence.json')
        data = json.load(f)
        waypoints = data['sequence']

        for i in range(len(waypoints)):
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = waypoints[i][0]
            goal.pose.position.y = waypoints[i][1]
            goal.pose.orientation = utils_cs.angle_to_quaternion(
                waypoints[i][2])
            self.goal_list.append(goal)

    def room_check(self):
        x = self.current_car_pose.position.x
        y = self.current_car_pose.position.y
        if np.linalg.norm([x - 3.19, y - 1.1]) < 1.1:
            room_id = 'A'
            self.room_check_order.append(room_id)
            self.current_room_visits[room_id]+=1
            if self.current_room_visits[room_id] <= self.max_room_visits[room_id]:
                self.medicine += 50
            else:
                rospy.loginfo(f"Room A visits exceeded maximum ({self.max_room_visits[room_id]}). Not picking up medicine.")

        elif np.linalg.norm([x - 1.8, y - 5.4]) < 1.5:
            room_id = 'B'
            self.room_check_order.append(room_id)
            self.current_room_visits[room_id]+=1
            if self.current_room_visits[room_id] <= self.max_room_visits[room_id]:
                self.doctor_time-=20
            else:
                rospy.loginfo(f"Room B visits exceeded maximum ({self.max_room_visits[room_id]}). Not reducing doctor_time.")
            return

        elif np.linalg.norm([x - 14.4, y - 3.69]) < 1.5:
            room_id = 'C'
            self.room_check_order.append(room_id)
            self.current_room_visits[room_id]+=1
            if self.current_room_visits[room_id] <= self.max_room_visits[room_id]:
                self.medicine += 20
            else:
                rospy.loginfo(f"Room C visits exceeded maximum ({self.max_room_visits[room_id]}). Not picking up medicine.")


        elif np.linalg.norm([x - 9.7, y - 1.2]) < 1.5:
            self.room_check_order.append('O')
            start_time = self.start_time
            end_time = time.time()
            consumed_time = start_time-end_time
            start_time = end_time
            self.life += self.medicine
            self.medicine = 0

        

    def pose_cb(self, msg):
        self.current_car_pose = msg.pose

    @property
    def is_goal_reached(self):
        """
        Check if goal pose has been reached.

        Use self.current_car_pose and self.current_goal to check if current goal
        has been reached. Make use of utils_cs.quaternion_to_angle to obtain the
        theta for current_car_pose and current_goal. If the car position is within
        1.0, set goal_reached to True.

        Returns:
            Bool (True): if goal reached, else False.
        """
        if self.current_goal is None:
            return False

        goal_reached = False
        
        """BEGIN SOLUTION"""
        curr = self.current_car_pose
        goal = self.current_goal

        cx, cy = curr.position.x, curr.position.y
        gx, gy = goal.position.x, goal.position.y

        dist = np.hypot(cx - gx, cy - gy)

        theta_curr = utils_cs.quaternion_to_angle(curr.orientation)
        theta_goal = utils_cs.quaternion_to_angle(goal.orientation)

        raw_diff = theta_curr - theta_goal
        theta_diff = (raw_diff + np.pi) % (2.0 * np.pi) - np.pi
        theta_diff = abs(theta_diff)
        """END SOLUTION"""

        if theta_diff >= 1:
            return False
        
        if dist < 1.0:
            goal_reached = True

        if goal_reached:
            self.room_check()
            return True
        else:
            return False

    @property
    def check_complete(self):
        num_check = len(self.room_check_order)
        rospy.loginfo('Total rooms checked: {} / 3'.format(num_check))
        if num_check == 3:
            return True

    def publish_goal(self):
        if self.goal_ctr == len(self.goal_list) and self.is_goal_reached:
            self.traversal_complete = True
        if self.goal_ctr == 0:
            rospy.loginfo("Send first goal")
            self.goal_pub.publish(self.goal_list[0])
            self.current_goal = self.goal_list[0].pose
            self.goal_ctr += 1

        while self.goal_ctr < len(self.goal_list) and self.is_goal_reached:
            rospy.loginfo("Send goal number {} in the list".format(self.goal_ctr + 1))
            self.goal_pub.publish(self.goal_list[self.goal_ctr])
            self.current_goal = self.goal_list[self.goal_ctr].pose
            self.goal_ctr += 1
