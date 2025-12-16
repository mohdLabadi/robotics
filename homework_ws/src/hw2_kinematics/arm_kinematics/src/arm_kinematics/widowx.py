#!/usr/bin/env python3
import geometry_msgs.msg
import roboticstoolbox as rtb
import rospkg
import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from spatialmath import SE3
from tf.transformations import quaternion_from_matrix, translation_from_matrix
from tf2_ros import TransformException
from visualization_msgs.msg import Marker
from urdf_parser_py.urdf import Robot

class WidowX:

    def __init__(self, urdf_path: str) -> None:
        """Constructor for WidowX class"""
        rospy.loginfo("Initialize Joint State subscriber")
        
        # Initialize internals.
        self.joint_angles = np.zeros(6)
        self.robot = rtb.Robot.URDF(urdf_path)
        self.target_position_world_frame = np.array([0.4, 0.0, 0.4])
        
        # Create ROS publishers.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.vdes_pub = rospy.Publisher("vdes_centering", Marker, queue_size=10)
        self.vactual_pub = rospy.Publisher("vactual_centering", Marker, queue_size=10)
        self.qdot_pub = rospy.Publisher("/wx250s/joint_commands", JointState, queue_size=10)
        
        # Read joint names from URDF on parameter server.
        urdf_xml = str(rospy.get_param("wx250s/robot_description"))
        robot = Robot.from_xml_string(urdf_xml)
        self.joint_names = [j.name for j in robot.joints if j.type != 'fixed']

        # Create ROS subscribers and timers.
        self.joint_subscriber = rospy.Subscriber('/wx250s/joint_states', JointState, self.joint_callback)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration.from_sec(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_update_timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.update_target_callback)
        self.qdot_timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.qdot_publish_callback)

        # Read gain from parameter server.
        self.Kp: float = float(rospy.get_param("Kp"))

    def update_target_callback(self, event: rospy.timer.TimerEvent) -> None:
        """Callback function to query the TF tree for the target position."""
        try:
            transform = self.tf_buffer.lookup_transform("wx250s/base_link", "target_frame", rospy.Time(0), rospy.Duration(1.0))
            self.target_position_world_frame = np.array([transform.transform.translation.x,
                                                         transform.transform.translation.y,
                                                         transform.transform.translation.z])
        except tf2_ros.TransformException as ex:
            pass # Handle case where transform is not being published.

    def joint_callback(self, msg: JointState) -> None:
        """Callback function for joint state updates"""
        
        # Store joint angles from message.
        self.joint_angles = np.array(msg.position)
        self.broadcast_ee_tf()

    def qdot_publish_callback(self, event: rospy.timer.TimerEvent) -> None:
        """Callback function to publish qdot commands."""
        qdot_msg = JointState()
        qdot_msg.header.stamp = rospy.Time.now()
        qdot_msg.name = self.joint_names

        # Create full qdot, adding zeros for gripper joints
        qdot_full = np.zeros(len(self.joint_names))
        qdot_full[:6] = self.qdot_tracking()
        qdot_msg.velocity = qdot_full.tolist()
        self.qdot_pub.publish(qdot_msg)


    def broadcast_ee_tf(self) -> None:
        """Helper function to publish a TF transform based on the current joint angles"""
        transform = self.forward_kinematics()
        q = quaternion_from_matrix(transform)
        translation = translation_from_matrix(transform)

        msg = geometry_msgs.msg.TransformStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "wx250s/base_link"
        msg.child_frame_id = "camera_frame"

        msg.transform.translation.x = translation[0]
        msg.transform.translation.y = translation[1]
        msg.transform.translation.z = translation[2]
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(msg)

    @property
    def camera_offset(self) -> np.ndarray:
        """Helper function implementing the camera frame transformation."""
        # BEGIN QUESTION 2.1 ALT="return np.zeros((4,4), dtype=float)"
        
        translation = np.array([-0.05, 0.0, 0.05])
        
        R = np.array([
            [ 0,  0, -1],  
            [-1,  0,  0], 
            [ 0,  1,  0]  
        ])
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation
        
        return T
        
        # END QUESTION 2.1

    def forward_kinematics(self) -> np.ndarray:
        """Peforms forward kinematics to compute the camera-to-base transform."""
        return self.robot.fkine(self.joint_angles, end="wx250s/ee_gripper_link", tool=SE3(self.camera_offset)).A

    def camera_pos_jacob_world_frame(self) -> np.ndarray:
        """Computes the position Jacobian of the camera position in the world frame."""
        return self.robot.jacob0(self.joint_angles, end="wx250s/ee_gripper_link", tool=SE3(self.camera_offset))[:3, :6]

    def vdes_ee_world_frame(self) -> np.ndarray:
        """Desired end-effector velocity in the world frame."""
        # BEGIN QUESTION 2.2 ALT="return np.zeros((3,), dtype=float)"
        
        T_current = self.forward_kinematics()  
        p_current = T_current[:3, 3]  
        vdes = self.Kp * (self.target_position_world_frame - p_current)
        
        return vdes
        
        # END QUESTION 2.2
        
    def qdot_tracking(self) -> np.ndarray:
        """Compute joint velocities to achieve desired end-effector velocity in world frame."""
        # BEGIN QUESTION 2.3 ALT="return np.zeros((6,), dtype=float)"
        
        J = self.camera_pos_jacob_world_frame()
        vdes = self.vdes_ee_world_frame()
        qdot = np.linalg.pinv(J) @ vdes
        
        return qdot
            
        # END QUESTION 2.3
