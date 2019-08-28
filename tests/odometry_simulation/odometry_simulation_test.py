#!/usr/bin/env python

"""
Imports
"""
import unittest
import rospy, rostest
import time

import tf2_ros
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf_conversions

"""
Classes
"""

class OdmTest(unittest.TestCase):
    def __init__(self, *args):
        super(OdmTest, self).__init__(*args)
        rospy.init_node("test_input", anonymous=True)
        self.state_pub = rospy.Publisher("gazebo/model_states", ModelStates, queue_size=10)
        rospy.Subscriber("gazebo/odom", Odometry, self.callback)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.success = False


    def test_transformation_broadcaster(self):
        self.success = False
        # Create model state message
        x = 0
        y = 0
        rz = 0
        vx = 0
        vy = 0
        wrz = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, rz)
        state_msg = ModelStates()
        state_msg.name = ["ground_plane", "robot"]
        state_msg.pose = [Pose(Point(0, 0, 0), Quaternion(0,0,0,1)), Pose(Point(x, y, 0), Quaternion(*q))]
        state_msg.twist = [Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)), Twist(Vector3(vx, vy, 0), Vector3(0, 0, wrz))]

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.state_pub.publish(state_msg)
            self.success = True
            try:
                data = self.tfBuffer.lookup_transform('gazebo/mani/gound_plane', 'odom', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.success = False
                rate.sleep()
                continue

    def callback(self, data):
        # Get model state values from simulation
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w
        self.vx = data.twist.twist.linear.x
        self.vy = data.twist.twist.linear.y
        self.vz = data.twist.twist.linear.z
        self.rx = data.twist.twist.angular.x
        self.ry = data.twist.twist.angular.y
        self.rz = data.twist.twist.angular.z
        rospy.loginfo("Odometry message received: header_frame_id: %s child_frame_id: %s" % (data.header.frame_id, data.child_frame_id))
        self.success = True

    def test_odometry_publisher(self):
        self.success = False
        # Create model state message
        x = 0
        y = 0
        rz = 0
        vx = 0
        vy = 0
        wrz = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, rz)
        state_msg = ModelStates()
        state_msg.name = ["ground_plane", "robot"]
        state_msg.pose = [Pose(Point(0, 0, 0), Quaternion(0,0,0,1)), Pose(Point(x, y, 0), Quaternion(*q))]
        state_msg.twist = [Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)), Twist(Vector3(vx, vy, 0), Vector3(0, 0, wrz))]

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.state_pub.publish(state_msg)
            rate.sleep()
        self.assertTrue(self.success, 'Odometry message was not sent')


if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_odometry_node', OdmTest)
