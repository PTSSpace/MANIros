#!/usr/bin/env python

"""
Imports
"""
import unittest
import rospy, rostest
import time

import tf2_ros
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from maniros.msg import EncoderOdometry
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

"""
Classes
"""

class OdmTest(unittest.TestCase):
    def __init__(self, *args):
        super(OdmTest, self).__init__(*args)
        rospy.init_node("test_input", anonymous=True)
        self.state_pub = rospy.Publisher("gazebo/model_states", ModelState, queue_size=10)
        rospy.Subscriber("gazebo/odom", Odometry, self.callback)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)


    def test_transformation_broadcaster(self):
        self.success = False
        # Create model state message
        state_msg = ModelState()
        state_msg.reference_frame = "gazebo/mani/gound_plane"
        state_msg.pose.pose.position.x = 0
        state_msg.pose.pose.position.y = 0
        state_msg.pose.pose.position.z = 0
        state_msg.pose.pose.quaternion.x = 0
        state_msg.pose.pose.quaternion.y = 0
        state_msg.pose.pose.quaternion.z = 0
        state_msg.pose.pose.quaternion.w = 0

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
        self.qx = data.pose.pose.quaternion.x
        self.qy = data.pose.pose.quaternion.y
        self.qz = data.pose.pose.quaternion.z
        self.qw = data.pose.pose.quaternion.w
        rospy.loginfo("JointState message received: reference_frame_id: %s" % (data.reference_frame))
        self.success = True

    def test_odometry_publisher(self):
        self.success = False
        # Create model state message
        state_msg = ModelState()
        state_msg.reference_frame = "gazebo/mani/gound_plane"
        state_msg.pose.pose.position.x = 0
        state_msg.pose.pose.position.y = 0
        state_msg.pose.pose.position.z = 0
        state_msg.pose.pose.quaternion.x = 0
        state_msg.pose.pose.quaternion.y = 0
        state_msg.pose.pose.quaternion.z = 0
        state_msg.pose.pose.quaternion.w = 0

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.state_pub.publish(state_msg)
            rate.sleep()
        self.assertTrue(self.success, 'Odometry message was not sent')


if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_odometry_node', OdmTest)
