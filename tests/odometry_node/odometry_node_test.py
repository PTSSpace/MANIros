#!/usr/bin/env python

"""
Imports
"""
import unittest
import rospy, rostest
import time

import tf2_ros
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
        self.enc_pub = rospy.Publisher("encoder_odometry", EncoderOdometry, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.callback)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        self.success = False

    def test_transformation_broadcaster(self):
        self.success = False
        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.success = True
            try:
                data = self.tfBuffer.lookup_transform('base_link', 'odom', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.success = False
                rate.sleep()
                continue

    def callback(self, data):
        # Get encoder values from encoder_odometry topic
        self.pose = data.pose.pose
        self.twist = data.twist.twist
        rospy.loginfo("Odometry message received: header_frame_id: %s child_frame_id: %s" % (data.header.frame_id, data.child_frame_id))
        self.success = True

    def test_odometry_publisher(self):
        self.success = False
        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            rate.sleep()
        self.assertTrue(self.success, 'Odometry message was not sent')

    def test_translation_odometry(self):
        self.success = False
        # Create mock rover translation message
        enc_msg = EncoderOdometry()
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [100, 100, 100, 100]               # Drive encoder pulses
        enc_msg.steer_pulses       = [0, 0, 0, 0]                       # Steer encoder pulses
        enc_msg.drive_revolutions  = [1, 1, 1, 1]                       # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [0, 0, 0, 0]                       # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                       # Steer encoder velocity [pulses per second]
        # Publish encoder message
        self.enc_pub.publish(enc_msg)

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.enc_pub.publish(enc_msg)
            rate.sleep()
        # Check for accurate odometry
        if (self.pose.position == Point(0, 0, 0)
            or self.pose.orientation != Quaternion(0 ,0, 0, 1)):
            self.success == False

        self.assertTrue(self.success, 'False odometry interpretation')

    def test_drive_odometry(self):
        self.success = False
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [100, 100, 100, 100]               # Drive encoder pulses
        enc_msg.steer_pulses       = [0, 0, 0, 0]                       # Steer encoder pulses
        enc_msg.drive_revolutions  = [1, 1, 1, 1]                       # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [100, 100, 100, 100]               # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                       # Steer encoder velocity [pulses per second]
        # Publish encoder message
        self.enc_pub.publish(enc_msg)

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.enc_pub.publish(enc_msg)
            rate.sleep()
        # Check for accurate odometry
        if (self.twist.linear == Vector3(0, 0, 0)
            or self.twist.angular != Vector3(0 ,0, 0)):
            self.success == False

        self.assertTrue(self.success, 'False odometry interpretation')

    def test_roation_odometry(self):
        self.success = False
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [100, 100, 100, 100]               # Drive encoder pulses
        enc_msg.steer_pulses       = [100, 100, 100, 100]               # Steer encoder pulses
        enc_msg.drive_revolutions  = [1, 1, 1, 1]                       # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [100, 100, 100, 100]               # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                       # Steer encoder velocity [pulses per second]
        # Publish encoder message
        self.enc_pub.publish(enc_msg)

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.enc_pub.publish(enc_msg)
            rate.sleep()
        # Check for accurate odometry
        if (self.twist.angular == Vector3(0 ,0, 0)
            or self.pose.orientation == Quaternion(0 ,0, 0, 1)):
            self.success == False

        self.assertTrue(self.success, 'False odometry interpretation')


if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_odometry_node', OdmTest)
