#!/usr/bin/env python

"""
Imports
"""
import unittest
import rospy, rostest
import time
import math

import tf2_ros
from maniros.srv import OdometryReset
from nav_msgs.msg import Odometry
from maniros.msg import EncoderOdometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

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

        # Get ros parameters
        self.rover_length       = rospy.get_param("/rover_length")                  # Rover length [m]
        self.rover_width        = rospy.get_param("/rover_width")                   # Rover width [m]
        self.DRIVE_ENC_PPR      = rospy.get_param("/drive_enc_ppr")                 # Drive encoder pulses per revolution
        self.STEER_ENC_PPR      = rospy.get_param("/steer_enc_ppr")                 # Steer encoder pulses per revolution
        self.MAX_VEL            = rospy.get_param("/max_vel")                       # Maximal wheel velocity [rad/s]
        self.MAX_ORT            = rospy.get_param("/max_ort")                       # Maximal wheel orientation [rad]
        self.wheel_diameter     = rospy.get_param("/wheel_diameter")                # Rover wheel diameter [m]
        self.success = False

        self.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        self.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    def roundValue(self, value, digit = 10):
        """
        Rounds values to avoid test failures.
        the specified digit.
        :param value: A signed value
        :param digit: Specifies the number of floating point to round to
        """
        return round(value, digit)

    def reset_odometry(self):
        set = False
        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        rospy.wait_for_service("reset_odometry")
        while not rospy.is_shutdown() and not set and time.time() < timeout_t:
            try:
                reset_odometry = rospy.ServiceProxy('reset_odometry', OdometryReset)
                set = reset_odometry(True)
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s" % e)
                rate.sleep()
                continue
        return set

    def test_01_reset_service(self):
        success = self.reset_odometry()
        self.assertTrue(success, 'Odometry service reset failed')

    def test_02_transformation_broadcaster(self):
        success = False
        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not success and time.time() < timeout_t:
            success = True
            try:
                data = self.tfBuffer.lookup_transform('base_link', 'odom', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                success = False
                rate.sleep()
                continue
        self.assertTrue(self.success, 'Transform message was not sent')

    def callback(self, data):
        # Get encoder values from encoder_odometry topic
        self.pose = data.pose.pose
        self.twist = data.twist.twist
        rospy.loginfo("Odometry message received: header_frame_id: %s child_frame_id: %s" % (data.header.frame_id, data.child_frame_id))
        self.success = True

    def test_03_odometry_publisher(self):
        self.success = False
        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            rate.sleep()
        self.assertTrue(self.success, 'Odometry message was not sent')

    def test_05_basic_translation_odometry(self):
        success = False
        # Create mock rover translation message
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [100, 100, 100, 100]                           # Drive encoder pulses
        enc_msg.steer_pulses       = [0, 0, 0, 0]                                   # Steer encoder pulses
        enc_msg.drive_revolutions  = [1, 1, 1, 1]                                   # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [0, 0, 0, 0]                                   # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                                   # Steer encoder velocity [pulses per second]
        # Reset odometry
        if self.reset_odometry():
            # Publish encoder message
            self.enc_pub.publish(enc_msg)

            timeout_t = time.time() + 10.0  # 10 s
            rate = rospy.Rate(10)           # 10 Hz
            while not rospy.is_shutdown() and not success and time.time() < timeout_t:
                self.enc_pub.publish(enc_msg)
                # Check for accurate odometry
                if (self.pose.position != Point(0, 0, 0)
                    and self.twist == Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))):
                    success = True
                rate.sleep()
        self.assertTrue(success, 'False odometry interpretation')

    def test_04_basic_drive_odometry(self):
        success = False
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [1000, 1000, 1000, 1000]                       # Drive encoder pulses
        enc_msg.steer_pulses       = [0, 0, 0, 0]                                   # Steer encoder pulses
        enc_msg.drive_revolutions  = [1, 1, 1, 1]                                   # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [100, 100, 100, 100]                           # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                                   # Steer encoder velocity [pulses per second]
        # Reset odometry
        if self.reset_odometry():
            # Publish encoder message
            self.enc_pub.publish(enc_msg)

            timeout_t = time.time() + 10.0  # 10 s
            rate = rospy.Rate(10)           # 10 Hz
            while not rospy.is_shutdown() and not success and time.time() < timeout_t:
                self.enc_pub.publish(enc_msg)
                # Check for accurate odometry
                if (self.twist.linear != Vector3(0, 0, 0)
                    and self.twist.angular == Vector3(0, 0, 0)):
                    success = True
                rate.sleep()
        self.assertTrue(success, 'False odometry interpretation')

    def test_06_basic_roation_odometry(self):
        success = False
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [-4000, -4000, 4000, 4000]                           # Drive encoder pulses
        enc_msg.steer_pulses       = [1000, 3000, 1000, 3000]                         # Steer encoder pulses
        enc_msg.drive_revolutions  = [-1, -1, 1, 1]                                   # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [-1000, -1000, 1000, 1000]                           # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                                   # Steer encoder velocity [pulses per second]
        # Reset odometry
        if self.reset_odometry():
            # Publish encoder message
            self.enc_pub.publish(enc_msg)

            timeout_t = time.time() + 10.0  # 10 s
            rate = rospy.Rate(10)           # 10 Hz
            while not rospy.is_shutdown() and not success and time.time() < timeout_t:
                self.enc_pub.publish(enc_msg)
                # Check for accurate odometry
                if (self.twist.angular != Vector3(0, 0, 0)
                    and self.pose.orientation != Quaternion(0, 0, 0, 1)):
                    success = True
                rate.sleep()
        self.assertTrue(success, 'False odometry interpretation')

    def test_07_drive_odometry(self):
        self.success = False
        v_enc = self.DRIVE_ENC_PPR
        alpha_enc = self.STEER_ENC_PPR/4.0
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [0, 0, 0, 0]                                   # Drive encoder pulses
        enc_msg.steer_pulses       = [alpha_enc, alpha_enc, alpha_enc, alpha_enc]                                   # Steer encoder pulses
        enc_msg.drive_revolutions  = [1, 1, 1, 1]                                   # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [v_enc, v_enc, v_enc, v_enc]                   # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                                   # Steer encoder velocity [pulses per second]
        # Reset odometry
        if self.reset_odometry():
            # Publish encoder message
            self.enc_pub.publish(enc_msg)

            timeout_t = time.time() + 10.0  # 10 s
            rate = rospy.Rate(10)           # 10 Hz
            while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                self.enc_pub.publish(enc_msg)
                rate.sleep()
        # Check for accurate odometry
        v = math.pi*self.wheel_diameter
        v = self.roundValue(v)
        self.assertEqual(self.twist.linear, Vector3(v, 0, 0))

    def test_08_translation_f_odometry(self):
        self.success = False
        p_enc = 0
        r_enc = 5
        v_enc = 0
        alpha_enc = self.STEER_ENC_PPR/4.0
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [p_enc, p_enc, p_enc, p_enc]                   # Drive encoder pulses
        enc_msg.steer_pulses       = [alpha_enc, alpha_enc, alpha_enc, alpha_enc]   # Steer encoder pulses
        enc_msg.drive_revolutions  = [r_enc, r_enc, r_enc, r_enc]                   # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [v_enc, v_enc, v_enc, v_enc]                   # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                                   # Steer encoder velocity [pulses per second]
        # Reset odometry
        if self.reset_odometry():
            # Publish encoder message
            self.enc_pub.publish(enc_msg)

            timeout_t = time.time() + 10.0  # 10 s
            rate = rospy.Rate(10)           # 10 Hz
            while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                self.enc_pub.publish(enc_msg)
                rate.sleep()
        # Check for accurate odometry
        circ = math.pi*self.wheel_diameter
        #x = (r_enc + (p_enc/self.STEER_ENC_PPR))*circ
        #x = self.roundValue(x)
        x = self.roundValue((r_enc + p_enc/self.DRIVE_ENC_PPR)*circ)
        self.assertEqual(self.pose.position, Point(x, 0, 0))

    def test_09_translation_b_odometry(self):
        self.success = False
        p_enc =  -self.DRIVE_ENC_PPR/2.0
        r_enc = -10
        v_enc = 0
        alpha_enc = self.STEER_ENC_PPR/4.0
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [p_enc, p_enc, p_enc, p_enc]                   # Drive encoder pulses
        enc_msg.steer_pulses       = [alpha_enc, alpha_enc, alpha_enc, alpha_enc]   # Steer encoder pulses
        enc_msg.drive_revolutions  = [r_enc, r_enc, r_enc, r_enc]                   # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [v_enc, v_enc, v_enc, v_enc]                   # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                                   # Steer encoder velocity [pulses per second]
        # Reset odometry
        if self.reset_odometry():
            # Publish encoder message
            self.enc_pub.publish(enc_msg)

            timeout_t = time.time() + 10.0  # 10 s
            rate = rospy.Rate(10)           # 10 Hz
            while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                self.enc_pub.publish(enc_msg)
                rate.sleep()
        # Check for accurate odometry
        circ = math.pi*self.wheel_diameter
        x = self.roundValue((r_enc + p_enc/self.DRIVE_ENC_PPR)*circ)
        self.assertEqual(self.pose.position, Point(x, 0, 0))

    def test_10_tranlation_r_odometry(self):
        self.success = False
        p_enc = self.DRIVE_ENC_PPR/4.0
        r_enc = 10
        v_enc = 0
        alpha_enc = 0
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [p_enc, p_enc, p_enc, p_enc]                   # Drive encoder pulses
        enc_msg.steer_pulses       = [alpha_enc, alpha_enc, alpha_enc, alpha_enc]   # Steer encoder pulses
        enc_msg.drive_revolutions  = [r_enc, r_enc, r_enc, r_enc]                   # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [v_enc, v_enc, v_enc, v_enc]                   # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                                   # Steer encoder velocity [pulses per second]
        # Reset odometry
        if self.reset_odometry():
            # Publish encoder message
            self.enc_pub.publish(enc_msg)

            timeout_t = time.time() + 10.0  # 10 s
            rate = rospy.Rate(10)           # 10 Hz
            while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                self.enc_pub.publish(enc_msg)
                rate.sleep()
        # Check for accurate odometry
        circ = math.pi*self.wheel_diameter
        y = -self.roundValue((r_enc + p_enc/self.DRIVE_ENC_PPR)*circ)
        self.assertEqual(self.pose.position, Point(0, y, 0))


    def test_11_tranlation_l_odometry(self):
        self.success = False
        p_enc = 0
        r_enc = 10
        v_enc = 0
        alpha_enc = self.STEER_ENC_PPR/2
        # Create mock rover drive message
        enc_msg = EncoderOdometry()
        enc_msg.drive_pulses       = [p_enc, p_enc, p_enc, p_enc]                   # Drive encoder pulses
        enc_msg.steer_pulses       = [alpha_enc, alpha_enc, alpha_enc, alpha_enc]   # Steer encoder pulses
        enc_msg.drive_revolutions  = [r_enc, r_enc, r_enc, r_enc]                   # Drive encoder wheel revolutions
        enc_msg.drive_velocity     = [v_enc, v_enc, v_enc, v_enc]                   # Drive encoder velocity [pulses per second]
        enc_msg.steer_velocity     = [0, 0, 0, 0]                                   # Steer encoder velocity [pulses per second]
        # Reset odometry
        if self.reset_odometry():
            # Publish encoder message
            self.enc_pub.publish(enc_msg)

            timeout_t = time.time() + 10.0  # 10 s
            rate = rospy.Rate(10)           # 10 Hz
            while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                self.enc_pub.publish(enc_msg)
                rate.sleep()
        # Check for accurate odometry
        circ = math.pi*self.wheel_diameter
        y = self.roundValue((r_enc + p_enc/self.DRIVE_ENC_PPR)*circ)
        self.assertEqual(self.pose.position, Point(0, y, 0))

if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_odometry_node', OdmTest)
