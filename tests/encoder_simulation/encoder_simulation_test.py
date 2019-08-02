#!/usr/bin/env python

"""
Imports
"""
import unittest
import rospy, rostest
import time

from maniros.msg import EncoderOdometry
from sensor_msgs.msg import JointState

"""
Classes
"""

class EncoderSimTest(unittest.TestCase):
        def __init__(self, *args):
                super(EncoderSimTest, self).__init__(*args)
                rospy.init_node("test_input", anonymous=True)
                rospy.Subscriber("/encoder_odometry", EncoderOdometry, self.callback)
                self.enc_pub = rospy.Publisher("manisim/joint_states", JointState, queue_size=10)

        def callback(self, data):
                self.drive_pulses       = data.drive_pulses
                self.drive_revolutions  = data.drive_revolutions
                self.drive_velocity     = data.drive_velocity
                self.steer_pulses       = data.steer_pulses
                self.steer_velocity     = data.steer_velocity
                self.success            = True

        def test_basic_function(self):
                self.success = False
                jn_msg          = JointState()
                jn_msg.name     = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
                jn_msg.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                jn_msg.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                jn_msg.effort   = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                timeout_t = time.time() + 10.0
                while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                        self.enc_pub.publish(jn_msg)
                        time.sleep(0.1)

                self.assertTrue(self.success, 'Callback has not been called')

        def test_stop(self):
                v               = 0                             # Velocity [rad/s]
                alpha           = 0                             # Orientation [rad]
                self.success    = False
                jn_msg          = JointState()
                jn_msg.name     = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
                jn_msg.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                jn_msg.velocity = [0, 0, v, v, v, v, 0, 0, 0, 0]
                jn_msg.effort   = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                timeout_t = time.time() + 10.0
                while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                        self.enc_pub.publish(jn_msg)
                        time.sleep(0.1)
                v_enc          = 0                              # Velocity [pulses/s]
                drive_velocity = [v_enc, v_enc, v_enc, v_enc]
                self.assertItemsEqual(self.drive_velocity, drive_velocity)

        def test_driving(self):
                v               = math.pi                       # Velocity [rad/s]
                alpha           = 0                             # Orientation [rad]
                self.success    = False
                jn_msg          = JointState()
                jn_msg.name     = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
                jn_msg.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                jn_msg.velocity = [0, 0, v, v, v, v, 0, 0, 0, 0]
                jn_msg.effort   = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                timeout_t = time.time() + 10.0
                while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                        self.enc_pub.publish(jn_msg)
                        time.sleep(0.1)
                v_enc          = int(self.DRIVE_ENC_PPR/2.0)      # Velocity [pulses/s]
                drive_velocity = [v_enc, v_enc, v_enc, v_enc]
                self.assertItemsEqual(self.drive_velocity, drive_velocity)

        def test_steering(self):
                v               = 0                             # Velocity [rad/s]
                alpha           = math.pi/4.0                   # Orientation [rad]
                self.success    = False
                jn_msg          = JointState()
                jn_msg.name     = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
                jn_msg.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                jn_msg.velocity = [0, 0, v, v, v, v, 0, 0, 0, 0]
                jn_msg.effort   = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                timeout_t = time.time() + 10.0
                while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                        self.enc_pub.publish(jn_msg)
                        time.sleep(0.1)
                alpha_enc      = int(self.DRIVE_ENC_PPR/2.0)    # Orientation [pulses]
                steer_pulses    = [alpha_enc, alpha_enc, alpha_enc, alpha_enc]
                self.assertItemsEqual(self.steer_pulses, steer_pulses)

if __name__ == '__main__':
        rostest.rosrun('maniros','test_encoder_pub', EncoderSimTest)
