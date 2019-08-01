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
                self.success = True

        def test_basic_function(self):
                self.success = False
                jn_msg = JointState()
                jn_msg.name = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
                jn_msg.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                jn_msg.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                jn_msg.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

                timeout_t = time.time() + 10.0
                while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
                        self.enc_pub.publish(jn_msg)
                        time.sleep(0.1)

                self.assertTrue(self.success, 'Callback has not been called')

if __name__ == '__main__':
        rostest.rosrun('maniros','test_encoder_pub', EncoderSimTest)
