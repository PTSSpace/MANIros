#!/usr/bin/env python

import unittest
import rospy, rostest
import time

from sensor_msgs.msg import Imu


class ImuPubTest(unittest.TestCase):
    def __init__(self, *args):
        super(ImuPubTest, self).__init__(*args)
        rospy.init_node("test_input", anonymous=True)
        rospy.Subscriber("/imu", Imu, self.callback)
        self.success = False

    def callback(self, data):
        rospy.loginfo("Angular velocity: I've heard vx:%d vy:%d vz:%d." % (
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z,
        ))
        rospy.loginfo("Linear acceleration: I've heard ax:%d ay:%d az:%d." % (
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
        ))
        self.success = True

    def test_basic_node_actions(self):
        
        timeout_t = time.time() + 10.0 #10 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.success, 'Callback has not been called')

if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_imu_pub', ImuPubTest)
