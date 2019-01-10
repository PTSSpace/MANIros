#!/usr/bin/env python

import unittest
import rospy, rostest
import time

from maniros.msg import RoverControl
from maniros.msg import MotorControl

class ControlAdapterTest(unittest.TestCase):
    def __init__(self, *args):
        super(ControlAdapterTest, self).__init__(*args)
        rospy.wait_for_service('rover_control')
        rospy.init_node("test_input", anonymous=True)
        self.pub = rospy.Publisher("rover_control", RoverControl, queue_size=10)
        rospy.Subscriber("motor_control", MotorControl, self.callback)
        self.success = False

    def callback(self, data):
        print "callback"
        rospy.loginfo("DC controller: I've heard fl:%d rl:%d rr:%d fr:%d." % (
            data.front_left_speed,
            data.rear_left_speed,
            data.rear_right_speed,
            data.front_right_speed
        ))
        self.success = True

    def test_basic_node_actions(self):
        rospy.loginfo('test')
        msg = RoverControl()
        msg.sequenceCount = 1
        msg.mode = 0
        msg.xDistance = 0
        msg.yDistance = 0
        msg.rotationDistance = 2
        msg.time = 10

        self.pub.publish(msg)
        timeout_t = time.time() + 10.0 #10 seconds

        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)

        self.assertTrue(self.success, 'Callback has not been called')

if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_control_adapter', ControlAdapterTest)
