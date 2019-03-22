#!/usr/bin/env python

import unittest
import rospy, rostest
import time

# from maniros.msg import RoverControl # Distance based control
from maniros.msg import MoveControl # Speed based control
from maniros.msg import MotorControl


class ControlAdapterTest(unittest.TestCase):
    def __init__(self, *args):
        super(ControlAdapterTest, self).__init__(*args)
        rospy.init_node("test_input", anonymous=True)
        self.pub = rospy.Publisher("cmd_vel", MoveControl, queue_size=10)
        rospy.Subscriber("motor_control", MotorControl, self.callback)
        self.success = False

    def callback(self, data):
        rospy.loginfo("W_angle: I've heard fl:%.3f rl:%.3f rr:%.3f fr:%.3f." % (
            data.front_left_angle, 
            data.rear_left_angle, 
            data.rear_right_angle, 
            data.front_right_angle
            ))
        rospy.loginfo("Rotation angle: I've heard fl:%.3f rl:%.3f rr:%.3f fr:%.3f." % (
            data.front_left_speed, 
            data.rear_left_speed, 
            data.rear_right_speed, 
            data.front_right_speed
            ))
        self.success = True

    def test_basic_node_actions(self):
        
        msg = MoveControl()
        msg.xSpeed = 1.0
        msg.ySpeed = 0.0         
        msg.rotationAngle = -0.1

        self.pub.publish(msg)
        timeout_t = time.time() + 10.0 #10 seconds

        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.pub.publish(msg)
            time.sleep(0.1)

        self.assertTrue(self.success, 'Callback has not been called')

if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_control_adapter', ControlAdapterTest)
