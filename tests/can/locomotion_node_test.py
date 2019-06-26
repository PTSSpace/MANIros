#!/usr/bin/env python

import unittest
import rospy, rostest
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID


class TeleopTest(unittest.TestCase):
    def __init__(self, *args):
        super(TeleopTest, self).__init__(*args)
        rospy.init_node("test_input", anonymous=True)
        self.pub = rospy.Publisher("joy", Joy, queue_size=10)
        rospy.Subscriber("teleop/cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("move_base/cancel", GoalID, self.move_callback)
        self.success = False

    def vel_callback(self, data):
        rospy.loginfo("Twist speed: I've heard vx:%d vy:%d vz:%d." % (
            data.linear.x,
            data.linear.y,
            data.linear.z,
        ))
        rospy.loginfo("Twist angle: I've heard rx:%d ry:%d rz:%d." % (
            data.angular.x,
            data.angular.y,
            data.angular.z,
        ))
        self.success = True

    def test_basic_node_actions(self):
        
        msg = Joy()
        msg.axes = [0.0]*6
        msg.axes[5] = 1.0       
        msg.axes[4] = 0.0 
        msg.axes[2] = 1.0
        msg.buttons = [0]*6
        msg.buttons[5] = 1      # deadman switch
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/vel_teleop_test";

        self.pub.publish(msg)
        timeout_t = time.time() + 10.0 #10 seconds

        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.pub.publish(msg)
            time.sleep(0.1)

        self.assertTrue(self.success, 'Callback has not been called')

    def move_callback(self, data):
        rospy.loginfo("Time stamp:" + str(data.stamp))
        rospy.loginfo("Goal id:" + str(data.id))
        self.success = True

    def test_move_cancel(self):
        
        msg = Joy()
        msg.axes = [0.0]*6
        msg.axes[5] = 0.0       
        msg.axes[4] = 0.0 
        msg.axes[2] = 0.0
        msg.buttons = [0]*6
        msg.buttons[2] = 1      # X button
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/move_teleop_test";

        self.pub.publish(msg)
        timeout_t = time.time() + 10.0 #10 seconds

        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.pub.publish(msg)
            time.sleep(0.1)

        self.assertTrue(self.success, 'Callback has not been called')

if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_teleop', TeleopTest)
