#!/usr/bin/env python

import unittest
import rospy, rostest
import time

from geometry_msgs.msg import Twist
from maniros.msg import MoveControl # Speed based control
# from maniros.msg import RoverControl # Distance based control


class CmdVelTest(unittest.TestCase):
    def __init__(self, *args):
        super(CmdVelTest, self).__init__(*args)
        rospy.init_node("test_input", anonymous=True)
        self.pub_move = rospy.Publisher("move_base/cmd_vel", Twist, queue_size=10)
        self.pub_teleop = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=10)
        # Locomotion control action
        self._action_name = "locomotion_control"
        self._as = actionlib.SimpleActionServer(self._action_name, LocomotionAction, execute_cb=self.action_cb, auto_start = False)
        self._as.start()
        self.success = False

    def action_cb(self, goal):
        rospy.loginfo("Linear: I've heard x:%f y:%f." % (
            goal.command.x,
            goal.command.y
            ))
        rospy.loginfo("Rotation: I've heard rz:%f." % (
            goal.command.rz
            ))
        self._result.sequence = [1]
        self._as.set_succeeded(self._result)
        self.success = True

    def test_teleop_actions(self):

        msg = Twist()
        #msg.linear = [0.0]*3
        #msg.angular = [0.0]*3
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.angular.z = 0.1

        self.pub_teleop.publish(msg)

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.pub_teleop.publish(msg)
            rate.sleep()

        self.assertTrue(self.success, 'Locomotion action has not been called')

    def test_autonomous_actions(self):

        msg = Twist()
        #msg.linear = [0.0]*3
        #msg.angular = [0.0]*3
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.angular.z = -1.0

        # Test blocking by human command
        self.pub_teleop.publish(msg)

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.pub_teleop.publish(msg)
            rate.sleep()

        self.success = False

        self.pub_move.publish(msg)
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.pub_move.publish(msg)
            rate.sleep()

        self.assertTrue(self.success, 'Locomotion action has not been called')

if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_cmd_vel', CmdVelTest)
