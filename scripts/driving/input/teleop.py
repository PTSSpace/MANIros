#!/usr/bin/python

import rospy
import subprocess

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
import math

class Teleop:
    def __init__(self):
        # initilizes and saves the state of the deadman switch (button RB)
        self.deadman = 0

        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)

    def on_joy(self, data):
        twist = Twist()
        abort = Twist()
        # adjusting input to right-hand rover coordinate system
        # seen from above (x - forward, y - left,z - upward)
        twist.linear.x = data.axes[5]
        twist.linear.y = -data.axes[4]
        twist.angular.z = -data.axes[2]* math.pi/2 
        abort.linear.x = 0
        abort.linear.y = 0
        abort.angular.z = 0

        if (data.buttons[5]):
            self.deadman = 1
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo("Teleop \t x:%f y:%f rot:%f" % (twist.linear.x, twist.linear.y, twist.angular.z))

        if (data.buttons[5] == 0 and self.deadman != 0):
            self.deadman = 0
            rospy.loginfo("Teleop: \t Deadman switch is:%d" % self.deadman)
            self.cmd_vel_pub.publish(abort)
            rospy.loginfo("Teleop: \t Abort message sent")

        # Cancel move base goal
        if data.buttons[2]: # B button
            rospy.loginfo('Cancelling move_base goal')
            cancel_msg = GoalID()
            self.goal_cancel_pub.publish(cancel_msg)

if __name__ == '__main__':
    rospy.init_node("teleop")
    controller = Teleop()
    rospy.spin()
