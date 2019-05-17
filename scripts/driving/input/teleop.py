#!/usr/bin/python

import rospy
import subprocess
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from maniros.msg import MoveCommand         # Locomotion control switches

#from actionlib_msgs.msg import GoalID


class Teleop:
    def __init__(self):
        # initilizes and saves the state of the deadman switch (button RB)
        self.deadman = 0

        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.lc_switch_pub = rospy.Publisher("teleop/lc_switch", Twist, queue_size=1)
        #self.goal_cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)

    def on_joy(self, data):
        # Check for dead man switch
        if (data.buttons[5]):
            self.deadman = 1
            # Move control message
            # adjusting input to right-hand rover coordinate system
            # seen from above (x - forward, y - left,z - upward)
            twist = Twist()
            twist.linear.x = data.axes[5]
            twist.linear.y = -data.axes[4]
            twist.angular.z = -data.axes[2]* math.pi/2
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo("Teleop \t x:%f \t y:%f \t rot:%f" % (twist.linear.x, twist.linear.y, twist.angular.z))
            if (any(data.buttons[1:3]) != 0):
                # Send locomotion switch command
                switch = MoveCommand()
                switch.SteerPower = data.buttons[0]
                switch.DrivePower = data.buttons[3]
                switch.Publisher = data.buttons[1]
                switch.ZeroEncoders = data.buttons[2]
                self.lc_switch_pub.publish(switch)
                rospy.loginfo("Teleop \t Steer:%d \t Drive:%d \t Publisher:%d \t ZeroEncoders:%d" % (switch.SteerPower, switch.DrivePower, switch.Publisher, switch.ZeroEncoders))

        if (data.buttons[5] == 0 and self.deadman != 0):
            self.deadman = 0
            rospy.loginfo("Teleop: \t Deadman switch is:%d" % self.deadman)
            # Move control abort message
            abort = Twist()
            abort.linear.x = 0
            abort.linear.y = 0
            abort.angular.z = 0
            self.cmd_vel_pub.publish(abort)
            rospy.loginfo("Teleop: \t Abort message sent")
        '''
        # Cancel move base goal
        if data.buttons[2]: # B button
            rospy.loginfo('Cancelling move_base goal')
            cancel_msg = GoalID()
            self.goal_cancel_pub.publish(cancel_msg)
        '''
if __name__ == '__main__':
    rospy.init_node("teleop")
    controller = Teleop()
    rospy.spin()
