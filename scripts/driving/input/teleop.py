#!/usr/bin/python

import rospy
import actionlib
import math

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from maniros.msg import MoveCommand         # Locomotion control switches
from maniros.msg import EpsAction
from maniros.msg import EpsGoal
from maniros.msg import EpsCommand

class Teleop:
    def __init__(self):
        # initilizes and saves the state of the deadman switch (button RB)
        self.deadman = 0
        # Start up action client and wait for action server
        # self.eps_client = actionlib.SimpleActionClient("eps_control", EpsAction)
        # self.eps_client.wait_for_server()

        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.lc_switch_pub = rospy.Publisher("teleop/lc_switch", MoveCommand, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)
        self.lcTwist = [0, 0, 0]
        self.lcSwitch = [0, 0, 0, 0]
        self.epsSwitch = [0, 0]

    def on_joy(self, data):
        # Check for dead man switch
        if (data.buttons[5] and self.deadman == 0):
            self.deadman = 1
        # Send operation commands
        elif (data.buttons[5] and self.deadman != 0):
            # Check for eps switches
            if (data.buttons[4] != self.epsSwitch[0]):
                if (any(data.buttons[4,6])):
                    # EPS Motor off message
                    msg = EpsCommand()
                    if (data.buttons[4] == 1):
                        msg.MotorPower = 0
                    elif (data.buttons[6] == 1):
                        msg.MotorPower = 1
                    # Send goal to the action server
                    goal = LocomotionGoal(command = msg)
                    """
                    self.eps_client.send_goal(goal,
                                          active_cb=self._goal_active,
                                          feedback_cb=self._goal_feedback,
                                          done_cb=self._goal_done)
                    """
                    rospy.logdebug("TP (out) \t MotorPower switch:%d" % (goal.MotorPower));
                self.epsSwitch = [data.buttons[4], data.buttons[6]]
            # Check for locomotion switches
            if (data.buttons[0:4] != self.lcSwitch):
                if (any(data.buttons[0:4]) != 0):
                    # Send locomotion switch command
                    switch = MoveCommand()
                    switch.header.stamp     = rospy.Time.now()
                    switch.header.frame_id  = "teleop/lc_switch";
                    switch.SteerPower       = data.buttons[0] - self.lcSwitch[0]
                    switch.DrivePower       = data.buttons[3] - self.lcSwitch[3]
                    switch.Publisher        = data.buttons[1] - self.lcSwitch[1]
                    switch.ZeroEncoders     = data.buttons[2] - self.lcSwitch[2]
                    self.lc_switch_pub.publish(switch)
                    rospy.logdebug("TP (out) \t Steer:%d \t Drive:%d" % (switch.SteerPower, switch.DrivePower))
                    rospy.logdebug("TP (out) \t Publisher:%d \t ZeroEncoders:%d" % (switch.Publisher, switch.ZeroEncoders))
                self.lcSwitch = data.buttons[0:4]
            # Check for locomotion commands
            if ([data.axes[2],data.axes[4],data.axes[5]] != self.lcTwist):
                # Move control message
                # adjusting input to right-hand rover coordinate system
                # seen from above (x - forward, y - right,z - downward)
                twist = Twist()
                twist.linear.x = data.axes[5]
                twist.linear.y = data.axes[4]
                twist.angular.z = data.axes[2]* math.pi/2
                self.cmd_vel_pub.publish(twist)
                rospy.logdebug("TP (out) \t x:%f \t y:%f \t rot:%f" % (twist.linear.x, twist.linear.y, twist.angular.z))
                self.lcTwist = [data.axes[2],data.axes[4],data.axes[5]]

        elif (data.buttons[5] == 0 and self.deadman != 0):
            self.deadman = 0
            rospy.logdebug("TP \t Deadman switch is:%d" % self.deadman)
            # Move control abort message
            abort = Twist()
            abort.linear.x = 0
            abort.linear.y = 0
            abort.angular.z = 0
            self.cmd_vel_pub.publish(abort)
            rospy.logdebug("TP (out) \t Abort message sent")

    def _goal_active(self):
        rospy.logdebug("TP \t Goal transitioned to active state")

    def _goal_feedback(self, feedback):
        rospy.logdebug("TP \t Goal feedback received: {}".format(feedback.sequence))

    def _goal_done(self, state, result):
        rospy.logdebug("TP \t Goal completed")
        rospy.logdebug(str(state))
        rospy.logdebug(str(result))
        rospy.logdebug("TP \t CAN message IO feedback: " + str(result.sequence))

    def shutdown(self):
        self.client.cancel_goal()

if __name__ == '__main__':
    rospy.init_node("teleop")
    controller = Teleop()
    rospy.spin()
