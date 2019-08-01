#!/usr/bin/python
"""
This program provides a ROS node for muxing autonomous rover navigation commands
with human joy stick commands.
The code prioritises human commands before utonomous commands and blocks for a set periode
after wach human interaction.
The navigation commands are passed on via a ROS action to the locomotion control node.

Subscribed ROS topics:
*   move_base/cmd_vel
*   teleop/cmd_vel
ROS actions:
*   locomotion_control
"""

"""
Imports
"""
import rospy
import time
import actionlib

from geometry_msgs.msg import Twist
from maniros.msg import MoveControl # Speed based control
# from maniros.msg import RoverControl # Distance based control
from maniros.msg import LocomotionAction
from maniros.msg import LocomotionGoal

"""
Classes
"""
class CmdVel:
    def __init__(self):
        # Start up action client and wait for action server
        self.client = actionlib.SimpleActionClient("locomotion_control", LocomotionAction)
        self.client.wait_for_server()
        # Subscribe to publishers
        rospy.Subscriber("move_base/cmd_vel", Twist, callback=self.on_autonomous_cmd)
        rospy.Subscriber("teleop/cmd_vel", Twist, callback=self.on_human_cmd)
        self.block_duration = 0
        self.human_cmd_time = time.time()

    def on_autonomous_cmd(self, data):
        # Autonomous navigation stack movement command
        time_since_human_cmd = time.time() - self.human_cmd_time
        if time_since_human_cmd >= self.block_duration:
            self.block_duration = 0 # stop blocking
            self.locomotion_control_client(data)

    def on_human_cmd(self, data):
        # Human joy stick movement command
        self.human_cmd_time = time.time()
        self.block_duration = 5                 # Set block for autonomous commands
        self.locomotion_control_client(data)

    def locomotion_control_client(self, data):
        # Send goal to the action server
        msg = MoveControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/cmd_vel";
        msg.x = data.linear.x
        msg.y = data.linear.y
        msg.rz = data.angular.z
        # Send goal to the action server
        goal = LocomotionGoal(command = msg)
        self.client.send_goal(goal,
                              active_cb=self._goal_active,
                              feedback_cb=self._goal_feedback,
                              done_cb=self._goal_done)
        rospy.logdebug("CV (out) \t x:%f y:%f rot:%f" % (goal.command.x, goal.command.y, goal.command.rz));

    def _goal_active(self):
        rospy.logdebug("CV \t Goal transitioned to active state")

    def _goal_feedback(self, feedback):
        rospy.logdebug("CV \t Goal feedback received: {}".format(feedback.sequence))

    def _goal_done(self, state, result):
        rospy.logdebug("CV \t Goal completed")
        rospy.logdebug(str(state))
        rospy.logdebug(str(result))
        rospy.logdebug("CV \t CAN message IO feedback: " + str(result.sequence))

    def shutdown(self):
        self.client.cancel_goal()

"""
Main
"""
if __name__ == '__main__':
    rospy.init_node("cmd_vel")
    muxer = CmdVel()
    rospy.spin()

rospy.on_shutdown(muxer.shutdown)
