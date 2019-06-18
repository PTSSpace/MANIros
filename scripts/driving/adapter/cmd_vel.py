#!/usr/bin/python

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
        time_since_human_cmd = time.time() - self.human_cmd_time
        if time_since_human_cmd >= self.block_duration:
            self.block_duration = 0 # stop blocking
            self.locomotion_control_client(data)

    def on_human_cmd(self, data):
        self.human_cmd_time = time.time()
        self.block_duration = 5
        self.locomotion_control_client(data)

    def locomotion_control_client(self, data):
        # Send goal to the action server
        msg = MoveControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/cmd_vel";
        msg.xSpeed = data.linear.x
        msg.ySpeed = data.linear.y
        msg.rotationAngle = data.angular.z
        # Send goal to the action server
        goal = LocomotionGoal(command = msg)
        self.client.send_goal(goal,
                              active_cb=self._goal_active,
                              feedback_cb=self._goal_feedback,
                              done_cb=self._goal_done)

        rospy.loginfo("Cmd_vel \t x:%f y:%f rot:%f" % (goal.command.xSpeed, goal.command.ySpeed, goal.command.rotationAngle));

    def _goal_active(self):
        rospy.loginfo("Goal transitioned to active state")

    def _goal_feedback(self, feedback):
        rospy.loginfo("Goal feedback received: {}".format(feedback.sequence))

    def _goal_done(self, state, result):
        rospy.loginfo("Goal one done callback triggered")
        rospy.loginfo(str(state))
        rospy.loginfo(str(result))
        rospy.loginfo("Do something result: " + str(result.sequence))

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
