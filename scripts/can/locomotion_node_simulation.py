#!/usr/bin/python
from __future__ import print_function

"""
This program provides a ROS node for the can0 bus interface.
It connects the locomotion nodes to the ROS network.
Veloctity and general motor commands are forwarded to the drive nodes.
Encoder odometry messages are received from the drive nodes and published to the ROS network.

Subscribed ROS topics:
*   teleop/lc_switch
ROS actions:
*   locomotion_control
Published ROS topics:
*   encoder_odometry
"""

"""
Imports
"""
from can_protocol import *
import rospy
import actionlib
import time
import math

# Import ROS messages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from maniros.msg import MoveCommand                                                 # Locomotion control switches
# Locomotion control action
from maniros.msg import LocomotionAction
from maniros.msg import LocomotionFeedback
from maniros.msg import LocomotionResult

# Velocity based vector protocol
from vector_protocol.vector_protocol import VectorTranslation

"""
Classes
"""

class LocomotionSimulation(object):
    """Locomotion control/monitoring server for CAN bus communication with wheel controllers"""
    _feedback = LocomotionFeedback()
    _result = LocomotionResult()

    def __init__(self, name):
        # Switch states
        self.lcInitialised      = False
        self.driveMode          = False
        self.steerMode          = False
        self.driving            = False
        self.lcError            = False
        self.wheelSpeed         = [0, 0, 0, 0]
        self.wheelAngle         = [MAX_ORT*5, MAX_ORT*5, MAX_ORT*5, MAX_ORT*5]                      # Imposible position for start orientation

        # Get ros parameters
        self.rover_length = rospy.get_param("/rover_length")
        self.rover_width = rospy.get_param("/rover_width")

        # Locomotion control publishers
        # Wheel velocity
        self.vel_pub = [rospy.Publisher("/mani/drive_fl_vel/command", Float64, queue_size=1),
                        rospy.Publisher("/mani/drive_rl_vel/command", Float64, queue_size=1),
                        rospy.Publisher("/mani/drive_rr_vel/command", Float64, queue_size=1),
                        rospy.Publisher("/mani/drive_fr_vel/command", Float64, queue_size=1)]
        # Wheel orientation
        self.ort_pub = [rospy.Publisher("/mani/steer_fl_ort/command", Float64, queue_size=1),
                        rospy.Publisher("/mani/steer_rl_ort/command", Float64, queue_size=1),
                        rospy.Publisher("/mani/steer_rr_ort/command", Float64, queue_size=1),
                        rospy.Publisher("/mani/steer_fr_ort/command", Float64, queue_size=1)]
        # Joint velocity and orientation subscriber
        self.joint_sub = rospy.Subscriber("/mani/joint_states", JointState, self.get_joint_states, queue_size=10)

        """
        # Subscribe to locomotion commands
        self.switch_sub = rospy.Subscriber("teleop/lc_switch", MoveCommand, self.locomotion_switch, queue_size=10)

        # Initialise ROS nodes
        rospy.loginfo("LC \t Initialise CAN Drive nodes")
        self.lcInitialised = True
        self.drive_node_initialise()
        """

        # Locomotion control action
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LocomotionAction, execute_cb=self.locomotion_control, auto_start = False)
        self._as.start()

    """
    def locomotion_switch(self, data):
        # Locomotion commands subscriber callback
        rospy.loginfo("LC (in) \t Steer:%d \t Drive:%d \t Publisher:%d \t ZeroEncoders:%d" % (data.SteerPower, data.DrivePower, data.Publisher, data.ZeroEncoders))
        if (data.DrivePower or data.SteerPower or data.Publisher or data.ZeroEncoders):
            # Toggle driving and steering switches
            if data.DrivePower:
                self.steerMode = not self.steerMode
            if data.SteerPower:
                self.driveMode = not self.driveMode
        for idx, wheel in enumerate(wheelIndex):
            # Publish locomotion command
            rospy.loginfo("LC (out) \t %s wheel \t Steer:%d \t Drive:%d"
                % (wheel, self.steerMode, self.driveMode))
            self.vel_pub[idx].publish(0.0)
    """
    def locomotion_control(self, goal):
        # Append message CAN bus message feedback
        self._feedback.sequence = []

        rospy.loginfo("LC (in) \t x:%d \t y:%d \t rot:%d - translating..." % (goal.command.xSpeed, goal.command.ySpeed, goal.command.rotationAngle))
        # Convert velocity twist messages to individual wheel velocity and orientation
        [wheelSpeed, wheelAngle] = VectorTranslation(self.rover_length, self.rover_width).translateMoveControl(goal.command)

        # Check current locomotion state
        if ((wheelAngle == self.wheelAngle) and not self.lcError and not(wheelSpeed == self.wheelSpeed)):
            self.driving = any(wheelSpeed)
            success = self.velocity_control(wheelSpeed)
        else:
            if self.driving:
                # Stop before executing new steering comand
                success = self.velocity_control([0, 0, 0, 0])
                if success:
                    self.driving = False
                    success = self.orientation_control(wheelAngle)
            else:
                success = self.orientation_control(wheelAngle)
            if (success and not (wheelSpeed == [0, 0, 0, 0])):
                self.driving = True
                success = self.velocity_control(wheelSpeed)

        # Publish the feedback
        self._as.publish_feedback(self._feedback)
        if success:
            self.lcError = False
            if all(self._feedback.sequence):
                rospy.loginfo('LC \t %s: Succeeded' % self._action_name)
            self._result.sequence = self._feedback.sequence
            self._as.set_succeeded(self._result)
        else:
            # Set error flag
            self.lcError = True
            success = False
            rospy.loginfo('LC \t %s: Failed' % self._action_name)

    def get_joint_states(self, data):
        # Get joint state values from simulation
        self.wheelAngle = data.position
        self.wheelSpeed = data.velocity

    def check_preempt(self):
        success = True
        if (self._as.is_preempt_requested() or rospy.is_shutdown()):
            rospy.loginfo('LC \t %s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        return success

    def orientation_control(self, wheelAngle):
        success = True
        r = rospy.Rate(500)
        rospy.loginfo('LC \t %s: Executing, orientation control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt and publish wheel velocity
            rospy.loginfo('LC \t %f' % wheelAngle[idx])
            self.ort_pub[idx].publish(Float64(wheelAngle[idx]))
            self._feedback.sequence.append(idx+1)
        # Check locomotion state
        while (wheelAngle != self.wheelAngle or success != False):
            sucess = self.check_preempt()
            r.sleep()
        return success

    def velocity_control(self, wheelSpeed):
        success = True
        r = rospy.Rate(500)
        rospy.loginfo('LC \t %s: Executing, velocity control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt and publish wheel velocity
            self.vel_pub[idx].publish(Float64(wheelSpeed[idx]*MAX_VEL))
            self._feedback.sequence.append(idx+1)
        # Check locomotion state
        while (wheelSpeed != self.wheelSpeed or success != False):
            sucess = self.check_preempt()
            r.sleep()
        return success

    def drive_node_initialise(self):
        # Initialise all driving nodes
        initMsg = MoveCommand()                                 # Initialisation message
        initMsg.SteerPower = True
        initMsg.DrivePower = True
        initMsg.Publisher = True
        initMsg.ZeroEncoders = True
        # Send CAN messages
        self.locomotion_switch(initMsg)

    def shutdown(self):
        rospy.loginfo("Shutting down locomotion node")

"""
Main
"""
if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("locomotion_simulation", anonymous=True)

    # Start ROS action
    server = LocomotionSimulation(rospy.get_name())
    rospy.spin()

rospy.on_shutdown(server.shutdown)
