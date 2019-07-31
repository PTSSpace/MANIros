#!/usr/bin/python
from __future__ import print_function

"""
This program provides a ROS node for the gazeobo simulation interface.
It connects the locomotion nodes to the ROS network.
Veloctity and general motor commands are forwarded to the drive nodes.
Joint state messages are received from the drive nodes and interpreted during the locomotion precedure.

Subscribed ROS topics:
*   mani/joint_states
ROS actions:
*   locomotion_control
Published ROS topics:
*   mani/drive_fl_vel/command
*   mani/drive_rl_vel/command
*   mani/drive_rr_vel/command
*   mani/drive_fr_vel/command
*   mani/steer_fl_ort/command
*   mani/steer_rl_ort/command
*   mani/steer_rr_ort/command
*   mani/steer_fr_ort/command
"""

"""
Imports
"""
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
from ..vector_protocol.vector_protocol import VectorTranslation

"""
Classes
"""

class LocomotionSimulation(object):
    """Locomotion control/monitoring server for CAN bus communication with wheel controllers"""
    _feedback = LocomotionFeedback()
    _result = LocomotionResult()

    def __init__(self, name):

        self._action_name = name

        # Set working parameters
        self.tollerance         = 0.01                                  # Tollerance for the angle/velocity state
        # Get ros parameters
        self.rover_length       = rospy.get_param("/rover_length")      # Rover length [m]
        self.rover_width        = rospy.get_param("/rover_width")       # Rover width [m]
        self.MAX_VEL            = rospy.get_param("/max_vel")           # Maximal wheel velocity [rad/s]
        self.MAX_ORT            = rospy.get_param("/max_ort")           # Maximal wheel orientation [rad]

        # Switch states
        self.lcInitialised      = False
        self.driveMode          = True
        self.steerMode          = True
        self.driving            = False
        self.lcError            = False
        self.publisherMode	= False
        self.wheelSpeed         = [0, 0, 0, 0]
        self.wheelAngle         = [self.MAX_ORT*5, self.MAX_ORT*5, self.MAX_ORT*5, self.MAX_ORT*5]                      # Imposible position for start orientation

        # Locomotion control publishers
        # Wheel velocity
        self.vel_pub = [rospy.Publisher("/manisim/drive_fl_vel/command", Float64, queue_size=1),
                        rospy.Publisher("/manisim/drive_rl_vel/command", Float64, queue_size=1),
                        rospy.Publisher("/manisim/drive_rr_vel/command", Float64, queue_size=1),
                        rospy.Publisher("/manisim/drive_fr_vel/command", Float64, queue_size=1)]
        # Wheel orientation
        self.ort_pub = [rospy.Publisher("/manisim/steer_fl_ort/command", Float64, queue_size=1),
                        rospy.Publisher("/manisim/steer_rl_ort/command", Float64, queue_size=1),
                        rospy.Publisher("/manisim/steer_rr_ort/command", Float64, queue_size=1),
                        rospy.Publisher("/manisim/steer_fr_ort/command", Float64, queue_size=1)]
        # Joint velocity and orientation subscriber
        self.joint_sub = rospy.Subscriber("/manisim/joint_states", JointState, self.get_joint_states, queue_size=10)

        # Subscribe to locomotion state commands
        self.switch_sub = rospy.Subscriber("teleop/lc_switch", MoveCommand, self.locomotion_switch, queue_size=10)

        # Locomotion control action
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LocomotionAction, execute_cb=self.locomotion_control, auto_start = False)

        # Initialise ROS nodes
        rospy.loginfo("LC \t Initialise Simulation Drive nodes")
#        self.drive_node_initialise()
        self.lcInitialised = True

        # Start locomotion control action
        self._as.start()


    def locomotion_switch(self, data):
        # Locomotion commands subscriber callback
        rospy.loginfo("LC (in) \t Steer:%d \t Drive:%d \t Publisher:%d \t ZeroEncoders:%d" % (data.SteerPower, data.DrivePower, data.Publisher, data.ZeroEncoders))
        if (data.DrivePower or data.SteerPower or data.Publisher or data.ZeroEncoders):
            # Toggle driving and steering switches
            if data.DrivePower:
                self.steerMode = not self.steerMode
            if data.SteerPower:
                self.driveMode = not self.driveMode
            # Toggle publisher switch
            # TODO: does nothing so far
            if data.Publisher:
                self.publisherMode = not self.publisherMode
            # Simulate zeroing encoders
            self.zero_encoders()
            # Publish locomotion command
            rospy.loginfo("LC (out) \t Steer:%d \t Drive:%d \t Publisher:%d \t Zero:%d"
            % (self.steerMode, self.driveMode, self.publisherMode, data.ZeroEncoders))
            if not self.driveMode:
                success = self.velocity_control([0, 0, 0, 0])

    def zero_encoders(self):
        """
        Simulate zeroing of encoders.
        Steer all wheel to end position for hard stop.
        """
        self.orientation_control([self.MAX_ORT, self.MAX_ORT, self.MAX_ORT, self.MAX_ORT])


    def locomotion_control(self, goal):
        # Append message simulation joint feedback
        self._feedback.sequence = []
        success = False
        rospy.loginfo("LC (in) \t x:%f \t y:%f \t rot:%f - translating..." % (goal.command.x, goal.command.y, goal.command.rz))
        # Convert velocity twist messages to individual wheel velocity and orientation
        [wheelSpeedNorm, wheelAngle] = VectorTranslation(self.rover_length, self.rover_width).translateMoveControl(goal.command)
        wheelSpeed = [value * self.MAX_VEL for value in wheelSpeedNorm]			# Scale wheel velocity
        # Check current locomotion state
        if ((wheelAngle == self.wheelAngle) and not self.lcError and not(wheelSpeed == self.wheelSpeed)):
            self.driving = any(wheelSpeed)
            if self.driveMode:
                success = self.velocity_control(wheelSpeed)
        else:
            if self.driving:
                # Stop before executing new steering comand
                if self.driveMode:
                    success = self.velocity_control([0, 0, 0, 0])
                if success:
                    success = False
                    self.driving = False
                    if self.steerMode:
                        success = self.orientation_control(wheelAngle)
            else:
                if self.steerMode:
                    success = self.orientation_control(wheelAngle)
            if (success and not (wheelSpeed == [0, 0, 0, 0])):
                success = False
                self.driving = True
                if self.driveMode:
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
            rospy.loginfo('LC \t %s: Failed' % self._action_name)

    def get_joint_states(self, data):
        # Get joint state values from simulation
        self.wheelAngle = data.position[6:10]
        self.wheelSpeed = data.velocity[2:6]

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
        for idx in range(0, len(wheelAngle)):
            # Extraxt and publish wheel velocity
            rospy.loginfo('LC \t %f' % wheelAngle[idx])
            self.ort_pub[idx].publish(Float64(wheelAngle[idx]))
            self._feedback.sequence.append(idx+1)
        # Check locomotion state
        upperAngle = [0, 0, 0, 0]
        lowerAngle = [0, 0, 0, 0]
        for idx, value in enumerate(wheelAngle):
            upperAngle[idx] = value + 10*self.tollerance
            lowerAngle[idx] = value - 10*self.tollerance
        exit = False
        while not exit and success:
            exit = True
            success = self.check_preempt()
            for idx in range (0, len(self.wheelAngle)):
                if ((self.wheelAngle[idx] > upperAngle[idx]) or (self.wheelAngle[idx] <  lowerAngle[idx])):
                    exit = False
            r.sleep()
        rospy.loginfo('LC \t Orientation control success: %d' % success)
        return success

    def velocity_control(self, wheelSpeed):
        success = True
        r = rospy.Rate(500)
        rospy.loginfo('LC \t %s: Executing, velocity control' % (self._action_name))
        for idx in range(0, len(wheelSpeed)):
            # Extraxt and publish wheel velocity
            rospy.loginfo('LC \t %f' % wheelSpeed[idx])
            self.vel_pub[idx].publish(Float64(wheelSpeed[idx]))
            self._feedback.sequence.append(idx+1)
        # Check locomotion state
        upperSpeed = [0, 0, 0, 0]
        lowerSpeed = [0, 0, 0, 0]
        for idx, value in enumerate(wheelSpeed):
            upperSpeed[idx] = value + self.tollerance
            lowerSpeed[idx] = value - self.tollerance
        exit = False
        while not exit and success:
            exit = True
            success = self.check_preempt()
            for idx in range (0, len(self.wheelSpeed)):
                if ((self.wheelSpeed[idx] > upperSpeed[idx]) or (self.wheelSpeed[idx] <  lowerSpeed[idx])):
                    exit = False
            r.sleep()
        rospy.loginfo('LC \t Velocity control success: %d' % success)
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
        rospy.loginfo("LC \t Shutting down locomotion node")

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
