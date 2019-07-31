#!/usr/bin/python
from __future__ import print_function

"""
This program provides a ROS node for the can0 bus interface.
It connects the locomotion nodes to the ROS network.
Velocitity and general motor commands are forwarded to the drive nodes.
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
import rospy
import actionlib
import time
import threading
import Queue

# Import CAN protocol parameters
from can_protocol import *
# Import CAN interface
from can_interface import CANInterface

# Import ROS messages
from maniros.msg import MoveCommand                                                 # Locomotion control switches
from maniros.msg import EncoderOdometry                                             # Encoder odometry feedback
from maniros.msg import Vector4                                                     # Vector format for wheel messages
from maniros.msg import Vector4Int                                                  # Vector format for wheel messages
# Locomotion control action
from maniros.msg import LocomotionAction
from maniros.msg import LocomotionFeedback
from maniros.msg import LocomotionResult

# Velocity based vector protocol
from vector_protocol.vector_protocol import VectorTranslation

"""
Classes
"""

class LocomotionControl(object):
    """Locomotion control/monitoring server for CAN bus communication with wheel controllers"""
    _feedback = LocomotionFeedback()
    _result = LocomotionResult()

    def __init__(self, name):
        # Get ros parameters
        self.rover_length   = rospy.get_param("/rover_length")
        self.rover_width    = rospy.get_param("/rover_width")
        MAX_CUR_B           = rospy.get_param("/max_cur_b")             # Maximal current on battery current sensor [A]
        MAX_CUR_M           = rospy.get_param("/max_cur_m")             # Maximal current on motor current sensor [A]
        MAX_VEL             = rospy.get_param("/max_vel")               # Maximal wheel velocity [rad/s]
        MAX_ORT             = rospy.get_param("/max_ort")               # Maximal wheel orientation [rad]

        self.MAX_RATING     = [MAX_VEL, MAX_ORT, MAX_CUR_B, MAX_CUR_M]  # Maximal parameters for motors and sensors

        # Switch states
        self.lcInitialised      = False
        self.driveMode          = False
        self.steerMode          = False
        self.publisherMode      = False
        self.driving            = False
        self.lcError            = False
        self.wheelSpeed         = [0, 0, 0, 0]
        self.wheelAngle         = [MAX_ORT, MAX_ORT, MAX_ORT, MAX_ORT]

        # Construct CAN bus interface
        self.ci = CANInterface(MAX_RATING)

        # Encoder odometry publisher
        self.encoder_pub = rospy.Publisher("encoder_odometry", EncoderOdometry, queue_size=10)
        # Subscribe to locomotion state commands
        self.switch_sub = rospy.Subscriber("teleop/lc_switch", MoveCommand, self.locomotion_switch, queue_size=10)

        # Start ROS publisher for encoder odometry
        self.timer = rospy.Timer(rospy.Duration(10), self.CAN_subscriber)

        # Locomotion control action
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LocomotionAction, execute_cb=self.locomotion_control, auto_start = False)
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
            if data.Publisher:
                self.publisherMode = not self.publisherMode
            # Motor start/stop command
            # Send CAN locomotion command
            for idx, wheel in enumerate(wheelIndex):
                rospy.loginfo("LC (out) \t %s wheel \t Steer:%d \t Drive:%d \t Publisher:%d \t Zero:%d"
                % (wheel, self.steerMode, self.driveMode, self.publisherMode, data.ZeroEncoders))
                self.ci.send_can_message(switchCmd[idx], [self.steerMode, self.driveMode, self.publisherMode, data.ZeroEncoders])

    def locomotion_control(self, goal):
        # Append message CAN bus message feedback
        self._feedback.sequence = []

        rospy.loginfo("LC (in) \t x:%d \t y:%d \t rot:%d - translating..." % (goal.command.x, goal.command.y, goal.command.rz))
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



    def feedback_wait(self):
        # Helper variables
        r = rospy.Rate(500)
        # Clear orientation feedback flags
        feedback = [0, 0, 0, 0]
        while not any(feedback): #TODO change to all
                try:
                    [idx, flag] = self.ci.listener.lcMsgQueue.get(block=False)
                    self.ci.listener.lcMsgQueue.task_done()
                    feedback[idx] = flag
                except Queue.Empty:
                    r.sleep()
                    #rospy.loginfo("Message queue empty")
                if (self._as.is_preempt_requested() or rospy.is_shutdown()):
                    rospy.loginfo('LC \t %s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    break
        return feedback

    def orientation_control(self, wheelAngle):
        success = False
        with self.ci.listener.lcMsgQueue.mutex:         #TODO: change so interrupt handler is not blocked
            self.ci.listener.lcMsgQueue.queue.clear()
        rospy.loginfo('LC \t %s: Executing, orientation control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt wheel orientation
            orientation = CAN_Listener.wrap_message_format(wheelAngle[idx]/MAX_ORT)
            # Send CAN locomotion command
            rospy.loginfo("LC (out) \t %s wheel \t Orientation: %d" % (wheel, orientation))
            sent = self.ci.send_can_message(orientationCmd[idx], [orientation])
            self._feedback.sequence.append(sent)

        rospy.loginfo('LC \t %s: Waiting for orientation feedback' % (self._action_name))
        feedback = self.feedback_wait()
        self._feedback.sequence = self._feedback.sequence + feedback
        if any(feedback) : #all(self._feedback.sequence): #TODO change to all
            success = True
            # Save locomotion state
            self.wheelAngle = wheelAngle
        return success

    def velocity_control(self, wheelSpeed):
        success = False
        with self.ci.listener.lcMsgQueue.mutex:         #TODO: change so interrupt handler is not blocked
            self.ci.listener.lcMsgQueue.queue.clear()
        rospy.loginfo('LC \t %s: Executing, velocity control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt wheel velocity
            velocity = CAN_Listener.wrap_message_format(wheelSpeed[idx])
            # Send CAN locomotion command
            rospy.loginfo("LC (out) \t %s wheel \t Velocity: %d" % (wheel, velocity))
            sent = self.ci.send_can_message(velocityCmd[idx], [velocity])
            self._feedback.sequence.append(sent)

        rospy.loginfo('LC \t %s: Waiting for velocity feedback' % (self._action_name))
        feedback = self.feedback_wait()
        self._feedback.sequence = self._feedback.sequence + feedback
        if any(feedback) : #all(self._feedback.sequence): #TODO change to all
            success = True
            # Save locomotion state
            self.wheelSpeed = wheelSpeed
        return success

    # TODO: Adjust encoder feedback !!!!!!!!!!!!!!!!!!
    def get_encoder_odometry(self):
        # Get message values from listener
        msg = EncoderOdometry()
        msg.drive_pulses = Vector4Int(*(self.ci.listener.pulses))
        msg.drive_revolutions = Vector4Int(*(self.ci.listener.revolutions))
        activity = Vector4Int(*(self.ci.listener.activity[1:5]))
        msg.drive_velocity = Vector4([0,0,0,0])
        return msg


    def scale_feedback_values(value, type):
        """
        - type == 0: Max Orientation
        - type == 1: Max Velocity
        - type == 2: Max Current Battery
        - type == 3: Max Current Motor
        """
        value = value * self.MAX_RATING[type]
        return value


    def CAN_subscriber(self, event):
        # Check if nodes are initialised
        if self.lcInitialised:
            # Check for node failure
            if any(self.ci.listener.activity[1:5]) == 0:
                rospy.loginfo("LC \t Error CAN Drive node died")
                rospy.loginfo(' '.join(map(str, self.ci.listener.activity[1:5])))
            # Publish odometry message
            msg = self.get_encoder_odometry() # IMU data message
            self.encoder_pub.publish(msg)
                    # Check for node activity
        else:
            rospy.loginfo("LC \t Initialise CAN Drive nodes")
            self.lcInitialised = True
            self.drive_node_initialise()
        # Set node activity in listener
        for idx in range (1,5):
            self.ci.listener.activity[idx] = 0

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
        # Cancel publisher
        self.timer.shutdown()
        # Cancel CAN listener
        self.ci.listener.stop()

"""
Main
"""
if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("locomotion_control", anonymous=True)

    # Start ROS action
    server = LocomotionControl(rospy.get_name())
    rospy.spin()

rospy.on_shutdown(server.shutdown)

