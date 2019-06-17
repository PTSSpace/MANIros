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
import rospy
import actionlib
import time
import threading
import queue

# Import CAN protocol parameters
from can_protocol import *
# Import CAN interface
from can_interface import CANInterface

# Import ROS messages
from maniros.msg import MoveCommand                                                 # Locomotion control switches
from maniros.msg import EncoderOdometry                                             # Encoder odometry feedback
from maniros.msg import Vector4                                                     # Vector format for wheel messages
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
        # Switch states
        self.driveMode = False
        self.steerMode = False
        self.publisherMode = False
        self.driving = False

        # Construct CAN bus interface
        self.ci = CANInterface()

        t1 = threading.Thread(name='block', 
                              target=self.wait_for_event,
                              args=(self.ci.listener.lc,))
        t1.start()

        # Get ros parameters
        self.rover_length = rospy.get_param("/rover_length")
        self.rover_width = rospy.get_param("/rover_width")

        # Encoder odometry publisher
        self.encoder_pub = rospy.Publisher("encoder_odometry", EncoderOdometry, queue_size=10)
        # Subscribe to locomotion commands
        self.switch_sub = rospy.Subscriber("teleop/lc_switch", MoveCommand, self.locomotion_switch, queue_size=10)

        # Start ROS publisher for encoder odometry
        self.timer = rospy.Timer(rospy.Duration(10), self.CAN_subscriber)

        # Locomotion control action
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LocomotionAction, execute_cb=self.locomotion_control, auto_start = False)
        self._as.start()

    def wait_for_event(self, e):
        """Wait for the event to be set before doing anything"""
        print ('wait_for_event starting')
        event_is_set = e.wait()
        print ('event set: %s', event_is_set)


    def locomotion_switch(self, data):
        # Locomotion commands subscriber callback
        rospy.loginfo("Command \t Steer:%d \t Drive:%d" % (data.SteerPower, data.DrivePower))
        rospy.loginfo("Command \t Publisher:%d \t ZeroEncoders:%d" % (data.Publisher, data.ZeroEncoders))
        for idx, wheel in enumerate(wheelIndex):
            if (any(data)):
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
                rospy.loginfo("Command \t %s wheel \t Steer:%d \t Drive:%d \t Publisher:%d \t Zero:%d"
                    % (wheel, self.steerMode, self.driveMode, self.publisherMode, data.ZeroEncoders))
                self.ci.send_can_message(switchCmd[idx], [self.steerMode, self.driveMode, self.publisherMode, data.ZeroEncoders])

    def locomotion_control(self, goal):
        # Append message CAN bus message feedback
        self._feedback.sequence = []

        rospy.loginfo("Adapter: I've heard x:%d \t y:%d \t rot:%d - translating..." % (goal.command.xSpeed, goal.command.ySpeed, goal.command.rotationAngle))
        # Clear orientation feedback  flags
        steer = [0, 0, 0, 0]
        # Convert velocity twist messages to individual wheel velocity and orientation
        [wheelSpeedArray, wheelAngleArray] = VectorTranslation(self.rover_length, self.rover_width).translateMoveControl(goal.command)

        # Check if rover is currently driving
        if self.driving:
            self.velocity_control(wheelSpeedArray)
            success = self.orientation_control(wheelAngleArray)
        else:
            success = self.orientation_control(wheelAngleArray)
            if success:
                self.velocity_control(wheelSpeedArray)
        self.driving = any(wheelSpeedArray)

        # publish the feedback
        self._as.publish_feedback(self._feedback)
        if success:
            if all(self._feedback.sequence):
                rospy.loginfo('%s: Succeeded' % self._action_name)
            else:
                success = False
                rospy.loginfo('%s: Failed' % self._action_name)

            self._result.sequence = self._feedback.sequence
            self._as.set_succeeded(self._result)


    def orientation_control(self, wheelAngleArray):
        success = True
        # Helper variables
        r = rospy.Rate(100)
        rospy.loginfo('%s: Executing, orientation control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt wheel orientation
            orientation = LocomotionControl.wrap_message_format(wheelAngleArray[idx]/MAX_ORT)
            # Send CAN locomotion command
            rospy.loginfo("Command: %s wheel \t Orientation: %d" % (wheel, orientation))
            sent = self.ci.send_can_message(orientationCmd[idx], [orientation])
            self._feedback.sequence.append(sent)

        rospy.loginfo('%s: Waiting for orientation feedback' % (self._action_name))
        while not any(steer):#any(self.ci.listener.steer): #TODO change to all
            try:
                [idx, orientation] = self.ci.listener.lcMsgQueue.get()
                epsMsgQueue.task_done()
                steer[idx] = orientation
            except queue.Empty:
                rospy.loginfo("Message queue empty")
            if (self._as.is_preempt_requested() or rospy.is_shutdown()):
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
        return success

    def velocity_control(self, wheelSpeedArray):
        rospy.loginfo('%s: Executing, velocity control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt wheel velocity
            velocity = LocomotionControl.wrap_message_format(wheelSpeedArray[idx])
            # Send CAN locomotion command
            rospy.loginfo("Command: %s wheel \t Velocity: %d" % (wheel, velocity))
            sent = self.ci.send_can_message(velocityCmd[idx], velocity)
            self._feedback.sequence.append(sent)

    def get_encoder_odometry(self):
        # Get message values from listener
        msg = EncoderOdometry()
        msg.pulses = Vector4(*(self.ci.listener.pulses))
        msg.revolutions = Vector4(*(self.ci.listener.revolutions))
        msg.activity = Vector4(*(self.ci.listener.activity))
        return msg

    @staticmethod
    def wrap_message_format(value):
        # Scale to integer number [0..MAX_VALUE]
        value = int(value) *MAX_VALUE
        return value


    def CAN_subscriber(self, event):
        # Check if nodes are initialised
        #if (any(self.ci.listener.activity) and not self.ci.listener.initialised): #TODO change to all
        if not self.ci.listener.lcInitialised:
            rospy.loginfo("Initialise CAN Drive nodes")
            self.Drive_node_initialise()
            self.ci.listener.lcInitialised = True
        else:
            # Check for node activity
            if any(self.ci.listener.activity[1:4]):
                # Publish odometry message
                msg = self.get_encoder_odometry() # IMU data message
                self.encoder_pub.publish(msg)
            else:
                # Check for node failure
                if self.ci.listener.lcInitialised:
                    rospy.loginfo("Error CAN Drive node died")
                    rospy.loginfo(' '.join(map(str, self.ci.listener.activity[1:4])))
        # Set node activity in listener
        self.ci.listener.activity[1:4] = [0, 0, 0, 0]

    def Drive_node_initialise(self):
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

