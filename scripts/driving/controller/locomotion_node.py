#!/usr/bin/python
from __future__ import print_function

"""
This program provides a ROS node for accessing the can0 bus interface.
It connects the locomotion nodes to the ROS network.
Velocitity and general motor commands are forwarded to the drive nodes.
Encoder odometry messages are received from the drive nodes and published to the ROS network.
All variables are converted into the specific units and scaled according to the definition
given in the can_interface. The commands are derived from the can_protocol.

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
from can_bus.can_protocol import *
# Import CAN interface
from can_bus.can_interface import CANInterface

# Import ROS messages
from maniros.msg import MoveCommand                                                 # Locomotion control switches
from maniros.msg import EncoderOdometry                                             # Encoder odometry feedback
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
        self.rover_length       = rospy.get_param("/rover_length")
        self.rover_width        = rospy.get_param("/rover_width")
        self.MAX_VEL            = rospy.get_param("/max_vel")                   # Maximal wheel velocity [rad/s]
        self.MAX_ORT            = rospy.get_param("/max_ort")                   # Maximal wheel orientation [rad]
        self.DRIVE_ENC_PPR      = rospy.get_param("/drive_enc_ppr")             # Drive encoder pulses per revolution
        self.STEER_ENC_PPR      = rospy.get_param("/steer_enc_ppr")             # Steer encoder pulses per revolution

        # Switch states
        self.lcInitialised      = False
        self.driveMode          = False
        self.steerMode          = False
        self.publisherMode      = False
        self.driving            = False
        self.lcError            = False
        self.wheelSpeed         = [0, 0, 0, 0]
        self.wheelAngle         = [self.MAX_ORT*5, self.MAX_ORT*5, self.MAX_ORT*5, self.MAX_ORT*5]                      # Imposible position for start orientation

        # Construct CAN bus interface
        self.ci = CANInterface()

        # Encoder odometry publisher
        self.encoder_pub = rospy.Publisher("encoder_odometry", EncoderOdometry, queue_size=10)
        # Subscribe to locomotion state commands
        self.switch_sub = rospy.Subscriber("teleop/lc_switch", MoveCommand, self.locomotion_switch, queue_size=10)

        # Start ROS publisher for encoder odometry
        self.timer = rospy.Timer(rospy.Duration(10), self.encoder_publisher)

        # Locomotion control action
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LocomotionAction, execute_cb=self.locomotion_control, auto_start = False)
        self._as.start()


    def locomotion_switch(self, data):
        """
        Callback for locomotion switches for turning on steer and drive motor/PID control,
        turning on odometry publisher and zeroing encoders
        :param data: MoveCommand ROS message
        """
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
        """
        Locomotion control action translating rover base movements to individual wheel
        orientation and velocity commands and forwarding these to the wheel controllers.
        Locomotion control logic:
        Case 1)
            * Wheel orientation matches the current state
            -> Control wheel velocity
        Case 2)
            * Wheel orientation differs from current state
            * Wheel velocity is zero
            -> Control wheel orientation
            -> Control wheel velocity
        Case 3)
            * Wheel orientation differs from current state
            * Wheel velocity is not zero
            -> Set wheel velocity to zero
            -> Control wheel orientation
            -> Control wheel velocity
        Case 4)
            * Wheel orientation matches the current state
            * lcError flag is set (previous locomotion command failed)
            * Wheel velocity is zero
            -> Case 2)
        Case 5)
            * Wheel orientation matches the current state
            * lcError flag is set (previous locomotion command failed)
            * Wheel velocity is not zero
            -> Case 3)
        The commands are called in the listed order above, each command is only executed
        if the previous one has been accomplished and feedback has been received from the
        wheel controllers.
        :param data: MoveControl ROS message
        """
        rospy.loginfo("LC (in) \t x:%f \t y:%f \t rot:%f - translating..." % (goal.command.x, goal.command.y, goal.command.rz))
        # Append message CAN bus message feedback
        self._feedback.sequence = []
        success = False

        # Convert velocity twist messages to individual wheel velocity and orientation
        [wheelSpeedNorm, wheelAngle] = VectorTranslation(self.rover_length, self.rover_width).translateMoveControl(goal.command)
        wheelSpeed = [value * self.MAX_VEL for value in wheelSpeedNorm]         # Scale wheel velocity

        # Locomotion control logic
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
            # Clear error flag
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

    def check_preempt(self):
        """
        Check for preemtion via the action server.
        """
        success = True
        if (self._as.is_preempt_requested() or rospy.is_shutdown()):
            rospy.loginfo('LC \t %s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        return success

    def feedback_wait(self):
        """
        Wait for command completed feedback from all wheel controllers.
        """
        success = True
        r = rospy.Rate(500)
        # Clear orientation feedback flags
        feedback = [0, 0, 0, 0]
        while not any(feedback) and success: #TODO change to all
                try:
                    # Get incoming command acomplished feedback from can message queue
                    [idx, flag] = self.ci.listener.lcMsgQueue.get(block=False)
                    self.ci.listener.lcMsgQueue.task_done()
                    # Set feedback flag
                    feedback[idx] = flag
                except Queue.Empty:
                    r.sleep()
                # Check for action preemtion
                success = self.check_preempt()
        return feedback

    def orientation_control(self, wheelAngle):
        """
        Wheel orientation control process. Forwards wheel orientation commands
        to CAN bus and waits for command completed feedback from wheel controllers.
        Regularly checks for ROS action preemtion.
        :param wheelAngle: List of wheel orientations in [rad]
        """
        success = False
        with self.ci.listener.lcMsgQueue.mutex:         #TODO: change so interrupt handler is not blocked
            self.ci.listener.lcMsgQueue.queue.clear()
        rospy.loginfo('LC \t %s: Executing, orientation control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt wheel orientation
            # Convert orientation: rad -> pulses
            # Shift by a quater rotation that encoder pulses are only positive
            orientation = int((wheelAngle[idx]/(math.pi/2.0)+1.0)*(self.STEER_ENC_PPR/4.0))
            # Send CAN locomotion command
            rospy.loginfo("LC (out) \t %s wheel \t Orientation: %d" % (wheel, orientation))
            sent = self.ci.send_can_message(orientationCmd[idx], [orientation])
            self._feedback.sequence.append(sent)

        # Check if orientation command has been accomplished
        rospy.loginfo('LC \t %s: Waiting for orientation feedback' % (self._action_name))
        feedback = self.feedback_wait()
        self._feedback.sequence = self._feedback.sequence + feedback
        if any(feedback) : #all(self._feedback.sequence): #TODO change to all
            success = True
            # Save locomotion state
            self.wheelAngle = wheelAngle
        return success

    def velocity_control(self, wheelSpeed):
        """
        Wheel velocity control process. Forwards wheel velocity commands
        to CAN bus and waits for command completed feedback from wheel controllers.
        Regularly checks for ROS action preemtion.
        :param wheelSpeed: List of wheel velocities in [rad/s]
        """
        success = False
        with self.ci.listener.lcMsgQueue.mutex:         #TODO: change so interrupt handler is not blocked
            self.ci.listener.lcMsgQueue.queue.clear()
        rospy.loginfo('LC \t %s: Executing, velocity control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt wheel velocity
             # Convert velocity: rad/s -> pulses/s
            velocity = int(wheelSpeed[idx]/(2.0*math.pi)*self.DRIVE_ENC_PPR)
            # Send CAN locomotion command
            rospy.loginfo("LC (out) \t %s wheel \t Velocity: %d" % (wheel, velocity))
            sent = self.ci.send_can_message(velocityCmd[idx], [velocity])
            self._feedback.sequence.append(sent)

        # Check if velocity command has been accomplished
        rospy.loginfo('LC \t %s: Waiting for velocity feedback' % (self._action_name))
        feedback = self.feedback_wait()
        self._feedback.sequence = self._feedback.sequence + feedback
        if any(feedback) : #all(self._feedback.sequence): #TODO change to all
            success = True
            # Save locomotion state
            self.wheelSpeed = wheelSpeed
        return success

    def get_encoder_odometry(self):
        """
        Get encoder feedback from CAN listener and write into odometry ROS message.
        """
        # Get message values from listener
        msg = EncoderOdometry()
        msg.steer_pulses        = self.ci.listener.orientation      # Steer motor orientation [pulses]
        msg.drive_pulses        = self.ci.listener.pulses           # Drive motor orientation [pulses]
        msg.drive_revolutions   = self.ci.listener.revolutions      # Drive motor revolutions
        msg.drive_velocity      = self.ci.listener.velocity         # Drive motor velocity [pulses/s]
        msg.steer_velocity      = [None, None, None, None]          # Steer motor velocity TODO: Not part of wheel controller feedback
        msg.activity            = self.ci.listener.activity[1:5]    # Wheel controller node activity
        return msg


    def scale_feedback_values(value, type):
        """
        Scale normed values from messages to their maximal value.
        - type == 0: Max Orientation
        - type == 1: Max Velocity
        - type == 2: Max Current Battery
        - type == 3: Max Current Motor
        """
        value = value * self.MAX_RATING[type]
        return value


    def encoder_publisher(self, event):
        """
        Publisher for encoder feedback from CAN listener class.
        Regularly publishes encoder values to the ROS system.
        initialises wheel controllers and checks for their activity
        via their publish rate.
        """
        # Check if nodes are initialised
        if self.lcInitialised:
            # Check for node activity/failure
            if any(self.ci.listener.activity[1:5]) == 0:
                rospy.loginfo("LC \t Error CAN Drive node died")
                rospy.loginfo(' '.join(map(str, self.ci.listener.activity[1:5])))
            # Publish odometry message
            msg = self.get_encoder_odometry() # IMU data message
            self.encoder_pub.publish(msg)
        else:
            # Initialise wheel controllers
            rospy.loginfo("LC \t Initialise CAN Drive nodes")
            self.lcInitialised = True
            self.drive_node_initialise()
        # Reset node activity in listener
        for idx in range (1,5):
            self.ci.listener.activity[idx] = 0

    def drive_node_initialise(self):
        """
        Initialisation message for rover.
        Turns all motor control states and odometry publisher on
        and zeroes encoders.
        """
        # Initialise all driving nodes
        initMsg = MoveCommand()                                 # Initialisation message
        initMsg.SteerPower = True
        initMsg.DrivePower = True
        initMsg.Publisher = True
        initMsg.ZeroEncoders = True
        # Send CAN messages
        self.locomotion_switch(initMsg)

    def shutdown(self):
        """
        Shutdown procedure for ROS node.
        Stops timer and can listener interface.
        """
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

