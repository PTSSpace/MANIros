#!/usr/bin/python

from __future__ import print_function
import rospy
import actionlib
import threading
import can
import struct
import time
import math

# Import ROS messages
#from maniros.msg import MoveControl         # Velocity based control
from maniros.msg import MoveCommand         # Locomotion control switches
from maniros.msg import EncoderOdometry     # Encoder odometry feedback
from maniros.msg import Vector4             # Vector format for wheel messages
# Locomotion control action
from maniros.msg import LocomotionAction
from maniros.msg import LocomotionFeedback
from maniros.msg import LocomotionResult

# Velocity based vector protocol
from vector_protocol.vector_protocol import VectorTranslation

# Set motor parameters
MaxVelocity     = 3             # Maximal wheel velocity [rps]
MaxOrientation  = math.pi/2     # Maximal wheel orientation [rad]

# Protocol parameters
MaxValue        = 2147483647    # Maximal signed value for 4 bytes

'''
Messages with lower numeric values for their ID's
have higher priority on the CAN network
'''

# Message header ID's:
'''
wheelIndex          = ['front_left', 'rear_left', 'rear_right', 'front_right']      # Wheel location on rover
powerCmd            = [0x0A1, 0x0A2, 0x0A3, 0x0A4]                                  # Motor/PID start/stop command
initialliseCmd      = [0x0B1, 0x0B2, 0x0B3, 0x0B4]                                  # Initialization and odometry publisher command
orientationCmd      = [0x0C1, 0x0C2, 0x0C3, 0x0C4]                                  # Set orientation locomotion command
velocityCmd         = [0x0D1, 0x0D2, 0x0D3, 0x0D4]                                  # Set velocity locomotion command
orientationOdm      = [0x0E1, 0x0E2, 0x0E3, 0x0E4]                                  # Orientation odometry locomotion feedback
velocityOdm         = [0x0F1, 0x0F2, 0x0F3, 0x0F4]                                  # Velocity odometry locomotion feedback
'''


wheelIndex          = ['front_left']
powerCmd            = [0x0A1]
initialliseCmd      = [0x0B1]
orientationCmd      = [0x0C1]
velocityCmd         = [0x0D1]
orientationOdm      = [0x0E1]
velocityOdm         = [0x0F1]

FREQUENCY           = 500000              # CAN bus bit rate

# CAN Listener class to catch encoder odometry feedback
class CAN_Listener(can.Listener):
    def __init__(self):
        self.initialised    = False
        self.activity       = [0, 0, 0, 0]
        self.steer          = [0, 0, 0, 0]
        self.orientation    = [.0, .0, .0, .0]
        self.pulses         = [0, 0, 0, 0]
        self.revolutions    = [0, 0, 0, 0]

    def on_message_received(self, rxMsg):
        ID = rxMsg.arbitration_id
        # Extract wheel data from the received CAN message
        if ID in orientationOdm:
            idx =  [int(x) for x in orientationOdm].index(ID)
            self.orientation[idx] = self.unwrap_message_format(struct.unpack('i', rxMsg.data[0:4])[0], 0)
            # Set position reached flag
            self.steer[idx] = 1
            rospy.loginfo("Message Source: %s \t Orientation: %3.3f" % (wheelIndex[idx], self.orientation[idx]))
            # Update node activity flag
            self.activity[idx] = 1
        elif ID in velocityOdm:
            idx =  [int(x) for x in velocityOdm].index(ID)
            self.pulses[idx] = struct.unpack('i', rxMsg.data[0:4])[0]
            self.revolutions[idx] = struct.unpack('i', rxMsg.data[4:8])[0]
            rospy.loginfo("Message Source: %s \t Pulses: %d \t Revolutions: %d" % (wheelIndex[idx], self.pulses[idx], self.revolutions[idx]))
            # Update node activity flag
            self.activity[idx] = 1
        else:
            rospy.loginfo("Message ID %d not in known list" % ID)

    def unwrap_message_format(self, value, type):
        # Scale to float number [0..1]
        value = float(value) /MaxValue
        if type:
            value *= MaxOrientation
        else:
            value *= MaxVelocity
        return value


class CANInterface():
    """docstring for can_interface"""
    def __init__(self):
        # Mutex lock to protect CAN interface
        self.lock = threading.Lock()
        # Start up CAN bus
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=FREQUENCY)

        # Create listener
        self.listener = CAN_Listener()
        # Add notifyier to call listener periodically
        with self.lock:
            self.notifier = can.Notifier(self.bus, [self.listener])

    def send_can_message(self, arbitration_id, length, data):
        # Create CAN message
        txMsg = can.Message(arbitration_id=arbitration_id,
                      dlc= length, data=data,
                      extended_id=False)
        # Send CAN message
        with self.lock:
            try:
                self.bus.send(txMsg)
                rospy.loginfo("Message sent on {}".format(self.bus.channel_info))
                return 1
            except can.CanError:
                rospy.loginfo("Message NOT sent")
                return 0


class LocomotionControl(object):
    """Locomotion control server for CAN bus communication with wheel controllers"""
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

        # Encoder odometry publisher
        self.encoder_pub = rospy.Publisher('encoder_odometry', EncoderOdometry, queue_size=10)
        # Subscribe to locomotion commands
        self.switch_sub = rospy.Subscriber("teleop/lc_switch", MoveCommand, self.locomotion_switch, queue_size=10)

        # Start ROS publisher for encoder odometry
        self.timer = rospy.Timer(rospy.Duration(10), self.CAN_subscriber)

        # Locomotion control action
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, LocomotionAction, execute_cb=self.locomotion_control, auto_start = False)
        self._as.start()

    def locomotion_switch(self, data):
        # Locomotion commands subscriber callback
        rospy.loginfo("Command \t Steer:%d \t Drive:%d" % (data.SteerPower, data.DrivePower))
        rospy.loginfo("Command \t Publisher:%d \t ZeroEncoders:%d" % (data.Publisher, data.ZeroEncoders))
        for idx, wheel in enumerate(wheelIndex):
            # Toggle driving and steering switches
            if (data.DrivePower or data.SteerPower):
                if data.DrivePower:
                    self.steerMode = not self.steerMode
                if data.SteerPower:
                    self.driveMode = not self.driveMode
                # Motor start/stop command
                # Convert to bytes
                steer_data = struct.pack('i',int(self.steerMode))
                drive_data = struct.pack('i',int(self.driveMode))
                # Send CAN locomotion command
                rospy.loginfo("Command \t %s wheel \t Steer:%d \t Drive:%d" % (wheel, self.steerMode, self.driveMode))
                self.ci.send_can_message(powerCmd[idx], 8, steer_data+drive_data)
                rospy.sleep(0.05)
            if (data.Publisher or data.ZeroEncoders):
                # Publisher start/stop and Zeroing command
                # Toggle publisher switch
                if data.Publisher:
                    self.publisherMode = not self.publisherMode
                # Convert to bytes
                publish_data = struct.pack('i',int(self.publisherMode))
                zeroing_data = struct.pack('i',int(data.ZeroEncoders))
                # Send CAN locomotion command
                rospy.loginfo("Command \t %s wheel \t Publisher:%d \t Zero:%d" % (wheel, self.publisherMode, data.ZeroEncoders))
                self.ci.send_can_message(initialliseCmd[idx], 8, publish_data+zeroing_data)

    def locomotion_control(self, goal):
        # Append message CAN bus message feedback
        self._feedback.sequence = []

        rospy.loginfo("Adapter: I've heard x:%d \t y:%d \t rot:%d - translating..." % (goal.command.xSpeed, goal.command.ySpeed, goal.command.rotationAngle))
        # Clear orientation feedback  flags
        self.ci.listener.steer = [0, 0, 0, 0]
        # Convert velocity twist messages to individual wheel velocity and orientation
        [wheelSpeedArray, wheelAngleArray] = VectorTranslation(rover_length, rover_width).translateMoveControl(goal.command)

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
            orientation = self.wrap_message_format(wheelAngleArray[idx]/MaxOrientation)
            # Convert to bytes
            data = struct.pack('i',orientation)
            # Send CAN locomotion command
            rospy.loginfo("Command: %s wheel \t Orientation: %d" % (wheel, orientation))
            sent = self.ci.send_can_message(orientationCmd[idx], 4, data)
            self._feedback.sequence.append(sent)

        rospy.loginfo('%s: Waiting for orientation feedback' % (self._action_name))
        while not any(self.ci.listener.steer): #TODO change to all
            if (self._as.is_preempt_requested() or rospy.is_shutdown()):
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            r.sleep()
        return success

    def velocity_control(self, wheelSpeedArray):
        rospy.loginfo('%s: Executing, velocity control' % (self._action_name))
        for idx, wheel in enumerate(wheelIndex):
            # Extraxt wheel velocity
            velocity = self.wrap_message_format(wheelSpeedArray[idx])
            # Convert to bytes
            data = struct.pack('i',velocity)
            # Send CAN locomotion command
            rospy.loginfo("Command: %s wheel \t Velocity: %d" % (wheel, velocity))
            sent = self.ci.send_can_message(velocityCmd[idx], 4, data)
            self._feedback.sequence.append(sent)

    def get_encoder_odometry(self):
        # Get message values from listener
        msg = EncoderOdometry()
        msg.pulses = Vector4(*(self.ci.listener.pulses))
        msg.revolutions = Vector4(*(self.ci.listener.revolutions))
        msg.activity = Vector4(*(self.ci.listener.activity))
        return msg

    def wrap_message_format(self, value):
        # Scale to integer number [0..MaxValue]
        value = int(value) *MaxValue
        return value


    def CAN_subscriber(self, event):
        # Check if nodes are initialised
        #if (any(self.ci.listener.activity) and not self.ci.listener.initialised): #TODO change to all
        if not self.ci.listener.initialised:
            rospy.loginfo("Initialise CAN nodes")
            self.CAN_node_initialise()
            self.ci.listener.initialised = True
        else:
            # Check for node activity
            if any(self.ci.listener.activity):
                # Publish odometry message
                msg = self.get_encoder_odometry() # IMU data message
                self.encoder_pub.publish(msg)
            else:
                # Check for node failure
                if self.ci.listener.initialised:
                    rospy.loginfo("Error CAN node died")
                    rospy.loginfo(' '.join(map(str, self.ci.listener.activity)))
        # Set node activity in listener
        self.ci.listener.activity = [0, 0, 0, 0]

    def CAN_node_initialise(self):
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


if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("can_node", anonymous=True)
    # Get ros parameters
    rover_length = rospy.get_param("/rover_length")
    rover_width = rospy.get_param("/rover_width")

    # Start ROS action
    server = LocomotionControl(rospy.get_name())
    rospy.spin()

rospy.on_shutdown(server.shutdown)

