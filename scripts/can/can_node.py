#!/usr/bin/python

from __future__ import print_function
import can
import struct
import time
import rospy
import threading
import math

# from maniros.msg import RoverControl      # Distance based control
from maniros.msg import MoveControl         # Velocity based control
from maniros.msg import MoveCommand         # Locomotion control switches
from maniros.msg import EncoderOdometry     # Encoder odometry feedback
from maniros.msg import Vector4             # Vector format for wheel messages


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
# Front left motor controller
FL_S        = 0x0A1             # Start/stop command
FL_I        = 0x0B1             # Initialization update
FL_CMD      = 0x0C1             # Locomotion command
FL_ODM      = 0x0D1             # Odometry update
# Rear left motor controller
RL_S_CMD    = 0x0A2             # Start/stop command
RL_I_CMD    = 0x0B2             # Initialization update
RL_CMD      = 0x0C2             # Locomotion command
RL_ODM      = 0x0D2             # Odometry update
# Rear right motor controller
RR_S        = 0x0A3             # Start/stop command
RR_I        = 0x0B3             # Initialization update
RR_CMD      = 0x0C3             # Locomotion command
RR_ODM      = 0x0D3             # Odometry update
# Front right motor controller
FR_S        = 0x0A4             # Start/stop command
FR_I        = 0x0B4             # Initialization update
FR_CMD      = 0x0C4             # Locomotion command
FR_ODM      = 0x0D4             # Odometry update

'''
wheelIndexArray = ['front_left', 'rear_left', 'rear_right', 'front_right']
commandIdArray = [FL_CMD, RL_CMD, RR_CMD, FR_CMD]
odometryIdArray = [FL_ODM, RL_ODM, RR_ODM, FR_ODM]
stopIdArray = [FL_S, RL_S, RR_S, FR_S]
initialliseIdArray = [FL_I, RL_I, RR_I, FR_I]
'''

wheelIndexArray = ['front_left'] #, 'rear_right']
commandIdArray = [FL_CMD] #, RR_CMD]
odometryIdArray = [FL_ODM] #, RR_ODM]
stopIdArray = [FL_S] #, RR_S]
initialliseIdArray = [FL_I] #, RR_I]

# Switch states
driveMode = False
steerMode = False
publisherMode = False

# Mutex lock to protect CAN interface
lock = threading.Lock()

# CAN Listener class to catch encoder odometry feedback
class CAN_Listener(can.Listener):
    def __init__(self):
        self.bus = bus
        self.initialised = False
        self.velocity = [.0, .0, .0, .0]
        self.orientation = [.0, .0, .0, .0]
        self.activity = [0, 0, 0, 0]

    def on_message_received(self, rxMsg):
        try:
            # Extract wheel data from the received CAN message
            idx =  [int(ID) for ID in odometryIdArray].index(rxMsg.arbitration_id)
            # Update node activity flag
            self.activity[idx] = 1
            self.velocity[idx] = unwrap_message_format(struct.unpack('i', rxMsg.data[0:4])[0], 0)
            self.orientation[idx] = unwrap_message_format(struct.unpack('i', rxMsg.data[4:8])[0], 1)
            rospy.loginfo("Message Source: %s \t Velocity: %1.3f \t Orientation: %3.3f" % (wheelIndexArray[idx], self.velocity[idx], self.orientation[idx]))
        except ValueError:
            rospy.loginfo("Message ID %d not in known list" % rxMsg.arbitration_id)

def wrap_message_format(value):
    # Scale to integer number [0..MaxValue]
    value = int(value) *MaxValue
    return value

def unwrap_message_format(value, type):
    # Scale to float number [0..1]
    value = float(value) /MaxValue
    if type:
        value *= MaxOrientation
    else:
        value *= MaxVelocity
    return value

def send_can_message(arbitration_id, length, data):
    global lock
    # Create CAN message
    txMsg = can.Message(arbitration_id=arbitration_id,
                  dlc= length, data=data,
                  extended_id=False)
    # Send CAN message
    with lock:
        try:
            bus.send(txMsg)
            rospy.loginfo("Message sent on {}".format(bus.channel_info))
        except can.CanError:
            rospy.loginfo("Message NOT sent")

def locomotion_switch(data):
    global steerMode, driveMode, publisherMode
    rospy.loginfo("Command \t Steer:%d \t Drive:%d" % (data.SteerPower, data.DrivePower))
    rospy.loginfo("Command \t Publisher:%d \t ZeroEncoders:%d" % (data.Publisher, data.ZeroEncoders))
    for index, wheel in enumerate(wheelIndexArray):
        # Toggle driving and steering switches
        if (data.DrivePower or data.SteerPower):
            if data.DrivePower:
                steerMode = not steerMode
            if data.SteerPower:
                driveMode = not driveMode
            # Motor start/stop command
            # Convert to bytes
            steer_data = struct.pack('i',int(steerMode))
            drive_data = struct.pack('i',int(driveMode))
            # Send CAN locomotion command
            rospy.loginfo("Command \t %s wheel \t Steer:%d \t Drive:%d" % (wheel, steerMode, driveMode))
            send_can_message(stopIdArray[index], 8, steer_data+drive_data)
        if (data.Publisher or data.ZeroEncoders):
            # Publisher start/stop and Zeroing command
            # Toggle publisher switch
            if data.Publisher:
                publisherMode = not publisherMode
            # Convert to bytes
            publish_data = struct.pack('i',int(publisherMode))
            zeroing_data = struct.pack('i',int(data.ZeroEncoders))
            # Send CAN locomotion command
            rospy.loginfo("Command \t %s wheel \t Publisher:%d \t Zero:%d" % (wheel, publisherMode, data.ZeroEncoders))
            send_can_message(initialliseIdArray[index], 8, publish_data+zeroing_data)

def locomotion_control(data):
    rospy.loginfo("Adapter: I've heard x:%d \t y:%d \t rot:%d - translating..." % (data.xSpeed, data.ySpeed, data.rotationAngle))
    # Convert velocity twist messages to individual wheel velocity and orientation
    [wheelSpeedArray, wheelAngleArray] = VectorTranslation(rover_length, rover_width).translateMoveControl(data)

    for index, wheel in enumerate(wheelIndexArray):
        # Extraxt wheel velocity and orientation
        velocity = wrap_message_format(wheelSpeedArray[index])
        orientation = wrap_message_format(wheelAngleArray[index]/MaxOrientation)
        # Convert to bytes
        speed_data = struct.pack('i',velocity)
        orientation_data = struct.pack('i',orientation)
        # Send CAN locomotion command
        rospy.loginfo("Command: %s wheel \t Velocity: %d \t Orientation: %d" % (wheel, velocity, orientation))
        send_can_message(commandIdArray[index], 8, speed_data+orientation_data)

def get_encoder_odometry():
    # Get message values from listener
    msg = EncoderOdometry()
    msg.velocity = Vector4(*(listener.velocity))
    msg.orientation = Vector4(*(listener.orientation))
    msg.activity = Vector4(*(listener.activity))
    return msg

def CAN_subscriber(event):
    global started, pub
   # print(listener.activity)
    # Check if nodes are initialised
#    if (any(listener.activity) and not listener.initialised):
#        rospy.loginfo("Initialise CAN nodes")
#        CAN_node_initialise()
#        listener.initialised = True
    # Check for node activity
    if any(listener.activity):
        # Publish odometry message
        msg = get_encoder_odometry() # IMU data message
        pub.publish(msg)
    else:
        # Check for node failure
        if listener.initialised:
            rospy.loginfo("Error CAN node died")
            rospy.loginfo(' '.join(map(str, listener.activity)))
    # Set node activity in listener
    listener.activity = [0, 0, 0, 0]

def CAN_node_initialise():
    # Initialise all driving nodes
    initMsg = MoveCommand()                                 # Initialisation message
    initMsg.SteerPower = True
    initMsg.DrivePower = True
    initMsg.Publisher = True
    initMsg.ZeroEncoders = True
    # Send CAN messages
    locomotion_switch(initMsg)

def can_interface():
    global timer
    # Start ROS node
    rospy.init_node("can_adapter", anonymous=True)
    # Subscribe to locomotion commands
    swtich_sub = rospy.Subscriber("teleop/lc_switch", MoveCommand, locomotion_switch, queue_size=10)
    move_sub = rospy.Subscriber("cmd_vel", MoveControl, locomotion_control, queue_size=10)

    # Start ROS publisher for encoder odometry
    timer = rospy.Timer(rospy.Duration(5), CAN_subscriber)

    rospy.spin()

def shutdown():
    global timer, listener
    timer.shutdown()
    listener.stop()

if __name__ == '__main__':
    pub = rospy.Publisher('encoder_odometry', EncoderOdometry, queue_size=10)

    # Get ros parameters
    rover_length = rospy.get_param("/rover_length")
    rover_width = rospy.get_param("/rover_width")

    # Start up CAN bus
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

    # Create listener
    listener = CAN_Listener()
    # Add notifyier to call listener periodically
    with lock:
        notifier = can.Notifier(bus, [listener])

    # Start CAN command publisher thread
    try:
        can_interface()
    except (rospy.ROSInterruptException, KeyboardInterrupt, SystemExit):
        pass

rospy.on_shutdown(shutdown)
