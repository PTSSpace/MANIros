#!/usr/bin/python

#from __future__ import print_function
#import can
import struct
#import time
import rospy
import threading

# from maniros.msg import RoverControl      # Distance based control
from maniros.msg import MoveControl         # Velocity based control
from maniros.msg import MotorControl         # Velocity based control


# Velocity based vector protocol
from vector_protocol.vector_protocol import VectorTranslation


'''
Messages with lower numeric values for their ID's
have higher priority on the CAN network
''' 

# Message header ID's:
# Front left motor controller
FL_CMD = 0x0C1      # Locomotion command
FL_ODM = 0x0D1      # Odometry update
# Rear left motor controller
RL_CMD = 0x0C2      # Locomotion command
RL_ODM = 0x0D2      # Odometry update
# Rear right motor controller
RR_CMD = 0x0C3      # Locomotion command
RR_ODM = 0x0D3      # Odometry update
# Front right motor controller
FR_CMD = 0x0C4      # Locomotion command
FR_ODM = 0x0D4      # Odometry update


wheelIndexArray = ['front_left', 'rear_left', 'rear_right', 'front_right']
commandIdArray = [FL_CMD, RL_CMD, RR_CMD, FR_CMD]


stop_reading = threading.Event()

lock = threading.Lock()


def command_format(value):
    value = int(value) *1000
    return value

def send_can_message(arbitration_id, length, data):
    global lock

    #msg = can.Message(arbitration_id=arbitration_id,
    #              dlc= length, data=data,
    #              extended_id=False)

    lock.acquire()
    try:
        print "message sent"
        #bus.send(msg)
        #rospy.loginfo("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        rospy.loginfo("Message NOT sent")
    finally:
        lock.release()

def can_listen():
    global lock

    while  True:
        lock.acquire()
        try:
            print ("waiting for CAN message")
            #msg = bus.recv()
            print ("message received")
            #listener(msg)
            #r_msg = listener.get_message()
            #print (r_msg.arbitration_id)
            #print (struct.unpack('i', r_msg.data[0:4])[0])
            #print (struct.unpack('i', r_msg.data[4:8])[0])
        finally:
            lock.release()


def callback(data):
    #rospy.loginfo("Adapter: I've heard x:%d y:%d rot:%d - translating..." % (data.xDistance, data.yDistance, data.rotationDistance))

    [wheelSpeedArray, wheelAngleArray] = VectorTranslation(rover_length, rover_width).translateMoveControl(data)

    for index, wheel in enumerate(wheelIndexArray):
        # Extraxt wheel velocity and orientation
        velocity = command_format(wheelSpeedArray[index])
        orientation = command_format(wheelAngleArray[index])
        # Convert to bytes
        speed_data = struct.pack('i',velocity)
        orientation_data = struct.pack('i',orientation)
        # Send CAN locomotion command
        rospy.loginfo("Command: %s wheel \t Velocity: %d \t Orientation: %d" % (wheel, velocity, orientation))
        send_can_message(commandIDArray[index], 8, speed_data+orientation_data)



        

def can_publish():
    rospy.init_node("control_adapter", anonymous=True)
    sub = rospy.Subscriber("cmd_vel", MoveControl, callback, queue_size=10)
    rospy.spin()

def shutdown():
    listener.stop()
    canListenerThread.join()


if __name__ == '__main__':
    try:
        pub = rospy.Publisher("encoder_odometry", MotorControl, queue_size=1)
        # Get ros parameters
        rover_length = 10 #rospy.get_param("/rover_length")
        rover_width = 10 #rospy.get_param("/rover_width")

        # Start up CAN bus
        #bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)

        # Create listener queue
        #listener = can.BufferedReader()
        
        # Start CAN odometry listener thread
        canListenerThread = threading.Thread(target=can_listen)
        canListenerThread.daemon = True
        canListenerThread.start()

        # Start CAN command publisher thread
        can_publish()


    except rospy.ROSInterruptException:
        pass

rospy.on_shutdown(shutdown)
