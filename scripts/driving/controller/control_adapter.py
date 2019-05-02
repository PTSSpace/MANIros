#!/usr/bin/python

import rospy

# from maniros.msg import RoverControl # Distance based control
from maniros.msg import MoveControl # Speed based control
from maniros.msg import MotorControl

# Velocity based vector protocol
from vector_protocol.vector_protocol import VectorTranslation

def callback(data):
    #rospy.loginfo("Adapter: I've heard x:%d y:%d rot:%d - translating..." % (data.xDistance, data.yDistance, data.rotationDistance))

    [wheelSpeedArray, wheelAngleArray] = VectorTranslation(rover_length, rover_width).translateMoveControl(data)

    # Create motor control message
    msg = MotorControl()
    msg.front_left_angle = wheelAngleArray[0]
    msg.rear_left_angle = wheelAngleArray[1]
    msg.rear_right_angle = wheelAngleArray[2]
    msg.front_right_angle = wheelAngleArray[3]

    msg.front_left_speed = wheelSpeedArray[0]
    msg.rear_left_speed = wheelSpeedArray[1]
    msg.rear_right_speed = wheelSpeedArray[2]
    msg.front_right_speed = wheelSpeedArray[3]


    pub.publish(msg)
    rospy.loginfo("W_angle \t fl:%.3f rl:%.3f rr:%.3f fr:%.3f" % (
        msg.front_left_angle, 
        msg.rear_left_angle, 
        msg.rear_right_angle, 
        msg.front_right_angle
    ))
    rospy.loginfo("W_speed \t fl:%.3f rl:%.3f rr:%.3f fr:%.3f" % (
        msg.front_left_speed, 
        msg.rear_left_speed, 
        msg.rear_right_speed, 
        msg.front_right_speed
    )) 

def listener():
    rospy.init_node("control_adapter", anonymous=True)
    sub = rospy.Subscriber("cmd_vel", MoveControl, callback, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher("motor_control", MotorControl, queue_size=1)
    # get ros parameters
    trans_scl = rospy.get_param("/teleop/translation_scale")      # translational speed scaling factor
    rover_length = rospy.get_param("/rover_length")
    rover_width = rospy.get_param("/rover_width")
    listener()