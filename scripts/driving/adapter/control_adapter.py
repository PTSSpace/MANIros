#!/usr/bin/env python

import rospy


from maniros.msg import RoverControl
from maniros.msg import MotorControl

from vector_protocol.vector_protocol import VectorTranslation

pub = rospy.Publisher("motor_control", MotorControl, queue_size=10)

def translateRoverControl(data):

    msg = MotorControl()

    vt = VectorTranslation(
        rospy.get_param("control_adapter/robot_height"), 
        rospy.get_param("control_adapter/robot_width")
    )

    offsets = ['fl', 'rl', 'rr', 'fr']
    for index, wheel in enumerate(offsets):
        vt.wheelOffsetArray[index] = rospy.get_param("control_adapter/offset_%s" % wheel, 0)


    vt.calculateRotationFactor(data.rotationDistance)
    
    vt.addRotationAndTranslation(data.xDistance, data.yDistance)

    for index, wheel in enumerate(vt.wheelIndexArray):
        setattr(msg, '%s_angle' % wheel, vt.wheelAngleArray[index])
        setattr(msg, '%s_speed' % wheel, vt.wheelSpeedArray[index])

    return msg

def callback(data):
    rospy.loginfo("Adapter: I've heard x:%d y:%d rot:%d - translating..." % (
        data.xDistance, 
        data.yDistance, 
        data.rotationDistance
    ))
    
    msg = translateRoverControl(data)
    pub.publish(msg)

    rospy.loginfo("Adapter: I've sent angles fl:%.3f rl:%.3f rr:%.3f fr:%.3f" % (
        msg.front_left_angle, 
        msg.rear_left_angle, 
        msg.rear_right_angle, 
        msg.front_right_angle
    ))
    rospy.loginfo("Adapter: I've sent speeds fl:%.3f rl:%.3f rr:%.3f fr:%.3f" % (
        msg.front_left_speed, 
        msg.rear_left_speed, 
        msg.rear_right_speed, 
        msg.front_right_speed
    )) 
    
def listener():
    rospy.init_node("control_adapter", anonymous=True)
    sub = rospy.Subscriber("rover_control", RoverControl, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
