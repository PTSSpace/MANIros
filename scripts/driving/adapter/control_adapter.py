#!/usr/bin/env python

import rospy


from maniros.msg import RoverControl
from maniros.msg import MotorControl

from vector_protocol.vector_protocol import VectorTranslation

pub = rospy.Publisher("motor_control", MotorControl, queue_size=10)

def callback(data):
    rospy.loginfo("Adapter: I've heard x:%d y:%d rot:%d - translating..." % (data.xDistance, data.yDistance, data.rotationDistance))
    
    #     robot_height = rospy.get_param("control_adapter/robot_height")
#     robot_width = rospy.get_param("control_adapter/robot_width")
    msg = VectorTranslation(10, 20).translateRoverControl(data)

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

# def normalizeArray(array):
#     maxvalue = abs(max(array, key=abs))
#     if maxvalue > 1:
#         for index, value in enumerate(array):
#             array[index] = value / maxvalue
#     return array

# def translateRoverControl(data):
#     msg = MotorControl()

#     robot_height = rospy.get_param("control_adapter/robot_height")
#     robot_width = rospy.get_param("control_adapter/robot_width")

#     #rotation, the scaled rotationDistance is multiplied with the width or height, returning a rotation component
#     #amd rotationHeightFactor = r_fac_x (as height is along the x axis)
#     #this rotationWidthFactor = r_fac_y
#     r_fac = data.rotationDistance / math.hypot(robot_width, robot_height)
#     r_fac_x = robot_height * r_fac
#     r_fac_y = robot_width * r_fac
    
#     #wheelIndexArray = [1, 2, 3, 4] see Motor Layout in KB for index, 1 = front-left then clockwise
#     wheelIndexArray = ['front_left', 'rear_left', "rear_right", "front_right"]
#     wheelSpeedArray = []
    
#     # rotationFactor  + translation (the vector xDistance, yDistance) = finalVector
#     for index, wheel in enumerate(wheelIndexArray):
#         if index <= 1: #assigns a negativ X Rotation to all left wheels
#             calcX = -r_fac_x + data.xDistance
#         else:
#             calcX = r_fac_x + data.xDistance
#         if 1 <= index <= 2: #assigns a negative Y Rotation to all rear wheels
#             calcY = -r_fac_y + data.yDistance
#         else:
#             calcY= r_fac_y + data.yDistance
        
#         angleXY = math.atan2(calcY, calcX)
#         speedXY = math.hypot(calcX, calcY)
#         #MISSING
#         #reverse speed if necessary, need Implementation of WHEN an angle should be turned
#         #this has to be determined by the hardware orientation of the servo
#         if math.fabs(angleXY) > (math.pi / 2): #this > (math.pi / 2) should be changed
#             angleXY -= math.copysign(math.pi, angleXY)
#             speedXY *= -1
        
#         setattr(msg, '%s_angle' % wheel, angleXY)
#         wheelSpeedArray.append(speedXY)

#     #normalize velocities
#     normalizeArray(wheelSpeedArray)
    
#     for index, wheel in enumerate(wheelIndexArray):
#         setattr(msg, '%s_speed' % wheel, wheelSpeedArray[index])

#     return msg;


if __name__ == '__main__':
    listener()
