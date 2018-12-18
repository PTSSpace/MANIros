#!/usr/bin/env python
import rospy
from maniros.msg import RoverControl
from maniros.msg import MotorControl
import math

pub = rospy.Publisher("motor_control", MotorControl, queue_size=10)

def callback(data):
    rospy.loginfo("Adapter: I've heard x:%d y:%d rot:%d - translating..." % (data.xDistance, data.yDistance, data.rotationDistance));
    msg = translateRoverControl(data);
    pub.publish(msg);
    rospy.loginfo("Adapter: I've sent angles fl:%d rl:%d rr:%d fr:%d" % (msg.front_left_angle, msg.rear_left_angle, msg.rear_right_angle, msg.front_right_angle));
    rospy.loginfo("Adapter: and velocities fl:%d rl:%d rr:%d fr:%d" % (msg.front_left_speed, msg.rear_left_speed, msg.rear_right_speed, msg.front_right_speed));

def listener():
    rospy.init_node("control_adapter", anonymous=True);
    sub = rospy.Subscriber("rover_control", RoverControl, callback)
    rospy.spin()

def translateRoverControl(data):
    msg = MotorControl();

    #MISSING
    #import some config file containing the wheel-center distance between 
    #the front and back wheels (height) and 
    #the left and right wheels (width)
    
    robot_height = 10
    robot_width = 5

    #rotation, the scaled rotationDistance is multiplied with the width or height, returning a rotation component
    #this rotationHeightFactor = r_fac_x (as height is along the x axis)
    #amd rotationWidthFactor = r_fac_y
    r_fac_x = (robot_height * data.rotationDistance) / math.hypot(robot_width, robot_height)
    r_fac_y = (robot_width * data.rotationDistance) / math.hypot(robot_width, robot_height)

    wheelIndexArray = [1, 2, 3, 4] #see Motor Layout in KB for index, 1 = front-left then clockwise
    wheelAngleArray = []
    wheelSpeedArray = []
    for wheel in wheelIndexArray:
        
        if 2 <= wheel <= 3: #assigns a positiv X Rotation to all rear wheels
            calcX = r_fac_x + data.xDistance
        else:
            calcX = -r_fac_x + data.xDistance
        if wheel <= 2: #assigns a negativ Y Rotation to all left wheels
            calcY = -r_fac_y + data.yDistance
        else:
            calcY = r_fac_y + data.yDistance
        
        wheelAngleArray.append(calcY) #math.atan2(calcY, calcX))
        wheelSpeedArray.append(math.hypot(calcX, calcY))
        
        
    #translation (the vector xDistance, yDistance) + rotationFactor = finalVector


   
    msg.front_left_angle = wheelAngleArray[0] * 1000
    msg.rear_left_angle = wheelAngleArray[1] * 1000
    msg.rear_right_angle = wheelAngleArray[2] * 1000
    msg.front_right_angle = wheelAngleArray[3] * 1000

    msg.front_left_speed = wheelSpeedArray[0] * 1000
    msg.rear_left_speed = wheelSpeedArray[1] * 1000
    msg.rear_right_speed = wheelSpeedArray[2] * 1000
    msg.front_right_speed = wheelSpeedArray[3] * 1000

    return msg;

if __name__ == '__main__':
    listener();