#!/usr/bin/env python
from __future__ import division
import rospy
from maniros.msg import MotorControl
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def callback(data):
    rospy.loginfo("Servo controller: I've heard fl:%d rl:%d rr:%d fr:%d. Moving Servos!" % (data.front_left_angle, data.rear_left_angle, data.rear_right_angle, data.front_right_angle)); 
    pwm.set_pwm(0, 0, rospy.get_param("/servo_controller/servo1_min"))

def listener():
    rospy.init_node("servo_controller", anonymous=True);
    sub = rospy.Subscriber("motor_control", MotorControl, callback);
    rospy.spin();

if __name__ == '__main__':
    listener();