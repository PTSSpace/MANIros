#!/usr/bin/env python
from __future__ import division
import rospy
from maniros.msg import MotorControl
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

# Configure min and max servo pulse lengths
servo_min = 1024  # Min pulse length out of 4096
servo_max = 3072  # Max pulse length out of 4096

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def callback(data):
    rospy.loginfo("Servo controller: I've heard fr:%d fl:%d rl:%d rr:%d. Moving Servos!" % (data.front_right_angle, data.front_left_angle, data.rear_left_angle, data.rear_right_angle));
    
    pwm.set_pwm(0, 0, servo_min)
    time.sleep(1)

    pwm.set_pwm(0, 0, servo_max)
    time.sleep(1)

def listener():
    rospy.init_node("servo_controller", anonymous=True);
    sub = rospy.Subscriber("motor_control", MotorControl, callback);
    rospy.spin();

if __name__ == '__main__':
    listener();


