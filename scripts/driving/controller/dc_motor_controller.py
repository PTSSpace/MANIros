#!/usr/bin/env python
from __future__ import division
import rospy
from maniros.msg import MotorControl
import time
import atexit
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    global myMotor;
    myMotor.run(Adafruit_MotorHAT.RELEASE);

def init():
    global mh, myMotor, maxSpeed;

    maxSpeed = 255;

    # create a default object, no changes to I2C address or frequency
    mh = Adafruit_MotorHAT(addr=0x61, i2c_bus=0);

    # Turn off motors at shutdown
    atexit.register(turnOffMotors);

    # Select a test-motor, set direction to FORWARD, and enebale motor
    myMotor = mh.getMotor(1);
    myMotor.run(Adafruit_MotorHAT.FORWARD);
    myMotor.run(Adafruit_MotorHAT.RELEASE);

def map(speed):
    global maxSpeed;
    return int(speed * maxSpeed);

def callback(data):
    global myMotor, mh;

    rospy.loginfo("DC Motor controller: I've heard fl:%d rl:%d rr:%d fr:%d." % (data.front_left_speed, data.rear_left_speed, data.rear_right_speed, data.front_right_speed)); 
    
    if data.front_left_speed < 0:
        myMotor.run(Adafruit_MotorHAT.BACKWARD);
    elif data.front_left_speed > 0:
        myMotor.run(Adafruit_MotorHAT.FORWARD);
    else:
        turnOffMotors();

    newSpeed = map(data.front_left_speed);
    myMotor.setSpeed(newSpeed);

    rospy.loginfo("DC Motor controller: I've set the speed to %d." % newSpeed); 
    

def listener():
    rospy.init_node("servo_controller", anonymous=True);
    sub = rospy.Subscriber("motor_control", MotorControl, callback);
    rospy.spin();

if __name__ == '__main__':
    init();
    listener();