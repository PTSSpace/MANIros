#!/usr/bin/env python
from __future__ import division
import rospy
from maniros.msg import MotorControl
import time
import atexit

# This requires this library to be installed: https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# Sets up all the motors
def init():
    global maxSpeed, motors, oldSpeeds, newSpeeds;

    # Init the global variables
    maxSpeed = rospy.get_param("/dc_controller/max_speed");
    motors = [];
    oldSpeeds = [0, 0, 0, 0];
    newSpeeds = [0, 0, 0, 0];

    # create a default object, no changes to I2C address or frequency
    mh = Adafruit_MotorHAT(addr=0x61, i2c_bus=0);

    # Turn off motors at shutdown
    atexit.register(turnOffMotors);

    # Select all 4 motors, set direction to FORWARD, and enebale the motors
    for i in xrange(0, 4):
        # Arrays in python start at 0, but the motor numbering on the motor starts at 1
        motors.append(mh.getMotor(i + 1));

        motors[i].run(Adafruit_MotorHAT.FORWARD);
        motors[i].run(Adafruit_MotorHAT.RELEASE);


# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    global motors;

    for i in xrange(0, 4):
        turnOffMotor(i);


# Turns off a specific dc motor
def turnOffMotor(id):
    global motors;

    motors[id].run(Adafruit_MotorHAT.RELEASE);


# Maps a float[0..1] to an int[0..255]
def mapSpeed(speed):
    global maxSpeed;

    return int(speed * maxSpeed);


# implements a smooth ramp up function, e.g. by using a sinoid
def rampUp(id, oldSpeed, newSpeed):
    global motors;

    # TODO: implement actual ramp-up function. Using a mocking function for now.
    motors[id].setSpeed(newspeed);


# Returns the speed of a motor by its id
def getSingleSpeedById(id, data):
    assert 0 <= id <= 3, "Invalid Motor ID!";

    speeds = [
        data.front_left_speed, 
        data.rear_left_speed, 
        data.rear_right_speed, 
        data.front_right_speed
    ];

    return speeds[id];


# updates all speed values
def saveNewSpeeds(data):
    global oldSpeeds, newSpeeds;

    # Make copy of the values of the 
    oldSpeeds = newSpeeds[:];

    for i in xrange(0, 4):
        newSpeeds[i] = mapSpeed(getSingleSpeedById(i, data));


# sets each motor to the correct direction
def updateDirections():
    global newSpeeds;

    for i in xrange(0, 4):
        # TODO: Check if motor is turned off and turn on if needed 
        if newSpeeds[i] > 0:
            motors[i].run(Adafruit_MotorHAT.FORWARD);
        elif newSpeeds[i] < 0:
            motors[i].run(Adafruit_MotorHAT.BACKWARD);
        else:
            turnOffMotor(i);


# Sets the new speed of all motors
def updateSpeeds():
    for i in xrange(0, 4):
        if(speedChanged(i)):
            setSpeed(i, getNewSpeed(i));
            rospy.loginfo("DC controller: Updated speed of motor %d to %d" % (i + 1, getNewSpeed(i)));


# Checks whether the speed has actually changed
def speedChanged(id):
    return getOldSpeed(id) != getNewSpeed(id);


# Sets the speed of
def setSpeed(id, speed):
    global motors;

    assert 0 <= speed <= getMaxSpeed(), "Invalid Speed Value!";
    motors[id].setSpeed(speed);


# Gets the old speed of a motor
def getOldSpeed(id):
    global oldSpeeds;

    return oldSpeeds[id];


# Gets the new speed of a motor
def getNewSpeed(id):
    global newSpeeds;

    return newSpeeds[id];


# Gets the global max speed
def getMaxSpeed():
    global maxSpeed;

    return maxSpeed;


############################################################
# Gets called if data arrives
############################################################
def callback(data):
    global motors, oldSpeeds, newSpeeds;

    rospy.loginfo("DC controller: I've heard fl:%d rl:%d rr:%d fr:%d." % (data.front_left_speed, data.rear_left_speed, data.rear_right_speed, data.front_right_speed)); 

    saveNewSpeeds(data);
    updateDirections();
    updateSpeeds();


def listener():
    rospy.init_node("dc_controller", anonymous=True);
    sub = rospy.Subscriber("motor_control", MotorControl, callback);
    init();
    rospy.spin();


if __name__ == '__main__':
    listener();