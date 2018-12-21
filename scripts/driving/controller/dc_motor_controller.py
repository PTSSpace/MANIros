#!/usr/bin/env python
import rospy
from maniros.msg import MotorControl
import atexit

# This requires this library to be installed: https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# TODO: Add a seperate DC Motor Class

class DCMotorController:
    motorSlugs = ["front_left", "rear_left", "rear_right", "front_right"];

    # Sets up all the motors
    def __init__(self, maxSpeed):
        self.maxSpeed = maxSpeed;
        self.motors = [];
        self.oldSpeeds = [0, 0, 0, 0];
        self.newSpeeds = [0, 0, 0, 0];
        self.motorStates = [False, False, False, False];

        # create a default object, no changes to I2C address or frequency
        mh = Adafruit_MotorHAT(addr=0x61, i2c_bus=0);

        # Turn off motors at shutdown
        atexit.register(self.turnOffMotors);

        # Select all 4 motors
        for i in xrange(1, 5):
            self.motors.append(mh.getMotor(i)); # the motor numbering on the motor boards starts at 1


    # Gets called if data arrives
    def callback(self, data):
        rospy.loginfo("DC controller: I've heard fl:%d rl:%d rr:%d fr:%d." % (data.front_left_speed, data.rear_left_speed, data.rear_right_speed, data.front_right_speed)); 
        self.updateSpeeds(data);


    # Declares the controller as a subscriber
    def listener(self):
        rospy.Subscriber("motor_control", MotorControl, self.callback);


    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
        for i in xrange(0, 4):
            self.turnOffMotor(i);


    # Turns off a specific dc motor
    def turnOffMotor(self, id):
        # Only turn off, if it is on
        if(self.isOn(id)):
            self.motors[id].run(Adafruit_MotorHAT.RELEASE);
            self.setMotorState(id, False);


    # Determines whether a motor is off
    def isOn(self, id):
        return self.motorStates[id];


    # Determines whether a motor is on
    def isOff(self, id):
        return not self.isOn(id);


    # Determines whether a motor should drive forwards
    def shouldDriveForwards(self, id):
        return self.newSpeeds[id] > 0

    
    # Determines whether a motor should drive backwards
    def shouldDriveBackwards(self, id):
        return self.newSpeeds[id] < 0;


    # Lets a motor drive forwards
    def driveForwards(self, id):
        self.motors[id].run(Adafruit_MotorHAT.FORWARD);
        self.setMotorState(id, True);


    # Lets a motor drive backwards
    def driveBackwards(self, id):
        self.motors[id].run(Adafruit_MotorHAT.BACKWARD);
        self.setMotorState(id, True);


    # Maps a float[0..1] to an int[0..255]
    def mapSpeed(self, speed):
        return int(speed * self.getMaxSpeed());


    # implements a smooth ramp up function, e.g. by using a sinoid
    def rampUp(self, id, oldSpeed, newSpeed):
        # TODO: implement actual ramp-up function. Using a mocking function for now.
        self.motors[id].setSpeed(newspeed);


    # updates all speed values
    def saveNewSpeeds(self, data):
        # Make copy of the values of the 
        self.oldSpeeds = self.newSpeeds[:];

        for i in xrange(0, 4):
            self.newSpeeds[i] = self.mapSpeed(self.getSingleSpeedById(i, data));


    # Sets each motor to the correct new direction
    def updateDirections(self):
        for i in xrange(0, 4):
            if self.shouldDriveForwards(i):
                self.driveForwards(i);
            elif self.shouldDriveBackwards(i):
                self.driveBackwards(i);
            else:
                self.turnOffMotor(i);


    # Sets the new speed of all motors
    def updateSpeeds(self, data):
        self.saveNewSpeeds(data);
        self.updateDirections();
        
        for i in xrange(0, 4):
            if(self.speedChanged(i)):
                self.setSpeed(i, self.getNewSpeed(i));
                rospy.loginfo("DC controller: Updated speed of %s motor to %d" % (DCMotorController.getMotorSlug(i), self.getNewSpeed(i)));


    # Checks whether the speed has actually changed
    def speedChanged(self, id):
        return self.getOldSpeed(id) != self.getNewSpeed(id);


    # Sets the speed of
    def setSpeed(self, id, speed):
        assert -(self.getMaxSpeed()) <= speed <= self.getMaxSpeed(), "Invalid Speed Value!";
        self.motors[id].setSpeed(speed);


    # Sets a motors state
    def setMotorState(self, id, value):
        assert isinstance(value, bool), "State is not a boolean!";
        self.motorStates[id] = value;


    # Gets the old speed of a motor
    def getOldSpeed(self, id):
        return self.oldSpeeds[id];


    # Gets the new speed of a motor
    def getNewSpeed(self, id):
        return self.newSpeeds[id];


    # Gets the global max speed
    def getMaxSpeed(self):
        return self.maxSpeed;


    # Returns the speed of a motor by its id
    @staticmethod
    def getSingleSpeedById(id, data):
        assert 0 <= id <= 3, "Invalid Motor ID!";
        return getattr(data, "%s_speed" % DCMotorController.getMotorSlug(id));


    # Returns the slug of a specific motor
    @classmethod
    def getMotorSlug(cls, id): # class methods need a class as second param
        return cls.motorSlugs[id];



if __name__ == '__main__':
    rospy.init_node("dc_controller", anonymous=True);
    maxSpeed = rospy.get_param("/dc_controller/max_speed");
    dcController = DCMotorController(maxSpeed);
    dcController.listener();
    rospy.spin()