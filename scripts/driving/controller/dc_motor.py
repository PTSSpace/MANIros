#!/usr/bin/env python

# This requires this library to be installed: https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor


class DCMotor:
    """Adapts the Adafruit_DCMotor class so that it can be used more easily"""

    FORWARDS = Adafruit_MotorHAT.FORWARD
    BACKWARDS = Adafruit_MotorHAT.BACKWARD
    OFF = Adafruit_MotorHAT.RELEASE

    def __init__(self, motor, name="undefined"):
        """
        Inits the motor object

        :param motor: The adapted Adafruit_DCMotor motor object
        :param name: The name of the motor
        """
        self.motor = motor
        self.name = name
        self.speed = 0
        self.state = DCMotor.OFF

    def set_speed(self, new_speed):
        """
        Sets the speed of a motor
        TODO: Convert speed to a python property

        :param new_speed: The new speed of the motor
        """
        if new_speed > 0:
            self.set_state(DCMotor.FORWARDS)
        elif new_speed < 0:
            self.set_state(DCMotor.BACKWARDS)
        else:
            self.set_state(DCMotor.OFF)

        self.motor.setSpeed(new_speed)
        self.speed = new_speed

    def set_state(self, state):
        """
        Sets the motors state

        :param state: The new motor state
        """
        if self.state != state:
            self.motor.run(state)
            self.state = state

    def turn_off(self):
        """
        Turns off the motor
        """
        self.set_state(DCMotor.OFF)
