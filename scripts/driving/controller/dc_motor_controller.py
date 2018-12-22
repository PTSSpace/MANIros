#!/usr/bin/env python
import rospy
from maniros.msg import MotorControl
import atexit

# This requires this library to be installed: https://githjoub.com/adafruit/Adafruit-Motor-HAT-Python-Library
from Adafruit_MotorHAT import Adafruit_MotorHAT
from dc_motor import DCMotor


class DCMotorController:
    """
    This class handles all DC motor control tasks
    """

    motor_names = ["front_left", "rear_left", "rear_right", "front_right"]

    def __init__(self, max_speed):
        """
        Sets up all the motors
        TODO: Discuss whether or not to use private variables

        :param max_speed: The global maximum speed, as determined by the used motor hat library
        """
        self.max_speed = max_speed
        self.motors = []

        mh = Adafruit_MotorHAT(addr=0x61, i2c_bus=0)

        # Store all 4 motor objects
        for i in range(1, 5):
            self.motors.append(
                DCMotor(mh.getMotor(i), self.get_motor_name(i - 1))
            )

        # Turn off motors at shutdown
        atexit.register(self.turn_off_motors)

    def callback(self, data):
        """
        Called if data arrives. Commands the update of the speeds of all motors.
        TODO: Check data integrity and whether the arrived data actually makes sense

        :param data: The received data
        """
        rospy.loginfo("DC controller: I've heard fl:%d rl:%d rr:%d fr:%d." % (
            data.front_left_speed,
            data.rear_left_speed,
            data.rear_right_speed,
            data.front_right_speed
        ))

        new_speeds = self.decode_speeds(data)
        self.update_speeds(new_speeds)

    def listener(self):
        """
        Declares this controller as a subscriber
        """
        rospy.Subscriber("motor_control", MotorControl, self.callback)

    def map_speed(self, speed):
        """
        Maps a float[0..1] to an int[0..255]

        :param speed: The speed that shall be mapped
        :return: The mapped speed value
        """
        return int(speed * self.max_speed)

    def decode_speeds(self, data):
        """
        Translates the received MotorControl message into an array
        TODO: Discuss whether we should actually send such an array in the message, instead of separate named values

        :param data: The received MotorControl message
        :return: The generated array containing the new speeds
        """
        new_speeds = []
        for i in range(0, len(self.motors)):
            new_speed = getattr(data, "%s_speed" % DCMotorController.get_motor_name(i))
            new_speeds.append(new_speed)

        return new_speeds

    def ramp_speed(self, num, new_speed):
        """
        Implements a smooth ramp up/down function, e.g. by using a sinoid
        TODO: implement actual ramp-up function. Using a mocking function for now.

        :param num: The
        :param new_speed:
        :return:
        """
        self.motors[num].set_speed(new_speed)

    def update_speeds(self, new_speeds):
        """
        Updates the speeds of all motors

        :param new_speeds: The array containing the new speeds
        """
        for i in range(0, len(self.motors)):
            new_speed = new_speeds[i]
            new_speed = self.map_speed(new_speed)

            # Check whether there is anything to change
            if new_speed != self.get_speed(i):
                self.set_speed(i, new_speed)
                rospy.loginfo("DC controller: Updated speed of %s motor to %d" % (
                    self.motors[i].name,
                    self.motors[i].speed
                ))

    def turn_off_motors(self):
        """
        Turns off all motors
        """
        for i in range(0, len(self.motors)):
            self.turn_off(i)

    def turn_off(self, num):
        """
        Turns of a specific motor

        :param num: The number of the motor that shall be turned off
        """
        self.motors[num].turn_off()

    def set_speed(self, num, new_speed):
        """
        Sets the speed of a motor

        :param num: The number of the motor
        :param new_speed: The new speed
        """
        assert -self.max_speed <= new_speed <= self.max_speed, "Invalid Speed Value!"
        self.ramp_speed(num, new_speed)

    def get_speed(self, num):
        """
        Returns the speed of a motor

        :param num: The number of the motor
        """
        return self.motors[num].speed

    @classmethod
    def get_motor_name(cls, num):
        """
        Returns the name of a motor

        :param num: The motor number
        """
        return cls.motor_names[num]


if __name__ == '__main__':
    rospy.init_node("dc_controller", anonymous=True)
    dc_controller = DCMotorController(rospy.get_param("/dc_controller/max_speed"))
    dc_controller.listener()
    rospy.spin()
