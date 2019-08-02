#!/usr/bin/env python

"""
This program provides a transformation for individual wheel encoder feedback (wheel coordinates)
high-level rover odometry (rover coordintes).

Input:
* wheel distance/velocity and orientation array (driveValue, steerValue)
Output:
* rover displacement/velocity in (x,y,rz)

The transformation is based on a right-hand rover coordinate system
As seen from above:
x - forward
y - left
z - upward
whereby the wheel indexing is is done in positive roation direction (counter-clockwise).
['front_left', 'rear_left', 'rear_right', 'front_right']
"""

import math

class VectorOdometry:
    """
    This class is used to convert from general speed/angle input to individual speed/angle output for each wheel
    """
    def __init__(self, length, width):
        """
        Constructor for the rover vector odometry
        :param length: The distance between front and back wheels [m] (with the camera being 'front')
        :param width: The distance between right and left [m] (with the panel tilting 'right/left')
        """
        self.rover_length = length                                              # [m]
        self.rover_width = width                                                # [m]
        self.wheelIndex = ['front_left', 'rear_left', 'rear_right', 'front_right']

        # Angle between perpendicular velocity/distance to rover hypotinuse and zero steering orientation
        self.beta = math.atan2(self.rover_width, self.rover_length)             # [m]

    def calculateRotationVelocity(self, driveValue, steerValue):
        """
        Computes the components of the wheel velocity/distance perpendicular to the rover hypotinuse.
        The rotational component of the wheel velocity/distance is derived through the mean of the
        oposite perpendicular velocities/distances:
        * fl and rr
        * rl and fr
        By middling all values an estimation of the roatational part of the locomotion in rz direction is achieved.
        :param driveValue: Wheel velocity/distance array comprised of the four fover wheel ordered as in 'wheelIndex' [m/s]
        :param steerValue: Wheel orientation array comprised of the four fover wheel ordered as in 'wheelIndex' [rad]
        """
        perpendicularValue              = [0, 0, 0, 0]                          # [m] or [m/s]
        for idx, wheel in enumerate(self.wheelIndex):
            if  idx == 0:
                perpendicularValue[idx] = -math.cos(-math.pi/2-steerValue[idx]+self.beta)*driveValue[idx]
            elif idx == 2:
                perpendicularValue[idx] = math.cos(-math.pi/2-steerValue[idx]+self.beta)*driveValue[idx]
            elif idx == 1:
                perpendicularValue[idx] = -math.cos(math.pi/2-steerValue[idx]-self.beta)*driveValue[idx]
            elif idx == 3:
                perpendicularValue[idx] = math.cos(math.pi/2-steerValue[idx]-self.beta)*driveValue[idx]
        # Through the mean of all perpendicular velocities/distances the angular velocity/distance is estimated
        angularValue = sum(perpendicularValue)/len(self.wheelIndex)             # [m] or [m/s]
        return angularValue

    def calculateRotationVector(self, angularValue):
        """
        Calculates the rotational vector component of each wheel velocity/distance vector.
        :param angularValue: Angular velocity/distance portion applied at each wheel
        """
        angularValueX = [0, 0, 0, 0]                                            # [m] or [m/s]
        angularValueY = [0, 0, 0, 0]                                            # [m] or [m/s]
        for idx, wheel in enumerate(self.wheelIndex):
            if idx <= 1: #assigns a positive X Rotation to all left wheels
                angularValueX[idx] = -math.sin(self.beta)*angularValue
            else: #assigns a negative X Rotation to all right wheels
                angularValueX[idx] = math.sin(self.beta)*angularValue
            if 1 <= idx <= 2: #assigns a negative Y Rotation to all rear wheels
                    angularValueY[idx] = -math.cos(self.beta)*angularValue
            else: #assigns a positive Y Rotation to all front wheels
                angularValueY[idx] = math.cos(self.beta)*angularValue
        return [angularValueX, angularValueY]

    def calculateTranslationVector(self, driveValue, steerValue, angularValueX, angularValueY):
        """
        Calculates the translational velocity/distance vector by subtracting the rotation velocity/distance vector
        from the wheel velocity/distance vector.
        :param driveValue: Wheel velocity/distance array comprised of the four fover wheel ordered as in 'wheelIndex' [m/s]
        :param steerValue: Wheel orientation array comprised of the four fover wheel ordered as in 'wheelIndex' [rad]
        :param angularValueX: X component of the orientation vector indicating the rotational velocity/distance
        :param angularValueY: Y component of the orientation vector indicating the rotational velocity/distance
        """

        translationalValueX = [0, 0, 0, 0]                                      # [m] or [m/s]
        translationalValueY = [0, 0, 0, 0]                                      # [m] or [m/s]
        for idx, wheel in enumerate(self.wheelIndex):
            if idx <= 1:
                translationalValueX[idx] = math.cos(steerValue[idx])*driveValue[idx] - angularValueX[idx]
            else:
                translationalValueX[idx] = math.cos(steerValue[idx])*driveValue[idx] - angularValueX[idx]
            if 1 <= idx <= 2:
                translationalValueY[idx] = math.sin(steerValue[idx])*driveValue[idx] - angularValueY[idx]
            else:
                translationalValueY[idx] = math.sin(steerValue[idx])*driveValue[idx] - angularValueY[idx]
        return [translationalValueX, translationalValueY]

    def calculateTranslationVelocity(self, translationalValueX, translationalValueY):
        """
        Forms the mean velocity/distance of all wheels.
        :param translationalValueX: X component of the direction vector indicating the translational velocity/distance
        :param translationalValueY: Y component of the direction vector indicating the translational velocity/distance
        """
        self.x = sum(translationalValueX)/len(self.wheelIndex)
        self.y = sum(translationalValueY)/len(self.wheelIndex)

    def roundValue(self, value, digit = 10):
        """
        Rounds values to avoid test failures.
        the specified digit.
        :param value: A signed value
        :param digit: Specifies the number of floating point to round to
        """
        return round(value, digit)

    def calculateTurningRate(self, angularValue):
        """
        Calculates the turning rate/turned angle of the rover [rad/s]/[rad]
        :param angularValue: Angular velocity/distance portion applied at each wheel
        """
        self.rz = angularValue/(math.hypot(self.rover_width, self.rover_length)/2)  # [rad] or [rad]

    def calculateOdometry(self, data):
        """
        Odometry calculation
        :param driveValue: Wheel velocity/distance array comprised of the four fover wheel ordered as in 'wheelIndex' [m/s]
        :param steerValue: Wheel orientation array comprised of the four fover wheel ordered as in 'wheelIndex' [rad]
        """
        self.x = 0
        self.y = 0
        self.rz = 0

        angularValue = self.calculateRotationVelocity(data.driveValue, data.steerValue)
        self.calculateTurningRate(angularValue)
        [angularValueX, angularValueY] = self.calculateRotationVector(angularValue)
        [translationalValueX, translationalValueY] = self.calculateTranslationVector(data.driveValue, data.steerValue, angularValueX, angularValueY)
        self.calculateTranslationVelocity(translationalValueX, translationalValueY)

        self.x = self.roundValue(self.x)
        self.y = self.roundValue(self.y)
        self.rz = self.roundValue(self.rz)

        return [self.x, self.y, self.rz]