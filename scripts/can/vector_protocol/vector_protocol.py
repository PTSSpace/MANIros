#!/usr/bin/env python

# Transformation based on right-hand rover coordinate system
# seen from above
# x - forward
# y - right
# z - downward

import math

class VectorTranslation:
    """
    This class is used to convert from general speed/angle input to individual speed/angle output for each wheel
    """
    def __init__(self, length, width):
        """
        Constructor for the rover vector protocol
        :param length: The distance between front and back wheels in mm (with the camera being 'front')
        :param name: The distance between right and left in mm(with the panel tilting 'right/left')
        """
        self.rover_length = length
        self.rover_width = width
        self.wheelIndexArray = ['front_left', 'rear_left', "rear_right", "front_right"]


    def normalizeArray(self, array, threshhold = 1):
        """
        Normalizes any array: e.g.
        threshhold = 1: [1.0, 2.0, -10.0] becomes [0.1, 0.2, -1]
        threshhold = 2: [1.0, 2.0, -10.0] becomes [0.2, 0.4, -2]
        threshhold = 20: [1.0, 2.0, -10.0] stays [1, 2, -10]
        :param array: An array with signed ints
        :param threshhold: Noramlize values to a threshhold (default: 1)
        """
        maxvalue = abs(max(array, key=abs)) # math.sqrt(2)
        if maxvalue > threshhold:
            for index, value in enumerate(array):
				array[index] = (value / maxvalue) * threshhold      # normalise and apply tranlational speed scaling factor
        return array

    def calculateRotationFactor(self, rotation):
        """
        Scales the rotationDistance by the real width and length the rover and
        calculates the x and y components of this scaled rotation
        :param rotation: the skarar rotationDistance
        """
        #rotation, the scaled rotationDistance is multiplied with the width or length, returning a rotation component
        #amd rotationHeightFactor = r_fac_x (as length is along the x axis)
        #this rotationWidthFactor = r_fac_y
        self.r_fac = rotation / math.hypot(self.rover_width, self.rover_length)
        self.r_fac_x = round(self.rover_width * self.r_fac, 4) # why round? because division of floats can lead to test failures 0.2 * 3 = 0.6000000001
        self.r_fac_y = round(self.rover_length * self.r_fac, 4)


    def addRotationAndTranslation(self, x_value, y_value):
        """
        rotationFactor  + translation (the vector xComponent, yComponent) = finalVector
        :param x_value: the translation along the X-axis
        :param y_value: the translation along the Y-axis
        """
        for index, wheel in enumerate(self.wheelIndexArray):
            if index <= 1: #assigns a negativ X Rotation to all left wheels
                x_calc = -self.r_fac_x + x_value
            else: #assigns a positive X Rotation to all right wheels
                x_calc = self.r_fac_x + x_value
            if 1 <= index <= 2: #assigns a negative Y Rotation to all rear wheels
                y_calc = -self.r_fac_y + y_value
            else: #assigns a positive Y Rotation to all front wheels
                y_calc= self.r_fac_y + y_value

            angle = math.atan2(y_calc, x_calc)
            speed = math.hypot(x_calc, y_calc)

            #MISSING
            #reverse speed if necessary, need Implementation of WHEN an angle should be turned
            #this has to be determined by the hardware orientation of the servo
            if (math.fabs(angle) > (math.pi / 2)):
				angle -= math.copysign(math.pi, angle)
				speed *= -1

            self.wheelAngle.append(angle)
            self.wheelSpeed.append(speed)

    def translateMoveControl(self, data):
    	self.wheelAngle = []
        self.wheelSpeed = []

        self.calculateRotationFactor(data.rotationAngle)

        self.addRotationAndTranslation(data.xSpeed, data.ySpeed)

        self.normalizeArray(self.wheelSpeed)

        return [self.wheelSpeed, self.wheelAngle]
