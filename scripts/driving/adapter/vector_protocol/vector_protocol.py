#!/usr/bin/env python
import math
import rospy


from maniros.msg import RoverControl
from maniros.msg import MotorControl

class VectorTranslation:
    """
    This class is used to convert RoverControl messages to MotorControls
    """
    def __init__(self, height, width):
        """
        Constructor for the rover vector protocol
        :param height: The distance between front and back wheels in mm (with the camera being 'front')
        :param name: The distance between right and left in mm(with the panel tilting 'right/left')
        """
        self.robot_height = height
        self.robot_width = width
        self.wheelIndexArray = ['front_left', 'rear_left', "rear_right", "front_right"]
        self.wheelAngleArray = []
        self.wheelSpeedArray = []

    def normalizeArray(self, array, threshhold = 1):
        """
        Normalizes any array: e.g.
        threshhold = 1: [1.0, 2.0, -10.0] becomes [0.1, 0.2, -1]
        threshhold = 2: [1.0, 2.0, -10.0] becomes [0.2, 0.4, -2]
        threshhold = 20: [1.0, 2.0, -10.0] stays [1, 2, -10]

        :param array: An array with signed ints
        :param threshhold: Noramlize values to a threshhold (default: 1)
        """
        maxvalue = abs(max(array, key=abs))
        if maxvalue > threshhold:
            for index, value in enumerate(array):
                array[index] = (value / maxvalue) * threshhold
        return array

    def calculateRotationFactor(self, rotation):
        """
        Scales the rotationDistance by the real width and height the rover and
        calculates the x and y components of this scaled rotation

        :param rotation: the skarar rotationDistance
        """
        #rotation, the scaled rotationDistance is multiplied with the width or height, returning a rotation component
        #amd rotationHeightFactor = r_fac_x (as height is along the x axis)
        #this rotationWidthFactor = r_fac_y
        self.r_fac = rotation / math.hypot(self.robot_width, self.robot_height)
        self.r_fac_x = round(self.robot_height * self.r_fac, 4) # why round? because division of floats can lead to test failures 0.2 * 3 = 0.6000000001
        self.r_fac_y = round(self.robot_width * self.r_fac, 4)

    def addRotationAndTranslation(self, x_value, y_value):
        """
        rotationFactor  + translation (the vector xDistance, yDistance) = finalVector

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
            if math.fabs(angle) > (math.pi / 2): #this > (math.pi / 2) should be changed
                angle -= math.copysign(math.pi, angle)
                speed *= -1
            
            self.wheelAngleArray.append(angle)
            self.wheelSpeedArray.append(speed)

    def translateRoverControl(self, data):
        self.msg = MotorControl()

        self.calculateRotationFactor(data.rotationDistance)
        
        self.addRotationAndTranslation(data.xDistance, data.yDistance)
        
        self.normalizeArray(self.wheelSpeedArray)
        
        for index, wheel in enumerate(self.wheelIndexArray):
            setattr(self.msg, '%s_angle' % wheel, self.wheelAngleArray[index])
            setattr(self.msg, '%s_speed' % wheel, self.wheelSpeedArray[index])

        return self.msg