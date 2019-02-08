#!/usr/bin/env python

import unittest
import rostest
import rospy

from driving.adapter.vector_protocol.vector_protocol import VectorTranslation
# NOT from maniros.scripts.driving.adapter.vector_protocol.vector_protocol import VectorTranslation

class CaseNormalize(unittest.TestCase):

    def __init__(self, testName, case):
        super(CaseNormalize, self).__init__(testName)
        self.case = case

    def setUp(self):
        self.vt = VectorTranslation(1, 1)

    def case_normalize(self):
        '''
        Tests the normalizeArray function
        case = [input: [array], expected_output: [array]]
        '''
        self.assertListEqual(self.vt.normalizeArray(self.case[0]), self.case[1])

class CaseRotationFactor(unittest.TestCase):

    def __init__(self, testName, case):
        super(CaseRotationFactor, self).__init__(testName)
        self.case = case

    def setUp(self):
        self.vt = VectorTranslation(1, 1)
        
    def case_rotation_factor(self):
        '''
        Tests the calculateRotationFactor function
        case = [param: [height, width], input: rotation, expected_output: [r_fac, r_fac_x, r_fac_y]]
        '''
        self.vt.robot_height = self.case[0][0]
        self.vt.robot_width = self.case[0][1]

        if (self.case[0][0] == 0 and self.case[0][1] == 0):
            with self.assertRaises(ZeroDivisionError):
                self.vt.calculateRotationFactor(1)

        else:
            self.vt.calculateRotationFactor(self.case[1])
            self.assertEqual(round(self.vt.r_fac, 6), self.case[2][0])
            self.assertEqual(self.vt.r_fac_x, self.case[2][1])
            self.assertEqual(self.vt.r_fac_y, self.case[2][2])

class CaseAddRotAndTrans(unittest.TestCase):

    def __init__(self, testName, case):
        super(CaseAddRotAndTrans, self).__init__(testName)
        self.case = case

    def setUp(self):
        self.vt = VectorTranslation(1, 1)
        self.vt.wheelOffsetArray = self.case[0][0]
        
    def case_add_rot_and_trans(self):
        '''
        Tests the addRotationAndTranslation function
        case = [param: [[wheelOffsetArray], r_fac_x, r_fac_y], input: [x_value, y_value], expected_output: [[wheelAngleArray], [wheelSpeedArray]]]
        '''
        self.vt.calculateRotationFactor(0)
        self.vt.r_fac_x = self.case[0][1]
        self.vt.r_fac_y = self.case[0][2]
        self.vt.addRotationAndTranslation(self.case[1][0], self.case[1][1])
        
        self.assertEqual([ round(angle, 3) for angle in self.vt.wheelAngleArray], self.case[2][0])
        self.assertEqual([ round(speed, 3) for speed in self.vt.wheelSpeedArray], self.case[2][1])


class VectorProtocolTestSuite(unittest.TestSuite):
    def __init__(self):
        super(VectorProtocolTestSuite, self).__init__()

        normalize_cases = [
            [[0, 0, 0, 0], [0, 0, 0, 0]],
            [[1.0, 0.5, -0.1, 0.2], [1.0, 0.5, -0.1, 0.2]],
            [[10.0, 20.0, 40.0, 80.0], [0.125, 0.25, 0.5, 1]],
            [[10.0, -20.0, 40.0, -80.0], [0.125, -0.25, 0.5, -1]],
            [[-1.414, -1.414, 1.414, 1.414], [-1, -1, 1, 1]],
        ]
        for case in normalize_cases:
            self.addTest(CaseNormalize('case_normalize', case))

        rotation_cases = [
            [[0, 0], 0, [0, 0, 0]],
            [[3, 4], 0, [0, 0, 0]],
            [[3, 4], 1, [0.2, 0.6, 0.8]],
            [[3, 4], -1, [-0.2, -0.6, -0.8]],
            [[3, 4], 10, [2, 6, 8]],            
            [[3, 4], 0.1, [0.02, 0.06, 0.08]],
            [[100, 400], 1, [0.002425, 0.2425, 0.9701]],            
        ]
        for case in rotation_cases:
            self.addTest(CaseRotationFactor('case_rotation_factor', case))

        addition_cases = [
            [[[0,0,0,0], 0, 0], [0, 0], [[0,0,0,0], [0,0,0,0]]], # no moevement = neither angles nor speed           
            [[[0,0,0,0], 0, 0], [1, 0], [[0,0,0,0], [1,1,1,1]]], # forward = no angles, full speed
            [[[0,0,0,0], 0, 0], [0, -1], [[-1.571,-1.571,-1.571,-1.571], [1,1,1,1]]],  # towards right = angles: - Pi/2 (270 deg), full speed     
            [[[0,0,0,0], 0, 0], [1, 1], [[0.785,0.785,0.785,0.785], [1,1,1,1]]], # towards front-left = angles: Pi/4 (45 deg), speed: sqrt(2) before normalization
            [[[0,0,0,0], 1, 1], [0, 0], [[-0.785,0.785,-0.785,0.785], [-1,-1,1,1]]], # counter-clockwise = angles: +/- Pi/4 (45 deg), speed: sqrt(2) before normalization, with backwardsdrive if  deg > 180 deg
            [[[1.571, 1.571, 1.571, 1.571], 0, 0], [0, 0], [[1.571,1.571,1.571,1.571], [0,0,0,0]]], # no moevement = neither angles (+offset) nor speed  
            [[[1.571, 1.571, 1.571, 1.571], 0, 0], [1, 0], [[1.571,1.571,1.571,1.571], [1,1,1,1]]], # forward = no angles (+offset), full speed
            [[[1.571, 1.571, 1.571, 1.571], 1, 1], [0, 0], [[0.786,-0.785,0.786,-0.785], [-1,1,1,-1]]], # counter-clockwise = angles: +/- Pi/4 (45 deg), speed: sqrt(2) before normalization, with backwardsdrive if deg > 180 deg
        ]
        for case in addition_cases:
            self.addTest(CaseAddRotAndTrans('case_add_rot_and_trans', case))       
  

if __name__ == '__main__':
    rostest.rosrun('maniros', 'suite_test', 'unit_suite.VectorProtocolTestSuite')

