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
        self.vt = VectorTranslation(0, 0)

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
        self.vt = VectorTranslation(0, 0)
        
    def case_rotation_factor(self):
        '''
        Tests the calculateRotationFactor function
        case = [param: [height, width], input: rotation, expected_output: [r_fac, r_fac_x, r_fac_y]]
        '''
        self.vt.robot_height = self.case[0][0]
        self.vt.robot_width = self.case[0][1]
        self.vt.calculateRotationFactor(self.case[1])
        
        self.assertEqual(self.vt.r_fac, self.case[2][0])
        self.assertEqual(self.vt.r_fac_x, self.case[2][1])
        self.assertEqual(self.vt.r_fac_y, self.case[2][2])


    
class VectorProtocolTestSuite(unittest.TestSuite):
    def __init__(self):
        super(VectorProtocolTestSuite, self).__init__()

        normalize_cases = [
            [[0, 0, 0, 0], [0, 0, 0, 0]],
            [[1.0, 0.5, -0.1, 0.2], [1.0, 0.5, -0.1, 0.2]],
            [[10.0, 20.0, 40.0, 80.0], [0.125, 0.25, 0.5, 1]]
        ]
        for case in normalize_cases:
            self.addTest(CaseNormalize('case_normalize', case))

        rotation_cases = [
            [[3, 4], 0, [0, 0, 0]],
            [[3, 4], 1, [0.2, 0.6, 0.8]],
        ]
        for case in rotation_cases:
            self.addTest(CaseRotationFactor('case_rotation_factor', case))
    
    @classmethod
    def setUpClass(cls):
        cls.vt = VectorTranslation(0, 0)

if __name__ == '__main__':
    rostest.rosrun('maniros', 'suite_test', 'unit_suite.VectorProtocolTestSuite')

