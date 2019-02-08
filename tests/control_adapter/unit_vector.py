#!/usr/bin/env python

import unittest
import rostest
import rospy

from driving.adapter.vector_protocol.vector_protocol import VectorTranslation
# NOT from maniros.scripts.driving.adapter.vector_protocol.vector_protocol import VectorTranslation

class TestVectorProtocol(unittest.TestCase):

    # @classmethod
    # def setUpClass(cls):
    #     cls.vt = VectorTranslation(0, 0)

    def setUp(self):
        self.vt = VectorTranslation(0, 0)
    
    # def tearDown(self):
    #     pass

    def test_normalize(self):
        cases = [
            [[0, 0, 0, 0], [0, 0, 0, 0]],
            [[1.0, 0.5, -0.1, 0.2], [1.0, 0.5, -0.1, 0.2]],
            [[10.0, 20.0, 40.0, 80.0], [0.125, 0.25, 0.5, 1]]
        ]
        for case in cases:
            self.assertListEqual(self.vt.normalizeArray(case[0]), case[1])

    def test_rotation_factor(self):
        '''
        Tests the calculateRotationFactor function
        case = [param: [height, width], input: rotation, expected_output: [r_fac, r_fac_x, r_fac_y]]
        '''
        cases = [
            [[3, 4], 0, [0, 0, 0]],
            [[3, 4], 1, [0.2, 0.6, 0.8]],
        ]
        for case in cases:
            self.vt.robot_height = case[0][0]
            self.vt.robot_width = case[0][1]
            self.vt.calculateRotationFactor(case[1])
            
            self.assertEqual(self.vt.r_fac, case[2][0])
            self.assertEqual(self.vt.r_fac_x, case[2][1])
            self.assertEqual(self.vt.r_fac_y, case[2][2])
        
    

if __name__ == '__main__':
    rostest.rosrun('maniros', 'vector_test', TestVectorProtocol)

