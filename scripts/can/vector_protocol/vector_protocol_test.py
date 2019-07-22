#!/usr/bin/env python

import unittest
import math
from vector_protocol import VectorTranslation

class geomMsg:
    def __init__(self, x, y, rz):
        self.xSpeed = x
        self.ySpeed = y
        self.rotationAngle = rz

class VecProTest(unittest.TestCase):
    def __init__(self, *args):
        super(VecProTest, self).__init__(*args)
        self.success = False

    def test_stop_function(self):

        length = 436.0
        width = 475.7

        x = 0
        y = 0
        rz = 0

        stp = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)
        
        self.assertEqual(vt.translateMoveControl(stp), [[x , x, x, x],[0, 0, 0, 0]])

    def test_forward_function(self):

        length = 436.0
        width = 475.7


        x = 0.3
        y = 0
        rz = 0

        fwd = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)
        
        self.assertEqual(vt.translateMoveControl(fwd), [[x , x, x, x],[0, 0, 0, 0]])

    def test_drive_l_function(self):

        length = 436.0
        width = 475.7


        x = 0
        y = 1
        rz = 0

        drv = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)

        self.assertEqual(vt.translateMoveControl(drv), [[y , y, y, y],[math.pi/2, math.pi/2, math.pi/2, math.pi/2]])

    def test_drive_r_function(self):

        length = 436.0
        width = 475.7


        x = 0
        y = -1
        rz = 0

        drv = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)

        self.assertEqual(vt.translateMoveControl(drv), [[-y , -y, -y, -y],[-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2]])

    def test_veer_l_function(self):

        length = 436.0
        width = 475.7


        x = 0
        y = 0.25
        rz = 0

        ver = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)
        
        self.assertEqual(vt.translateMoveControl(ver), [[y , y, y, y],[math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]])

    def test_veer_r_function(self):

        length = 436.0
        width = 475.7

        x = 0
        y = -0.75
        rz = 0

        ver = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)
        
        self.assertEqual(vt.translateMoveControl(ver), [[-y , -y, -y, -y],[math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]])

    def test_backward_function(self):

        length = 436.0
        width = 475.7

        x = -1
        y = 0
        rz = 0

        bck = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)
        
        self.assertEqual(vt.translateMoveControl(bck), [[x , x, x, x],[0, 0, 0, 0]])

    def test_steer_l_function(self):

        length = 436.0
        width = 475.7

        x = 1
        y = 1
        rz = 0

        ser = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)
        
        self.assertEqual(vt.translateMoveControl(ser), [[x , x, x, x],[math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]])


    def test_steer_r_function(self):

        length = 436.0
        width = 475.7

        x = 1
        y = -1
        rz = 0

        ser = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)
        
        self.assertEqual(vt.translateMoveControl(ser), [[x , x, x, x],[math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]])

    def test_backward_l_function(self):

        length = 436.0
        width = 475.7

        x = -1
        y = 1
        rz = 0

        ser = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)

        angle = math.atan2(y,x) - math.pi
        
        self.assertEqual(vt.translateMoveControl(ser), [[x , x, x, x],[angle, angle, angle, angle]])

    def test_backward_r_function(self):

        length = 436.0
        width = 475.7

        x = -0.5
        y = -0.5
        rz = 0

        ser = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)

        angle = math.atan2(y,x) + math.pi
        
        self.assertEqual(vt.translateMoveControl(ser), [[math.sqrt(2)*x , math.sqrt(2)*x, math.sqrt(2)*x, math.sqrt(2)*x],[angle, angle, angle, angle]])

    def test_rotation_pos_function(self):
        
        length = 436.0                                                                                                                                  
        width = 475.7                                                                                                                                   
                                                                                                                                                        
        x = 0.0                                                                                                                                        
        y = 0.0                                                                                                                                        
        rz = math.pi/2                                                                                                                                          
                                                                                                                                                        
        cmd = geomMsg(x, y, rz)                                                                                                                         
        vt = VectorTranslation(length, width)                                                                                                           
                                                                                                                                                        
        alpha = math.atan2(length,width)                                                                                                               
        vel = 1 # normalised math.sqrt(2*pow(rz,2)+(math.pow(width,2)+math.pow(length,2))/4)                                                                                                                  
        self.assertEqual(vt.translateMoveControl(cmd), [[ vel, vel, -vel, -vel],[alpha, -alpha, alpha, -alpha]])

    def test_rotation_neg_function(self):

        length = 436.0
        width = 475.7

        x = 0.0
        y = 0.0
        rz = -math.pi/2                                                                                                           

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(length, width)

        alpha = math.atan2(length,width)
        vel = 1 # normalised math.sqrt(2*pow(rz,2)+(math.pow(width,2)+math.pow(length,2))/4)
        self.assertEqual(vt.translateMoveControl(cmd), [[ -vel, -vel, vel, vel],[alpha, -alpha, alpha, -alpha]])

if __name__ == '__main__':
    unittest.main()
