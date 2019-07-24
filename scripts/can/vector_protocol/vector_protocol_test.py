#!/usr/bin/env python

import unittest
import math
from vector_protocol import VectorTranslation

class geomMsg:
    def __init__(self, x, y, rz):
        self.x = x                      # Rover velocity in x-direction [m/s]
        self.y = y                      # Rover velocity in y-direction [m/s]
        self.rz = rz                    # Rover rotation velocity [rad]

class VecProTest(unittest.TestCase):
    def __init__(self, *args):
        super(VecProTest, self).__init__(*args)
        self.success = False
        self.length = 0.436             # [m]
        self.width = 0.475              # [m]

    def test_stop_function(self):
        x = 0
        y = 0
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)

        self.assertEqual(vt.translateMoveControl(cmd), [[x, x, x, x],[0, 0, 0, 0]])

    def test_forward_function(self):
        x = 0.3
        y = 0
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)

        self.assertEqual(vt.translateMoveControl(cmd), [[x, x, x, x],[0, 0, 0, 0]])

    def test_drive_l_function(self):
        x = 0
        y = -1
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)
        vel = vt.roundArray([-y, -y, -y, -y])
        ort = vt.roundArray([-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])

    def test_drive_r_function(self):
        x = 0
        y = 0.5
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)
        vel = vt.roundArray([y, y, y, y])
        ort = vt.roundArray([math.pi/2, math.pi/2, math.pi/2, math.pi/2])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])

    def test_veer_l_function(self):
        x = 0.5
        y = -0.25
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)
        vel = vt.roundArray([math.hypot(x,y), math.hypot(x,y), math.hypot(x,y), math.hypot(x,y)])
        ort = vt.roundArray([math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])

    def test_veer_r_function(self):
        x = 0.5
        y = 0.7
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)
        vel = vt.roundArray([math.hypot(x,y), math.hypot(x,y), math.hypot(x,y), math.hypot(x,y)])
        ort = vt.roundArray([math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])

    def test_backward_function(self):
        x = -1.5
        y = 0
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)

        self.assertEqual(vt.translateMoveControl(cmd), [[-1, -1, -1, -1],[0, 0, 0, 0]])

    def test_steer_l_function(self):
        x = 1
        y = -1
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)
        vel = vt.roundArray([x, x, x, x])
        ort = vt.roundArray([math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])

    def test_steer_r_function(self):
        x = 1
        y = 1
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)
        vel = vt.roundArray([x, x, x, x])
        ort = vt.roundArray([math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])
    def test_backward_l_function(self):
        x = -1
        y = -1
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)

        angle = math.atan2(y,x) + math.pi
        vel = vt.roundArray([x, x, x, x])
        ort = vt.roundArray([angle, angle, angle, angle])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])

    def test_backward_r_function(self):
        x = -0.5
        y = 0.5
        rz = 0

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)

        angle = math.atan2(y,x) - math.pi
        vel = vt.roundArray([math.sqrt(2)*x, math.sqrt(2)*x, math.sqrt(2)*x, math.sqrt(2)*x])
        ort = vt.roundArray([angle, angle, angle, angle])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])

    def test_rotation_pos_function(self):
        x = 0.0
        y = 0.0
        rz = math.pi/2

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)

        alpha = math.atan2(self.length,self.width)
        v = rz/2 * math.hypot(self.width,self.length)
        velocity = vt.normalizeArray([v, v, -v, -v])
        vel = vt.roundArray(velocity)
        ort = vt.roundArray([alpha, -alpha, alpha, -alpha])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])


    def test_rotation_neg_function(self):
        x = 0.0
        y = 0.0
        rz = -math.pi/2

        cmd = geomMsg(x, y, rz)
        vt = VectorTranslation(self.length, self.width)

        alpha = math.atan2(self.length,self.width)
        v = rz/2 * math.hypot(self.width,self.length)
        velocity = vt.normalizeArray([v, v, -v, -v])
        vel = vt.roundArray(velocity)
        ort = vt.roundArray([alpha, -alpha, alpha, -alpha])

        self.assertEqual(vt.translateMoveControl(cmd), [vel,ort])

if __name__ == '__main__':
    unittest.main()
