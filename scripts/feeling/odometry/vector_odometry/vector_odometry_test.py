#!/usr/bin/env python

"""
Imports
"""
import unittest
import math
from vector_odometry import VectorOdometry

"""
Classes
"""

class locomotionMsg:
    def __init__(self, d, s):
        # Wheel indexes [front left, rear left, rear right, front right]
        self.driveValue = d           # Wheel distance [m]/velocity [m/s]
        self.steerValue = s           # Wheel rotation angle [rad]/velocity [rad/s]

class VecOdmTest(unittest.TestCase):
    def __init__(self, *args):
        super(VecOdmTest, self).__init__(*args)
        self.success = False
        self.length = 0.436             # [m]
        self.width = 0.475              # [m]

    def test_stop_motion(self):
        vel = [0, 0, 0, 0]
        ort = [0, 0, 0, 0]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)

        self.assertEqual(vo.calculateOdometry(cmd), [0, 0, 0])

    def test_forward_motion(self):
        vel = [0.3, 0.3, 0.3, 0.3]
        ort = [0, 0, 0, 0]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)

        self.assertEqual(vo.calculateOdometry(cmd),  [0.3, 0, 0])

    def test_drive_r_motion(self):
        vel = [1, 1, 1, 1]
        ort = [-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)

        self.assertEqual(vo.calculateOdometry(cmd), [0, -1, 0])

    def test_drive_l_motion(self):
        vel = [0.5, 0.5, 0.5, 0.5]
        ort = [math.pi/2, math.pi/2, math.pi/2, math.pi/2]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)

        self.assertEqual(vo.calculateOdometry(cmd), [0, 0.5, 0])

    def test_veer_r_motion(self):
        x = 0.5
        y = -0.25
        rz = 0
        vel = [math.hypot(x,y), math.hypot(x,y), math.hypot(x,y), math.hypot(x,y)]
        ort = [math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)
        X = vo.roundValue(x)
        Y = vo.roundValue(y)
        RZ = vo.roundValue(rz)
        self.assertEqual(vo.calculateOdometry(cmd), [X,Y,RZ])

    def test_veer_l_motion(self):
        x = 0.5
        y = 0.7
        rz = 0
        vel = [math.hypot(x,y), math.hypot(x,y), math.hypot(x,y), math.hypot(x,y)]
        ort = [math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)
        X = vo.roundValue(x)
        Y = vo.roundValue(y)
        RZ = vo.roundValue(rz)
        self.assertEqual(vo.calculateOdometry(cmd), [X,Y,RZ])

    def test_backward_motion(self):
        vel = [-1.5, -1.5, -1.5, -1.5]
        ort = [0, 0, 0, 0]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)

        self.assertEqual(vo.calculateOdometry(cmd), [-1.5, 0, 0])

    def test_steer_r_motion(self):
        x = 1
        y = -1
        rz = 0
        vel = [math.hypot(x,y), math.hypot(x,y), math.hypot(x,y), math.hypot(x,y)]
        ort = [math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)
        X = vo.roundValue(x)
        Y = vo.roundValue(y)
        RZ = vo.roundValue(rz)
        self.assertEqual(vo.calculateOdometry(cmd), [X,Y,RZ])

    def test_steer_l_motion(self):
        x = 1
        y = 1
        rz = 0
        vel = [math.hypot(x,y), math.hypot(x,y), math.hypot(x,y), math.hypot(x,y)]
        ort = [math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)
        X = vo.roundValue(x)
        Y = vo.roundValue(y)
        RZ = vo.roundValue(rz)
        self.assertEqual(vo.calculateOdometry(cmd), [X,Y,RZ])

    def test_backward_r_motion(self):
        x = -1
        y = -1
        rz = 0
        angle = math.atan2(y,x) + math.pi
        vel = [-math.hypot(x,y), -math.hypot(x,y), -math.hypot(x,y), -math.hypot(x,y)]
        ort = [angle, angle, angle, angle]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)
        X = vo.roundValue(x)
        Y = vo.roundValue(y)
        RZ = vo.roundValue(rz)
        self.assertEqual(vo.calculateOdometry(cmd), [X,Y,RZ])

    def test_backward_l_motion(self):
        x = -0.5
        y = 0.5
        rz = 0
        angle = math.atan2(y,x) - math.pi
        vel = [-math.hypot(x,y), -math.hypot(x,y), -math.hypot(x,y), -math.hypot(x,y)]
        ort = [angle, angle, angle, angle]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)
        X = vo.roundValue(x)
        Y = vo.roundValue(y)
        RZ = vo.roundValue(rz)
        self.assertEqual(vo.calculateOdometry(cmd), [X,Y,RZ])


    def test_rotation_pos_motion(self):
        x = 0.0
        y = 0.0
        rz = 2/ math.hypot(self.width,self.length) # math.pi/2
        alpha = math.atan2(self.length,self.width)
        velocity = rz/2 * math.hypot(self.width,self.length)
        vel = [ -velocity, -velocity, velocity, velocity]
        ort = [-alpha, alpha, -alpha, alpha]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)
        X = vo.roundValue(x)
        Y = vo.roundValue(y)
        RZ = vo.roundValue(rz)
        self.assertEqual(vo.calculateOdometry(cmd), [X,Y,RZ])

    def test_rotation_neg_motion(self):
        x = 0.0
        y = 0.0
        rz = -2/ math.hypot(self.width,self.length) # -math.pi/2
        alpha = math.atan2(self.length,self.width)
        velocity = rz/2 * math.hypot(self.width,self.length)
        vel = [ -velocity, -velocity, velocity, velocity]
        ort = [-alpha, alpha, -alpha, alpha]

        cmd = locomotionMsg(vel, ort)
        vo = VectorOdometry(self.length, self.width)
        X = vo.roundValue(x)
        Y = vo.roundValue(y)
        RZ = vo.roundValue(rz)
        self.assertEqual(vo.calculateOdometry(cmd), [X,Y,RZ])

if __name__ == '__main__':
    unittest.main()
