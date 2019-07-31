#!/usr/bin/env python

import unittest
import rospy, rostest
import time
import math
import actionlib

# Import ROS messages
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from maniros.msg import MoveCommand                                     # Locomotion control switches
from maniros.msg import MoveControl 					# Speed based control
# Locomotion control action
from maniros.msg import LocomotionAction
from maniros.msg import LocomotionGoal


class LocSimTest(unittest.TestCase):
    def __init__(self, *args):
        super(LocSimTest, self).__init__(*args)
        rospy.init_node("test_input", anonymous=True)
        # Get ROS parameters
        self.MAX_VEL            = rospy.get_param("/max_vel")           # Maximal wheel velocity [rad/s]
        self.MAX_ORT            = rospy.get_param("/max_ort")           # Maximal wheel orientation [rad]
        # Set working parameters
        self.tollerance         = 0.01                                  # Tollerance for the angle/velocity state
        self.wheelSpeed         = [0, 0, 0, 0]
        self.wheelAngle         = [self.MAX_ORT*5, self.MAX_ORT*5, self.MAX_ORT*5, self.MAX_ORT*5]

        self.velPub = [False, False, False, False]
        self.ortPub = [False, False, False, False]

        # Start up action client and wait for action server
        self.client = actionlib.SimpleActionClient("locomotion_control", LocomotionAction)
        self.client.wait_for_server()
        # Locomotion control subscriber
        # Wheel velocity
        rospy.Subscriber("/manisim/drive_fl_vel/command", Float64, self.drive_fl_vel, queue_size=1),
        rospy.Subscriber("/manisim/drive_rl_vel/command", Float64, self.drive_rl_vel, queue_size=1),
        rospy.Subscriber("/manisim/drive_rr_vel/command", Float64, self.drive_rr_vel, queue_size=1),
        rospy.Subscriber("/manisim/drive_fr_vel/command", Float64, self.drive_fr_vel, queue_size=1)
        # Wheel orientation
        rospy.Subscriber("/manisim/steer_fl_ort/command", Float64, self.steer_fl_ort, queue_size=1),
        rospy.Subscriber("/manisim/steer_rl_ort/command", Float64, self.steer_rl_ort, queue_size=1),
        rospy.Subscriber("/manisim/steer_rr_ort/command", Float64, self.steer_rr_ort, queue_size=1),
        rospy.Subscriber("/manisim/steer_fr_ort/command", Float64, self.steer_fr_ort, queue_size=1)
        # Joint velocity and orientation publisher
        self.joint_pub = rospy.Publisher("/manisim/joint_states", JointState, queue_size=10)

        # Publish locomotion state commands
        self.switch_pub = rospy.Publisher("teleop/lc_switch", MoveCommand, queue_size=10)

    # Drive callbacks
    def drive_fl_vel(self, data):
        self.wheelSpeed[0] = data
        self.velPub[0] = True
    def drive_rl_vel(self, data):
        self.wheelSpeed[1] = data
        self.velPub[1] = True
    def drive_rr_vel(self, data):
        self.wheelSpeed[2] = data
        self.velPub[2] = True
    def drive_fr_vel(self, data):
        self.wheelSpeed[3] = data
        self.velPub[3] = True
    # Steer callbacks
    def steer_fl_ort(self, data):
        self.wheelAngle[0] = data
        self.ortPub[0] = True
    def steer_rl_ort(self, data):
        self.wheelAngle[1] = data
        self.ortPub[1] = True
    def steer_rr_ort(self, data):
        self.wheelAngle[2] = data
        self.ortPub[2] = True
    def steer_fr_ort(self, data):
        self.wheelAngle[3] = data
        self.ortPub[3] = True

    def _goal_done(self, state, result):
        rospy.loginfo("Goal completed" + str(result.sequence))
        self.success = True

    def test_1_locomotion_publishers(self):
        called = False
        # Create move and joint commands
        mv_msg = MoveControl()
        mv_msg.header.stamp = rospy.Time.now()
        mv_msg.header.frame_id = "/cmd_vel";
        mv_msg.x = 1
        mv_msg.y = 0
        mv_msg.rz = 0
        mv_msg.header.stamp = rospy.Time.now()
        mv_msg.header.frame_id = "/manisim";

        jn_msg = JointState()
        jn_msg.name = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
        jn_msg.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        jn_msg.velocity = [0, 0, self.MAX_VEL, self.MAX_VEL, self.MAX_VEL, self.MAX_VEL, 0, 0, 0, 0]
        jn_msg.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # Send goal to the action server
        self.success = False
        goal = LocomotionGoal(command = mv_msg)
        self.client.send_goal(goal, done_cb=self._goal_done)

        # Check orientation publishers
        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        while not rospy.is_shutdown() and time.time() < timeout_t:
            if all(self.ortPub):
                called = True
                break
            rate.sleep()
        if called:
            # Check velocity publishers
            called = False
            while not rospy.is_shutdown() and time.time() < timeout_t:
                self.joint_pub.publish(jn_msg)
                if all(self.velPub):
                    called = True
                    break
                rate.sleep()
        else:
            called = False
        self.assertTrue(called, 'Locomotion action was not passed to simulation')


    def test_2_locomotion_action(self):
        called = False
        # Create move and joint commands
        mv_msg = MoveControl()
        mv_msg.header.stamp = rospy.Time.now()
        mv_msg.header.frame_id = "/cmd_vel";
        mv_msg.x = 0
        mv_msg.y = 0
        mv_msg.rz = 0
        mv_msg.header.stamp = rospy.Time.now()
        mv_msg.header.frame_id = "/manisim";

        jn_msg = JointState()
        jn_msg.name = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
        jn_msg.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        jn_msg.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        jn_msg.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Send goal to the action server
        self.success = False
        goal = LocomotionGoal(command = mv_msg)
        self.client.send_goal(goal, done_cb=self._goal_done)

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz
        # Check for action succeded callback
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.joint_pub.publish(jn_msg)			# Publish adequate joint state feedback
            rate.sleep()
        self.assertTrue(self.success, 'Locomotion action was not completed')

    def test_3_stop(self):
        # Motion commands
        x = 0
        y = 0
        # Create move and joint commands
        mv_msg = MoveControl()
        mv_msg.header.stamp = rospy.Time.now()
        mv_msg.header.frame_id = "/cmd_vel";
        mv_msg.x = x
        mv_msg.y = y
        mv_msg.rz = 0
        mv_msg.header.stamp = rospy.Time.now()
        mv_msg.header.frame_id = "/manisim";

        jn_msg = JointState()
        jn_msg.name = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
        jn_msg.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        jn_msg.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        jn_msg.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Send goal to the action server
        self.success = False
        goal = LocomotionGoal(command = mv_msg)
        self.client.send_goal(goal, done_cb=self._goal_done)

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz

        # Check orientation publisher
        while not rospy.is_shutdown() and time.time() < timeout_t:
            self.joint_pub.publish(jn_msg)
            if all(value == 0 for value in self.wheelAngle):
                break
            rate.sleep()
        # Check velocity publisher
        while not rospy.is_shutdown() and time.time() < timeout_t:
            self.joint_pub.publish(jn_msg)
            if all(value == 0 for value in self.wheelSpeed):
                break
            rate.sleep()
        # Check action callback
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            rate.sleep()
        self.assertTrue(self.success, 'Locomotion stop action failed')

    def test_4_motion(self):				# Note: test success is dependant on driving variable in tested class (test order is important)
        # Motion commands
        x = 0.5
        y = 0.7
        # Create move and joint commands
        mv_msg = MoveControl()
        mv_msg.header.stamp = rospy.Time.now()
        mv_msg.header.frame_id = "/cmd_vel";
        mv_msg.x = x
        mv_msg.y = y
        mv_msg.rz = 0
        mv_msg.header.stamp = rospy.Time.now()
        mv_msg.header.frame_id = "/manisim";

        jn_msg = JointState()
        jn_msg.name = ['cam_pan', 'cam_tilt' 'drive_1_fl', 'drive_2_rl', 'drive_3_rr', 'drive_4_fr', 'steer_1_fl', 'steer_2_rl', 'steer_3_rr', 'steer_4_fr']
        jn_msg.position = [0, 0, 0, 0, 0, 0, math.atan2(y,x), math.atan2(y,x), math.atan2(y,x), math.atan2(y,x)]
        jn_msg.velocity = [0, 0, math.hypot(x,y)*self.MAX_VEL, math.hypot(x,y)*self.MAX_VEL, math.hypot(x,y)*self.MAX_VEL, math.hypot(x,y)*self.MAX_VEL, 0, 0, 0, 0]
        jn_msg.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Bound for checking states
        upperAngle = [0, 0, 0, 0]
        lowerAngle = [0, 0, 0, 0]
        upperSpeed = [0, 0, 0, 0]
        lowerSpeed = [0, 0, 0, 0]
        for idx in range(0, len(self.wheelAngle)):
            upperAngle[idx] = jn_msg.position[idx+6] + 10*self.tollerance
            lowerAngle[idx] = jn_msg.position[idx+6] - 10*self.tollerance
            upperSpeed[idx] = jn_msg.velocity[idx+2] + self.tollerance
            lowerSpeed[idx] = jn_msg.velocity[idx+2] - self.tollerance

        # Send goal to the action server
        self.success = False
        goal = LocomotionGoal(command = mv_msg)
        self.client.send_goal(goal, done_cb=self._goal_done)
        self.joint_pub.publish(jn_msg)

        timeout_t = time.time() + 10.0  # 10 s
        rate = rospy.Rate(10)           # 10 Hz

        # Check orientation publisher
        exit = False
        while not rospy.is_shutdown() and time.time() < timeout_t and not exit:
            exit = True
            self.joint_pub.publish(jn_msg)
            for idx in range (0, len(self.wheelAngle)):
                if ((self.wheelAngle[idx] > upperAngle[idx]) or (self.wheelAngle[idx] <  lowerAngle[idx])):
                    exit = False
            rate.sleep()
        # Check celocity publisher
        exit = False
        while not rospy.is_shutdown() and time.time() < timeout_t and not exit:
            exit = True
            self.joint_pub.publish(jn_msg)
            for idx in range (0, len(self.wheelSpeed)):
                if ((self.wheelSpeed[idx] > upperSpeed[idx]) or (self.wheelSpeed[idx] <  lowerSpeed[idx])):
                    exit = False
            rate.sleep()
        # Check action callback
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            rate.sleep()
        self.assertTrue(self.success, 'Locomotion motion action failed')

if __name__ == '__main__':
    rostest.rosrun('maniros', 'test_teleop', LocSimTest)
