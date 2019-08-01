#!/usr/bin/env python

"""
This program provides a ROS node to pusblish odometry from the locomotion encoders.
Encoder information is receved in the form of pulses and revolution.
The values are transformed into an absolute rotation and translation of the /base_link frame.
The odometry forwarded to the navigation stack.

Subscribed ROS topics:
*   encoder_odometry
Published ROS topics:
*   odom

"""

"""
Imports
"""
import math

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from maniros.msg import EncoderOdometry
from maniros.msg import MotorControl
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import tf_conversions

# Velocity based vector protocol
from vector_odometry.vector_odometry import VectorOdometry

"""
Global variables
"""

PUB_RATE                = 1                                                         # Rate to publish odometry data [Hz]
wheelIndex              = ['front_left', 'rear_left', 'rear_right', 'front_right']  # Wheel location on rover

"""
Classes
"""

class OdometryPublisher(object):


    def __init__(self, name):
        # Frame transformation odom -> base_link
        # Start in odom frame origin
        self.x      = 0.0
        self.y      = 0.0
        self.rz     = 0.0
        self.vx     = 0
        self.vy     = 0
        self.wrz    = 0

        self.drive_pulses           = [0, 0, 0, 0]                 # Drive encoder pulses
        self.steer_pulses           = [0, 0, 0, 0]                 # Steer encoder pulses
        self.drive_revolutions      = [0, 0, 0, 0]                 # Drive encoder wheel revolutions
        self.drive_velocity         = [0, 0, 0, 0]                 # Drive encoder velocity [pulses per second]
        self.steer_velocity         = [0, 0, 0, 0]                 # Steer encoder velocity [pulses per second]
        self.prev_drive_pulses      = [0, 0, 0, 0]
        self.prev_drive_revolutions = [0, 0, 0, 0]

        # Joint velocity and orientation subscriber
        self.enc_sub = rospy.Subscriber("encoder_odometry", EncoderOdometry, self.get_encoder_values, queue_size=10)
        # Transform broardcaster odom -> base_link
        self.odom_bcr = tf2_ros.TransformBroadcaster()
        # Odometry publisher base_link
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        # Start ROS publisher for encoder odometry
        self.timer = rospy.Timer(rospy.Duration(1/PUB_RATE), self.odometry_publisher)

        # Get ros parameters
        self.rover_length       = rospy.get_param("/rover_length")      # Rover length [m]
        self.rover_width        = rospy.get_param("/rover_width")       # Rover width [m]
        self.DRIVE_ENC_PPR      = rospy.get_param("/drive_enc_ppr")     # Drive encoder pulses per revolution
        self.STEER_ENC_PPR      = rospy.get_param("/steer_enc_ppr")     # Steer encoder pulses per revolution
        self.MAX_VEL            = rospy.get_param("/max_vel")           # Maximal wheel velocity [rad/s]
        self.MAX_ORT            = rospy.get_param("/max_ort")           # Maximal wheel orientation [rad]
        self.wheel_diameter     = rospy.get_param("/wheel_diameter")    # Rover wheel diameter [m]


    def get_encoder_values(self, data):
        # Get encoder values from encoder_odometry topic
        self.drive_pulses       = data.drive_pulses                     # Drive encoder pulses
        self.steer_pulses       = data.steer_pulses                     # Steer encoder pulses
        self.drive_revolutions  = data.drive_revolutions                # Drive encoder wheel revolutions
        self.drive_velocity     = data.drive_velocity                   # Drive encoder velocity [pulses per second]
        self.steer_velocity     = data.steer_velocity                   # Steer encoder velocity [pulses per second]


    def odometry_publisher(self, event):
            # Individual wheel orientation and velocity
            # Wheel indexes [front left, rear left, rear right, front right]
            wheelAngle              = [0, 0, 0, 0]                          # [rad]
            wheelSpeed              = [0, 0, 0, 0]                          # [rad/s]
            # TODO: Adjust positive negative values according to SIDE of encoder
            for idx, wheel in enumerate(wheelIndex):
                if idx <= 1: #assigns a positive orientation to all left wheels
                    # Calculate steering angle [rad]
                    wheelAngle[idx] = (self.steer_pulses[idx]/self.STEER_ENC_PPR-0.5)*self.MAX_ORT      # Driving forward 0 rad
                else: #assigns a negative orientation to all right wheels
                    wheelAngle[idx] = -(self.steer_pulses[idx]/self.STEER_ENC_PPR-0.5)*self.MAX_ORT     # Driving forward 0 rad
                # Calculate driving velocity
                wheelSpeedRad = self.drive_velocity[idx]/self.DRIVE_ENC_PPR*self.MAX_VEL                # Driving velocity [rad/s]
                wheelSpeed[idx] = wheelSpeedRad*self.wheel_diameter/2                                   # Driving velocity  [m/s]



            # Compute rover velocity from individual wheel velocities and orientations
            vo = VectorOdometry(self.rover_length, self.rover_width)
            velOdm = MotorControl ()
            velOdm.driveValue = wheelSpeed                                                  # Wheel velocity [m/s]
            velOdm.steerValue = wheelAngle                                                  # Wheel rotation angle [rad]
            [self.vx, self.vy, self.wrz] = vo.calculateOdometry(velOdm)



            # Compute rover pose from individual distance traveled per wheel
            circ = math.pi*self.wheel_diameter                                                          # Wheel circumferance
            wheelDistance = [0, 0, 0, 0]
            for idx, wheel in enumerate(wheelIndex):
                wheelDistance[idx] = (self.drive_pulses[idx]-self.prev_drive_pulses[idx])/self.DRIVE_ENC_PPR*circ + (self.drive_revolutions[idx]-self.prev_drive_revolutions[idx])*circ
            distOdm = MotorControl ()
            distOdm.driveValue = wheelDistance                                              # Wheel velocity [m]
            distOdm.steerValue = wheelAngle                                                 # Wheel rotation angle [rad]
            [dx, dy, drz] = vo.calculateOdometry(distOdm)
            # Update pose information
            self.x += dx
            self.y += dy
            self.rz += drz
            # Update previous wheel state
            self.prev_drive_pulses = self.drive_pulses
            self.prev_drive_revolutions = self.drive_revolutions

            current_time = rospy.Time.now()
            # Publish 6DOF transform from odometry yaw (rz rotation)
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.rz)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.odom_bcr.sendTransform(t)

            # Publish ROS odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*q))
            odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.wrz))
            self.odom_pub.publish(odom)


    def shutdown(self):
        rospy.loginfo("Shutting down odometry node")

"""
Main
"""
if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("odometry_publisher", anonymous=True)

    # Start ROS action
    server = OdometryPublisher(rospy.get_name())
    rospy.spin()

rospy.on_shutdown(server.shutdown)
