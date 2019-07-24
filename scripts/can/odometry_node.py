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
import tf
from nav_msgs.msg import Odometry
from maniros.msg import EncoderOdometry
from maniros.msg import MotorContol
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


# Velocity based vector protocol
from vector_protocol.vector_protocol import VectorOdometry

"""
Global variables
"""
current_time = rospy.Time.now()
last_time = rospy.Time.now()

PUB_RATE                = 1                                                         # Rate to publish odometry data [Hz]
wheelIndex              = ['front_left', 'rear_left', 'rear_right', 'front_right']  # Wheel location on rover

"""
Classes
"""

class OdometrySimulation(object):


    def __init__(self, name):
        # Frame transformation odom -> base_link
        # Start in odom frame origin
        self.x      = 0.0
        self.y      = 0.0
        self.rz     = 0.0
        self.vx     = 0
        self.vy     = 0
        self.wrz    = 0

        self.drive_pulses           = 0                 # Drive encoder pulses
        self.steer_pulses           = 0                 # Steer encoder pulses
        self.drive_revolutions      = 0                 # Drive encoder wheel revolutions
        self.drive_velocity         = 0                 # Drive encoder velocity [pulses per second]
        self.steer_velocity         = 0                 # Steer encoder velocity [pulses per second]
        self.prev_drive_pulses      = 0
        self.prev_drive_revolutions = 0

        # Joint velocity and orientation subscriber
        self.joint_sub = rospy.Subscriber("encoder_odometry", EncoderOdometry, self.get_encoder_values, queue_size=10)
        # Transform broardcaster odom -> base_link
        self.odom_bcr = tf.TransformBroadcaster()
        # Odometry publisher base_link
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        # Get ros parameters
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


    def odometry_publisher(self):

        # Individual wheel orientation and velocity
        # Wheel indexes [front left, rear left, rear right, front right]
        wheelAngle              = [0, 0, 0, 0]                          # [rad]
        wheelSpeed              = [0, 0, 0, 0]                          # [rad/s]
        # Angle between perpendicular velocity to hypotinuse and zero steering orientation
        alpha = math.atan2(rover_width, rover_length)
        # Set ROS publisher rate
        r = rospy.Rate(PUB_RATE)
        while not rospy.is_shutdown():
            # TODO: Adjust positive negative values according to SIDE of encoder
            for idx, wheel in enumerate(wheelIndex):
                if index <= 1: #assigns a positive orientation to all left wheels
                    # Calculate steering angle [rad]
                    wheelAngle[idx] = (self.steer_pulses[idx]/self.STEER_ENC_PPR-0.5)*self.MAX_ORT      # Driving forward 0 rad
                else: #assigns a negative orientation to all right wheels
                    wheelAngle[idx] = -(self.steer_pulses[idx]/self.STEER_ENC_PPR-0.5)*self.MAX_ORT     # Driving forward 0 rad
                # Calculate driving velocity
                wheelSpeedRad = self.drive_velocity[idx]/self.DRIVE_ENC_PPR*self.MAX_VEL                # Driving velocity [rad/s]
                wheelSpeed[idx] = wheelSpeedRad*self.wheel_diameter/2                                   # Driving velocity  [m/s]



            # Compute rover velocity from individual wheel velocities and orientations
            vo = VectorOdometry(self.rover_length, self.rover_width)
            velOdm = MotorContol ()
            velOdm.driveValue = Vector4(*(wheelSpeed))                                                  # Wheel velocity [m/s]
            velOdm.steerValue = Vector4(*(wheelAngle))                                                  # Wheel rotation angle [rad]
            [self.vx, self.vy, self.wrz] = vo.calculateOdometry(odm)



            # Compute rover pose from individual distance traveled per wheel
            circ = math.pi*self.wheel_diameter                                                          # Wheel circumferance
            wheelDistance = (self.drive_pulses-self.prev_drive_pulses)/self.DRIVE_ENC_PPR*circ + (self.drive_revolutions-self.prev_drive_revolutions)*circ
            distOdm = MotorContol ()
            distOdm.driveValue = Vector4(*(wheelDistance))                                              # Wheel velocity [m]
            distOdm.steerValue = Vector4(*(wheelAngle))                                                 # Wheel rotation angle [rad]
            [dx, dy, drz] = vo.vo.calculateOdometry(distOdm)
            # Update pose information
            self.x += dx
            self.y += dy
            self.rz += drz
            # Update previous wheel state
            self.prev_drive_pulses = self.drive_pulses
            self.prev_drive_revolutions = self.drive_revolutions

            # Create 6DOF transform from odometry yaw (rz rotation)
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, rz)

            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # Publish ROS odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # Set position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # Set velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # Publish Message
            odom_pub.publish(odom)

            last_time = current_time
            r.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down odometry node")

"""
Main
"""
if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("odometry_simulation", anonymous=True)

    # Start ROS action
    server = LocomotionSimulation(rospy.get_name())
    rospy.spin()

rospy.on_shutdown(server.shutdown)