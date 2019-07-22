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
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

"""
Global variables
"""
current_time = rospy.Time.now()
last_time = rospy.Time.now()

PUB_RATE                = 1                                                         # Rate to publish odometry data [Hz]
wheelIndex          = ['front_left', 'rear_left', 'rear_right', 'front_right']      # Wheel location on rover

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

        # Joint velocity and orientation subscriber
        self.joint_sub = rospy.Subscriber("encoder_odometry", EncoderOdometry, self.get_encoder_values, queue_size=10)
        # Transform broardcaster odom -> base_link
        self.odom_bcr = tf.TransformBroadcaster()
        # Odometry publisher base_link
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        # Get ros parameters
        self.rover_length = rospy.get_param("/rover_length")
        self.rover_width = rospy.get_param("/rover_width")


    def get_encoder_values(self, data):
        # Get encoder values from encoder_odometry topic
        self.drive_pulses       = data.drive_pulses                     # Drive encoder pulses
        self.steer_pulses       = data.steer_pulses                     # Steer encoder pulses
        self.drive_revolutions  = data.drive_revolutions                # Drive encoder wheel revolutions
        self.drive_velocity     = data.drive_velocity                   # Drive encoder velocity [pulses per second]
        self.steer_velocity     = data.steer_velocity                   # Steer encoder velocity [pulses per second]


    def odometry_publisher(self):
        # Get ros parameters
        rover_length = rospy.get_param("/rover_length")
        rover_width = rospy.get_param("/rover_width")
        DRIVE_ENC_PPR           = rospy.get_param("/drive_enc_ppr")     # Drive encoder pulses per revolution
        STEER_ENC_PPR           = rospy.get_param("/steer_enc_ppr")     # Steer encoder pulses per revolution
        MAX_VEL                 = rospy.get_param("/max_vel")           # Maximal wheel velocity [rad/s]
        MAX_ORT                 = rospy.get_param("/max_ort")           # Maximal wheel orientation [rad]

        # Individual wheel orientation and velocity
        # Wheel indexes [front left, rear left, rear right, front right]
        wheelAngle              = [0, 0, 0, 0]                          # [rad]
        wheelSpeed              = [0, 0, 0, 0]                          # [rad/s]
        # Set ROS publisher rate
        r = rospy.Rate(PUB_RATE)
        while not rospy.is_shutdown():
            # Calculate steering angle [rad]
            wheelAngle = (self.steer_pulses/STEER_ENC_PPR-0.5)*MAX_ORT  # Driving forward 0 rad
            # Calculate driving velocity [rad/s]
            wheelSpeed = self.drive_velocity/DRIVE_ENC_PPR*MAX_VEL

            # Compute odometry from individual wheel velocities and orientations

            """
            Computes the components of the wheel velocity perpendicular to the rover hypotinuse
            the mean of the oposite perpendicular velocities:
            * fl and rr
            * rl and fr
            provides the rotational component of the wheel velocities.
            By middling both values the estimation of the roatational part of the locomotion in rz direction is acchieved.
            """
            for idx, wheel in enumerate(wheelIndex):
                perpendicularSpeed = math.cos(wheelAngle(idx)-math.atan2(rover_length/rover_width))



    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
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