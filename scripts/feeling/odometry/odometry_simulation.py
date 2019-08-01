#!/usr/bin/env python

"""
This program provides a ROS node for converting Gazebo ModelState messages to Odometry messages.
Using Odometry messages the Rover position can be visualised in rviz and compared to odometry
from the wheel encoders.

Subscribed ROS topics:
*   gazebo/model_states
Published ROS topics:
*   gazebo/odom

"""

"""
Imports
"""
import math

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import tf_conversions

"""
Global variables
"""

PUB_RATE                = 1                                                         # Rate to publish odometry data [Hz]
wheelIndex              = ['front_left', 'rear_left', 'rear_right', 'front_right']  # Wheel location on rover

"""
Classes
"""

class OdometrySimulationPublisher(object):


    def __init__(self, name):
        # Joint velocity and orientation subscriber
        self.enc_sub = rospy.Subscriber("gazebo/model_states", EncoderOdometry, self.odometry_publisher, queue_size=10)
        # Transform broardcaster odom -> base_link
        self.odom_bcr = tf2_ros.TransformBroadcaster()
        # Odometry publisher base_link
        self.odom_pub = rospy.Publisher("gazebo/odom", Odometry, queue_size=50)

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


    def odometry_publisher(self, data):
        current_time = rospy.Time.now()                                 # Momentary time for odometry header

        # Create 6DOF transform from odometry yaw (rz rotation)
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = data.reference_frame
        t.transform.translation.x = data.pose.pose.position.x
        t.transform.translation.y = data.pose.pose.position.y
        t.transform.translation.z = data.pose.pose.position.z
        t.transform.rotation.x = data.pose.pose.quaternion.x
        t.transform.rotation.y = data.pose.pose.quaternion.y
        t.transform.rotation.z = data.pose.pose.quaternion.z
        t.transform.rotation.w = data.pose.pose.quaternion.w
        # Publish ROS transform
        self.odom_bcr.sendTransform(t)


        # Create Odometry message from ModelState message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = data.reference_frame
        odom.pose.pose = data.pose
        odom.twist.twist = data.twist
        # Publish ROS odometry message
        self.odom_pub.publish(odom)

    def shutdown(self):
        rospy.loginfo("Shutting down odometry node")

"""
Main
"""
if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("odometry_simulation_publisher", anonymous=True)

    # Start ROS action
    server = OdometrySimulationPublisher(rospy.get_name())
    rospy.spin()

rospy.on_shutdown(server.shutdown)
