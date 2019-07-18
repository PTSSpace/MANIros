#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

"""
Classes
"""

class OdometrySimulation(object):


    def __init__(self, name):

        # Joint velocity and orientation subscriber
        self.joint_sub = rospy.Subscriber("encoder_odometry", ???, queue_size=10)
        # Transform broardcaster odom -> base_link 
        self.odom_bcr = tf.TransformBroadcaster()
        # Odometry publisher base_link
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)


        # Get ros parameters
        self.rover_length = rospy.get_param("/rover_length")
        self.rover_width = rospy.get_param("/rover_width")

    def get_joint_states(self, data):
        # Get joint state values from simulation
        self.wheelAngle = data.position[6:10]
        self.wheelSpeed = data.velocity[2:6]


"""
    def get_encoder_odometry(self):
        # Get message values from listener
        msg = EncoderOdometry()
        msg.pulses = Vector4(*(self.ci.listener.pulses))
        msg.revolutions = Vector4(*(self.ci.listener.revolutions))
        msg.activity = Vector4(*(self.ci.listener.activity[1:5]))
        return msg
"""


    def CAN_subscriber(self, event):
        # Check if nodes are initialised
        if self.lcInitialised:
            # Check for node failure
            if any(self.ci.listener.activity[1:5]) == 0:
                rospy.loginfo("LC \t Error CAN Drive node died")
                rospy.loginfo(' '.join(map(str, self.ci.listener.activity[1:5])))
            # Publish odometry message
            msg = self.get_encoder_odometry() # IMU data message
            self.encoder_pub.publish(msg)
                    # Check for node activity
        else:
            rospy.loginfo("LC \t Initialise CAN Drive nodes")
            self.lcInitialised = True
            self.drive_node_initialise()
        # Set node activity in listener
        for idx in range (1,5):
            self.ci.listener.activity[idx] = 0


x = 0.0
y = 0.0
th = 0.0

vx = 0.1
vy = -0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(1.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

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