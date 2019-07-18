#!/usr/bin/env python

"""
This program provides a ROS node for simulating locomotion encoders.
It reinterprets joint from the gazebo simulation as quadrature encoders.
The encoder values are forwarded to the odometry publisher.

Subscribed ROS topics:
*   mani/joint_states
Published ROS topics:
*   encoder_odometry

"""

"""
Imports
"""
import math
import rospy

"""
Global variables
"""
PULSES_PER_REV          8384

MAX_VEL                 = 3                                            			# Maximal wheel velocity [rad/s]
MaxOrientation          = math.pi                          						# Maximal wheel orientation [rad]
MaxVelEnc 				= MAX_VEL * PULSES_PER_REV/(2*MaxOrientation)   		# Maximal wheel velocity [encoder pulses per second]
MaxOrEnc				= PULSES_PER_REV/2                            			# Maximal wheel orientation [encoder pulses]

PUB_RATE 				= 5														# Rate to publish odometry data [Hz]

"""
Classes
"""

class EncoderSimulation():
	def __init__(self, name):
        # Joint velocity and orientation subscriber
        self.joint_sub = rospy.Subscriber("/mani/joint_states", JointState, self.get_joint_states, queue_size=10)
        # Odometry publisher base_link
        self.enc_pub = rospy.Publisher("encoder_odometry", ???, queue_size=10)

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