#!/usr/bin/env python

"""
This program provides a ROS node for simulating locomotion encoders.
It reinterprets joint state outputs from the gazebo simulation
as quadrature encoders.
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
from maniros.msg import EncoderOdometry
from sensor_msgs.msg import JointState

"""
Global variables
"""
PUB_RATE 				= 50.0													# Rate to publish odometry data [Hz]

"""
Classes
"""

class EncoderSimulation():
	def __init__(self):
		# Get ROS parameters
		self.DRIVE_ENC_PPR   	= rospy.get_param("/drive_enc_ppr")				# Drive encoder pulses per revolution
		self.STEER_ENC_PPR   	= rospy.get_param("/steer_enc_ppr")				# Steer encoder pulses per revolution
		# Define working variables
		self.jointVelocity         = [0, 0, 0, 0, 0, 0, 0, 0]					# [rad/s]
		self.jointPosition         = [0, 0, 0, 0, 0, 0, 0, 0]					# [rad]
                self.activity              = [False, False, False, False]                               # Wheel joint node activity

		# Joint velocity and orientation subscriber
		rospy.Subscriber("/manisim/joint_states", JointState, self.get_joint_states, queue_size=10)
        # Odometry publisher base_link
		self.enc_pub = rospy.Publisher("encoder_odometry", EncoderOdometry, queue_size=10)
		self.timer = rospy.Timer(rospy.Duration(1.0/PUB_RATE), self.encoder_simulation_publisher)

	def get_joint_states(self, data):
		# Get joint state values from simulation
                self.jointPosition = data.position[2:10]
                self.jointVelocity = data.velocity[2:10]
                self.activity      = [True, True, True, True]

	def encoder_simulation_publisher(self, event):
		msg = EncoderOdometry()
		for i in range(0,4):
			[msg.drive_pulses[i], msg.drive_revolutions[i]] = self.drive_encoder_position(self.jointPosition[i])
			msg.drive_velocity[i] 							= self.drive_encoder_velocity(self.jointVelocity[i])
			msg.steer_pulses[i] 							= self.steer_encoder_position(self.jointPosition[i+4])
			msg.steer_velocity[i] 							= self.steer_encoder_velocity(self.jointVelocity[i+4])
                        msg.activity[i]                                                         = self.activity[i]
                        self.activity[i]                                                        = False
		self.enc_pub.publish(msg)

	def drive_encoder_position(self, drivePosition):
		# Convert position: rad -> (pulses, rotations)
		rotations 	= int(drivePosition /(2.0*math.pi))
		angle 		= drivePosition - (2.0*math.pi * rotations)
		pulses 		= int(angle/(2.0*math.pi)*self.DRIVE_ENC_PPR)
		return [pulses, rotations]

	def drive_encoder_velocity(self, driveVelocity):
		# Convert velocity: rad/s -> pulses/s
		ppr 		= int(driveVelocity/(2.0*math.pi)*self.DRIVE_ENC_PPR)
		return ppr

	def steer_encoder_position(self, steerPosition):
		# Convert position: rad -> pulses
		# Shift by a quater rotation that encoder pulses are only positive
		pulses 		= int((steerPosition/(math.pi/2.0)+1.0)*(self.STEER_ENC_PPR/4.0))
		return pulses

	def steer_encoder_velocity(self, steerVelocity):
		# Convert velocity: rad/s -> pulses/s
		ppr 		= int(steerVelocity/(2.0*math.pi)*self.STEER_ENC_PPR)
		return ppr

	def shutdown(self):
		rospy.loginfo("Shutting down odometry node")
"""
Main
"""
if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("odometry_simulation", anonymous=True)

    # Start ROS action
    encoder = EncoderSimulation()
    rospy.spin()

rospy.on_shutdown(encoder.shutdown)
