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
from maniros.msg import EncoderOdometry
from sensor_msgs.msg import JointState

"""
Global variables
"""

PULSES_PER_REV          = rospi.get_param("/drive_enc_ppr")

MAX_VEL                 = rospi.get_param("/max_vel")                           # Maximal wheel velocity [rad/s]
MaxOrientation          = rospi.get_param("/max_ort")                          						# Maximal wheel orientation [rad]
MaxVelEnc 				= MAX_VEL * PULSES_PER_REV/(2*MaxOrientation)   		# Maximal wheel velocity [encoder pulses per second]
MaxOrEnc				= PULSES_PER_REV/2                            			# Maximal wheel orientation [encoder pulses]

PUB_RATE 				= 5														# Rate to publish odometry data [Hz]

Angle_Old 				= 0														# last knows wheelAngle
Rotations 				= 0														# counter for full rotations scince start

"""
Classes
"""

class EncoderSimulation():
	def __init__(self):
		self.jointSpeed         = [0, 0, 0, 0, 0, 0, 0, 0]		#rad/s
		self.jointAngle         = [0, 0, 0, 0, 0, 0, 0, 0]		#rad
		# Joint velocity and orientation subscriber
		rospy.Subscriber("/mani/joint_states", JointState, self.get_joint_states, queue_size=10)
        # Odometry publisher base_link
		self.enc_pub = rospy.Publisher("encoder_odometry", EncoderOdometry, queue_size=10)

		self.timer = rospy.Timer(rospy.Duration(1/PUB_RATE), self.encoder_simulation_publisher)

	def get_joint_states(self, data):
		# Get joint state values from simulation
		self.jointAngle = data.position[2:10]
		self.jointSpeed = data.velocity[2:10]


	def encoder_simulation_publisher(self):
		msg = EncoderOdometry()
		for i in range(0,4):
			msg.drive_pulses[i] = wheelAngle_to_pulse(self, jointAngle[i])

		for i in range(0,4):
			msg.steer_pulses[i] = joint_angle_to_joint_pulses(self, jointAngle[i+4])

		for i in range(0,4):
			msg.drive_revolutions[i] = revolutions_counter(self, jointAngle[i], jointSpeed[i])

		for i in range(0,4):
			msg.drive_velocity[i] = rad_to_pulse(self, jointSpeed[i])

		for i in range(0,4):
			msg.steer_velocity[i] = rad_to_pulse(self, jointSpeed[i+4])

		self.enc_pub.publish(msg)

	def shutdown(self):
		rospy.loginfo("Shutting down odometry node")

	def rad_to_pulse(self, angle):												# position from driving wheel and velocity
		return angle/(2*math.pi)*PULSES_PER_REV

	def joint_angle_to_joint_pulses(self, angle):									#steeringangle ONLY
		return (PULSES_PER_REV/(2*math.pi))*angle+PULSES_PER_REV/4

	def revolutions_counter(self, wheelAngle):									# +1/-1 after a full rotation (8384 pulses)
		return Rotations=int(wheelAngle/(2*math.pi))

	def wheelAngle_to_pulse(self, wheelAngle):
		rad = wheelAngle-(2*math.pi*revolutions_counter(self, wheelAngle))
		return rad_to_pulse(self, rad)
    	


"""
Main
"""
if __name__ == '__main__':
    # Start ROS node
    rospy.init_node("odometry_simulation", anonymous=True)

    # Start ROS action
    encoder = EncoderSimulation()
    rospy.spin()

rospy.on_shutdown(server.shutdown)
