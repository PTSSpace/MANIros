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
PULSES_PER_REV          = 8384

MAX_VEL                 = math.pi                                            	# Maximal wheel velocity [rad/s]
MaxOrientation          = math.pi                          						# Maximal wheel orientation [rad]
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
		self.wheelSpeed         = [0, 0, 0, 0]		#rad/s
		self.wheelAngle         = [0, 0, 0, 0]		#rad
		# Joint velocity and orientation subscriber
		self.joint_sub = rospy.Subscriber("/manisim/joint_states", JointState, self.get_joint_states, queue_size=10)
        # Odometry publisher base_link
		self.enc_pub = rospy.Publisher("encoder_odometry", EncoderOdometry, queue_size=10)

	def get_joint_states(self, data):
		# Get joint state values from simulation
		self.wheelAngle = data.position[6:10]
		self.wheelSpeed = data.velocity[2:6]

	def shutdown(self):
		rospy.loginfo("Shutting down odometry node")

	def rad_to_pulse(vel):
		return vel/(2*math.pi)*PULSES_PER_REV

	def revolutions_counter(wheelAngle,wheelSpeed):
		if(wheelAngle-Angle_Old>1.5*math.pi and wheelSpeed>0):
			Rotations=Rotations+1
			Angle_Old=wheelAngle

		if(Angle_Old-wheelAngle>1.5*math.pi and wheelSpeed<0):
			Rotations=Rotations-1
			Angle_Old=wheelAngle
    	
		return Rotations




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

#current_time = rospy.Time.now()
#last_time = rospy.Time.now()
