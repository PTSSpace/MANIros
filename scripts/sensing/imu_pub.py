#!/usr/bin/env python

# -----------------------
# Import required Python libraries
# -----------------------
import rospy
import numpy
import smbus2 
import math
from sensor_msgs.msg import Imu

# -----------------------
# Constants
# -----------------------

bus_num 		= 1         # Bus number
address 		= 0x68      # Device adresss

# Register adresses
pwr_mgmt_1		= 0x6b		# Power Management 1
pwr_mgmt_1		= 0x6c		# Power Management 2 
smplrt_div   	= 0x19		# Sample Rate Divider
config       	= 0x1a		# Configuration
gyro_config  	= 0x1b		# Gyro Configuration
int_enable   	= 0x38		# Interrupt Enable
# Acceleration values (8 most significant bits)
# following 8 bits at adress+1 
accel_xout_h	= 0x3b
accel_yout_h	= 0x3d
accel_zout_h	= 0x3f
# Gyro values (8 most significant bits)
# following 8 bits at adress+1 
gyro_xout_h  	= 0x43
gyro_yout_h		= 0x45
gyro_zout_h		= 0x47
 
# -----------------------
# Functions
# -----------------------

def read_raw_data(reg):
	# 16-bit values from Gyro and Accelerometer
    high = bus.read_byte_data(address, reg)		# 8 most significant bits
    low = bus.read_byte_data(address, reg+1)	# 8 least significant bits
    # concatenate higher and lower value
    value = ((high << 8) | low)  
    # get signed value from mpu6050
    if(value > 32768):
            value = value - 65536	# 16-bit maximum (0b11111111)
    return value

def read_imu_data():
	# Read Accelerometer raw value
	acc_x = read_raw_data(accel_xout_h)
	acc_y = read_raw_data(accel_yout_h)
	acc_z = read_raw_data(accel_zout_h)

	# Read Gyroscope raw value
	gyro_x = read_raw_data(gyro_xout_h)
	gyro_y = read_raw_data(gyro_yout_h)
	gyro_z = read_raw_data(gyro_zout_h)

	# Generate ROS IMU message
	imu_msg = Imu()
	imu_msg.header.stamp = rospy.Time.now()
	imu_msg.header.frame_id = '/imu'

	# Scale raw values with IMU meassurement ranges
	imu_msg.orientation_covariance[0] = -1
	imu_msg.angular_velocity.x = gyro_x / 131.0
	imu_msg.angular_velocity.y = gyro_y / 131.0
	imu_msg.angular_velocity.z = gyro_z / 131.0
	imu_msg.linear_acceleration.x = acc_x / 16384.0
	imu_msg.linear_acceleration.y = acc_y / 16384.0
	imu_msg.linear_acceleration.z = acc_z / 16384.0

	return imu_msg


def imu_publisher():
	pub = rospy.Publisher('imu', Imu, queue_size=10)
	rospy.init_node('imu_publisher', anonymous=True)
	rate = rospy.Rate(1) # 1hz
	while not rospy.is_shutdown():
		msg = read_imu_data() # IMU data message
		pub.publish(msg)
		rate.sleep()

def shutdown():
	print('Shutting down node...')

# -----------------------
# Main
# -----------------------

if __name__=="__main__": 
	bus = smbus2.SMBus(bus_num) # Open I2C bus

	# Aktivieren, um das Modul ansprechen zu koennen
	bus.write_byte_data(address, smplrt_div, 7) # 1Hz rample rate (= accelerometer output rate)
	bus.write_byte_data(address, pwr_mgmt_1, 0) # Start internal clock (0b00000000)
	bus.write_byte_data(address, config, 0) # External input disabled and filter set (0b00000000)
	bus.write_byte_data(address, gyro_config, 0) # Set full scale range to 250 deg/s and disable self-testing (0b00000000)
	bus.write_byte_data(address, int_enable, 1) # Set data ready interrupt (0b00000001)
	rospy.loginfo('Starting IMU measurements...')


	# Get ros parameters
	#rospy.get_param('/')	# get frame_id for tf 

	try:
	    imu_publisher()
	except rospy.ROSInterruptException:
	    pass

	rospy.on_shutdown(shutdown)
