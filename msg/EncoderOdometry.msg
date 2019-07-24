# This message contains high-level motor state information based on encoder feedback
# Wheel indexes [front_left, rear_left, rear_right, front_right]
int32[4] drive_pulses					# Drive encoder pulses
int32[4] steer_pulses					# Steer encoder pulses
int32[4] drive_revolutions    		# Drive encoder wheel revolutions
float64[4] drive_velocity					# Drive encoder velocity [pulses per second]
float64[4] steer_velocity					# Steer encoder velocity [pulses per second]
#int32[4] activity					# CAN node activity - Controller MCU
