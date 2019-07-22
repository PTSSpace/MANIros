# This message contains high-level motor state information based on encoder feedback
# Wheel indexes [front left, rear left, rear right, front right]
Vector4Int drive_pulses					# Drive encoder pulses
Vector4Int steer_pulses					# Steer encoder pulses
Vector4Int drive_revolutions    		# Drive encoder wheel revolutions
Vector4 drive_velocity					# Drive encoder velocity [pulses per second]
Vector4 steer_velocity					# Steer encoder velocity [pulses per second]
#Vector4Int activity					# CAN node activity - Controller MCU
