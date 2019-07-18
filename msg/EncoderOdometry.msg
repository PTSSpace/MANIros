# This message contains high-level motor state information based on encoder feedback
# Wheel indexes [front left, rear left, rear right, front right]
Vector4Int pulses					# Encoder pulses from velocity motor
Vector4Int revolutions    				# Wheel revolutions from velocity motor
#Vector4 activity				# CAN node activity
Vector4 pulses_per_second			# velocity in pulses per second
