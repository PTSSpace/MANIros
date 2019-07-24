# This message contains high-level motor control information based on move commands
# Wheel indexes [front_left, rear_left, rear_right, front_right]
float64[4] driveValue				# Wheel distance [m]/velocity [m/s]
float64[4] steerValue    			# Wheel rotation angle [rad]/velocity [rad/s]
