# This message contains high-level motor state information based on encoder feedback
# 4 speed values for 4 dc motors, normalized between 0 and 2,147,483,647 (4 byte integer)
Vector4 velocity    		# [front left, rear left, rear right, front right]
# 4 angles, represented in degree, for 4 dc motors, normalized between 0 and 2,147,483,647 (4 byte integer)
Vector4 orientation    		# [front left, rear left, rear right, front right]
