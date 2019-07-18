"""
Imports
"""
import math

"""
Global variables
"""
# Set CAN protocol parameters
BITRATE           = 500000                                                          # CAN bus bit rate

# Message header ID's:
"""
Messages with lower numeric values for their ID's
have higher priority on the CAN network
"""

# Electrical Power Supply (EPS)
powerCmd            = [0x000]                                                       # Motor power on/off command
errorWrn            = [0x010]                                                       # Sensor overcurrent warning (fault line latched low)
currentWrn          = [0x020]                                                       # Critical current warning (80 percent of max current)
powerFd            	= [0x030]                                                       # Motor power status feedback/update
currentFd          	= [0x0E0]                                                       # Feedback from current sensors
currentSensorIndex	= ['battery','front_left', 'rear_left', 'rear_right', 'front_right']

# Locomotion Control (LC)

wheelIndex          = ['front_left', 'rear_left', 'rear_right', 'front_right']      # Wheel location on rover
switchCmd      		= [0x0B1, 0x0B2, 0x0A3, 0x0B4]                                  # Motor/PID start/stop command, initialization and odometry publisher command
orientationCmd      = [0x0C1, 0x0C2, 0x0B3, 0x0C4]                                  # Set orientation locomotion command
velocityCmd         = [0x0D1, 0x0D2, 0x0C3, 0x0D4]                                  # Set velocity locomotion command
locomotionFb      	= [0x0E1, 0x0E2, 0x0D3, 0x0E4]                                  # Locomotion task accomplished feedback
odometryFb       	= [0x0F1, 0x0F2, 0x0E3, 0x0F4]                                  # Odometry locomotion feedback

"""
wheelIndex          = ['front_left']
switchCmd      		= [0x0A1]
orientationCmd      = [0x0B1]
velocityCmd         = [0x0C1]
locomotionFb 		= [0x0D1]
odometryFb			= [0x0E1]
"""

MAX_VALUE        	= 2147483647                                                 	# Maximal signed value for 4 bytes

# Set EPS parameters
MAX_CUR_B			= 25															# Maximal current on battery current sensor [A]
MAX_CUR_M			= 12															# Maximal current on motor current sensor [A]

# Set motor parameters
MAX_VEL     		= math.pi                                                     # Maximal wheel velocity [rad/s]
MAX_ORT  			= math.pi/2                                                     # Maximal wheel orientation [rad]
