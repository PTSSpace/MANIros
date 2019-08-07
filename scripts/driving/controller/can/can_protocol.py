"""
This header provides variables for a rudamentary CAN protocol.
All messages are bounded to 8 bytes and subdivided into either 1 byte values for boleans
or 2 byte values for signed integers.

All booleans represent state switch information. All integer values passed to or received
from the bus are given in the following formats:
Electrical Power Supply:
	* normed values to fit an int16 (current sensor feedback)
Locomotion Control:
	* either encoder pulses or pulses per second (encoder velocity or orientation)
	* counters (drive encoder rotations)
The maximum value of 32768 for a signed 16 bit integer will not be exeeded due to the limited
range of the encoders (about 8384 pulses per second) and the limit maximum velocity of the
motors (about 3 rounds per second). Furthermore the current sensor values will be scaled
and to int16 which allows for a greater subdivision than the sensors own sensitvity.

Messages with lower numeric values for their ID's have higher priority on the CAN network.
All message ID's are given in Hexadecimal. To ensure the priority of specified commands,
each command has its own range denoted by the letter in the hex numbers.
Each command further has its own indentifier number to indicate which node it is specified for
or originating from. The OBC is the only communication point to the other nodes and does
therefore not need an indentifier number.

ID List:
| Position/function on rover | Indentifier number | ID's  |
|----------------------------|--------------------|-------|
| electrical power supply    | 0                  | 0xXX0 |
| front_left wheel           | 1                  | 0xXX1 |
| rear_left wheel            | 2                  | 0xXX2 |
| rear_right wheel           | 3                  | 0xXX3 |
| front_right wheel          | 4                  | 0xXX4 |
"""

"""
Global variables
"""
# Set CAN protocol parameters
BITRATE           	= 500000                                                     	# CAN bus bit rate

# Message header ID's:

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

MAX_VALUE        	= 32768                                                 		# Maximal signed value for 2 bytes