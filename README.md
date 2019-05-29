# MANIros
Base ROS package to develop with the MANI rover. Provides a high level api for the available sensors and motors.

## Installation

To get all the required dependencies on your jetson board [these](https://github.com/PTScientists/MANIansible) ansible
scripts can be used, after that place this package at `~/catkin_ws/src/maniros`. If you prefere the latest development
version use the `develop` branch, otherwise pick `master`.

## Development

Development is done on topic branches, if you work on a new feature create your own branch and start working. After your
done create a pull request against the `develop` branch. If all tests complete successfully, and there are additional
ones for new features, it can be reviewed and merged into master. 

## CAN Bus
The CAN interface is set to **can0** but may be set to **can1** if desired.
The adequate pins are:

| Interface | CAN_STBY | CAN_RX | CAN_TX|
|:----------|:----------:|:--------:|:-------:|
| can0 | Pin 4 | Pin 5| Pin 7 |
| can0 | Pin 14 | Pin 15 | Pin 17 |

### Parameters

The CAN bus bit rate is adjsuted by setting **FREQUENCY**.
The maximum wheel velocity **MAX_VEL** can be set at the top of the Pyhton code on the OBC or in the defines.h file on the Drive Nodes.
If the parameters are changed, they must be adjusted on all nodes connected to the bus.

#### Message ID's:

Messages with lower numeric values for their ID's have higher priority on the CAN network. All message ID's are given in Hexadecimal.
To ensure the priority of specified commands, each command has its own range denoted by the letter in the hex numbers.
Each command further has its own indentifier number to indicate which node it is specified for or originating from. The OBC is the only communication point to the other nodes and does therefore not need an indentifier number.

##### ID List

| Wheel location on rover | Indentifier number | ID's |
|:------------------------|:------------------:|:----:|
| front_left | 1 | 0xXX1 |
| rear_left | 2 | 0xXX2 |
| rear_right | 3 | 0xXX3 |
| front_right | 4 | 0xXX4 |

##### Command List

| Message | ID's | Description | Sender | Receiver | Data length | Data division |
|:--------|:----:|:-----------:|:------:|:--------:|:-----------:|:-------------:|
| powerCmd | 0x0AX | Power switch command for steering/driving motor | OBC| Drive Node | 8 byte | steerMode \[0,1\] (bytes 1 to 4) driveMode \[0,1\] (bytes 5 to 8) |
| initialliseCmd | 0x0BX | Initialisatin command for odometry publisher and zeroing steering encoders | OBC| Drive Node | 8 byte | publisherMode \[0,1\] (bytes 1 to 4) zeroEncoders \[0,1\] (bytes 5 to 8) |
| orientationCmd | 0x0CX | Set orientation command | OBC| Drive Node | 4 byte | set_orientation \[-2147483647..2147483647\] (bytes 1 to 4) |
| velocityCmd | 0x0DX | Set velocity command | OBC| Drive Node | 4 byte | set_velocity \[-2147483647..2147483647\] (bytes 1 to 4) |
| orientationOdm | 0x0EX | Odometry feedback for reached steering orientation | Drive Node | OBC | 4 byte | current_orientation \[-2147483647..2147483647\] (bytes 1 to 4) |
| velocityOdm | 0x0FX | Odometry feedback of absolute encoder counts for rover distance traveled | Drive Node | OBC | 8 byte | pulses \[-2147483647..2147483647\] (bytes 1 to 4) revolutions \[-2147483647..2147483647\] (bytes 5 to 8) |

**The velocity and orientation are scaled values based on the maximal velocity and orientation, respectively.**
