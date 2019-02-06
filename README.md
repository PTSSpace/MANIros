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

## Arduino

The Arduino needs the ros_lib library, including the custom message header.

If you installed the Arduino IDE a sketchbok folder already exists, otherwise create `sketchbook/libraries` in your homefolder.

Then run

```
cd catkin_ws && source devel/setup.sh
rm -rf </home/nvidia/>sketchbook/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py </home/nvidia/>sketchbook/libraries
```


You also need to change the path in `maniros/arduino_firmware/motor_driver.cpp`
in line 3 to match your path to the ros_lib

Change the BOARD and PORT in `maniros/arduino_firmware/CMakeLists.txt` to reflect your Arduino Board and USB Port.

To upload a sketch, run:

```
catkin_make
catkin_make maniros_arduino_firmware_motor_driver_nano-upload
```

