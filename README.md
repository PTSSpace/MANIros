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
