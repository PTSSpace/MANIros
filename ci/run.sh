#!/bin/bash

# install dependencies
apt-get update
apt-get install -y i2c-tools libi2c-dev

# create workspace and load environment
mkdir ~/catkin_ws
ln -s /source ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/kinetic/setup.bash

# build package
catkin_make

