#!/bin/bash

# install dependencies
apt-get update
apt-get install -y i2c-tools libi2c-dev

# source environment
source /opt/ros/kinetic/setup.bash

# create workspace
mkdir -p ~/catkin_ws/src
ln -s /source ~/catkin_ws/src
cd ~/catkin_ws

# build package
catkin_make

