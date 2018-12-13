#!/bin/bash

# install dependencies
apt-get update
apt-get install -y i2c-tools libi2c-dev

# source environment
source /opt/ros/kinetic/setup.bash

# create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source ~/devel/setup.bash

# link package
ln -s /source ~/catkin_ws/src/maniros

# build package
catkin_make

