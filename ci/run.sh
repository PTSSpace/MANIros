#!/bin/bash

# install dependencies
apt-get update
apt-get install -y i2c-tools libi2c-dev

# source environment
source /opt/ros/kinetic/setup.bash

# install ros dependencies
apt-get install ros-kinetic-geometry
apt-get install ros-kinetic-geometry2

# create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source ~/devel/setup.bash

# link package
ln -s /source ~/catkin_ws/src/maniros

# build package
catkin_make

source ~/catkin_ws/devel/setup.bash

# run tests
./src/maniros/tests/run.sh

