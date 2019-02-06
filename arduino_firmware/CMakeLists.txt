cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

generate_arduino_firmware(motor_driver_nano
  SRCS motor_driver.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD nano328 # or mega2560 (see: https://creative-chaos.com/2013/02/06/board-ids-to-use-with-cmake-for-arduino/ for all ID's)
  PORT /dev/ttyUSB0 # or /dev/ttyACM0 (see: $~: ls /dev/tty* for device port)
)
