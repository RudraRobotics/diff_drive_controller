cmake_minimum_required(VERSION 3.0.2)
project(medicbot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_control_boilerplate
)

catkin_package()
    
include_directories(
  include/
  libs/arduino_serial/
  ${catkin_INCLUDE_DIRS}
)

set(ARDUINO_SRCS
	libs/arduino_serial/arduino_serial.c
	libs/arduino_serial/rs232.c
)
add_library(arduino_serial SHARED ${ARDUINO_SRCS})

# Medicbot Hardware Interface
add_library(medicbot_hw_interface SHARED
  src/medicbot_hw_interface.cpp
)
target_link_libraries(medicbot_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(medicbot_hw_interface_node src/medicbot_hw_main.cpp)
target_link_libraries(medicbot_hw_interface_node
  medicbot_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
  arduino_serial
)
