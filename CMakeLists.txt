cmake_minimum_required(VERSION 3.0.2)
project(tension_calibration)

find_package(catkin REQUIRED COMPONENTS
  ros_dynamixel_pkg
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tension_calibration
#  CATKIN_DEPENDS ros_dynamixel_pkg roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
