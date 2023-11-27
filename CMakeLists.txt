cmake_minimum_required(VERSION 3.0.2)
project(tension_calibration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
