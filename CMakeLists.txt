cmake_minimum_required(VERSION 3.0.2)
project(tension_calibration)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS scripts/tension_calibrator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)