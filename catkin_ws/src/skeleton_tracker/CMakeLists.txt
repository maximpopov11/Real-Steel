cmake_minimum_required(VERSION 3.0.2)
project(skeleton_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cv_bridge
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs geometry_msgs cv_bridge std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/skeleton_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
