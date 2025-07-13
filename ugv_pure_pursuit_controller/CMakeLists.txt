cmake_minimum_required(VERSION 2.8.3)
project(ugv_pure_pursuit_controller)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  ackermann_msgs
  visualization_msgs
  ugv_bt_interfaces
  roslint
)

catkin_package(
 CATKIN_DEPENDS
 ugv_bt_interfaces
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(
  PROGRAMS
    scripts/pure_pursuit_action.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslint_python()
