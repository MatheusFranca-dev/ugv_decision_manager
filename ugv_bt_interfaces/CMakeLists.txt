cmake_minimum_required(VERSION 3.0.2)
project(ugv_bt_interfaces)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  message_runtime
  actionlib
  actionlib_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    Waypoint.msg
    BTStatus.msg
)

## Generate services in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
    Waypoint.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    actionlib_msgs
    std_msgs
    message_runtime
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
