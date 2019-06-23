cmake_minimum_required(VERSION 2.8.3)
project(auv_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  SixDoF.msg
  Thrust.msg
  Trajectory.msg
)

add_action_files(
  FILES
  TrajectoryGenerator.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    geometry_msgs
    message_runtime
    std_msgs
)