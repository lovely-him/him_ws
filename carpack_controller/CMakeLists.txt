cmake_minimum_required(VERSION 3.0.2)
project(carpack_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  gazebo_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2_ros
  roscpp
  rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ackermann_msgs#
  geometry_msgs#
  geometry_msgs#
  gazebo_msgs#
  sensor_msgs#
  nav_msgs#
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES carpack_controller
  CATKIN_DEPENDS ackermann_msgs gazebo_msgs geometry_msgs geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS
#   scripts/car_controller.py
   scripts/motor_pwm.py
   scripts/key_get.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )