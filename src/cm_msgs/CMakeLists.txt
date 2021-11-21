cmake_minimum_required(VERSION 3.0.2)
project(cm_msgs)

# Find catkin macros and libraries, also other catkin packages.
find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    actionlib_msgs
    std_msgs
    message_generation
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg/bridge' folder
add_message_files(
  DIRECTORY msg/bridge
  FILES
  LED.msg
  Flash.msg
  Motor.msg
  TouchSensorsInfo.msg
  IRSensorsInfo.msg
  DisplayInfo.msg
  SoundSensorsInfo.msg
  MotorsInfo.msg
  LEDsInfo.msg
  RobotInfo.msg
  Response.msg
  Event.msg
  AudioInfo.msg
  Audio.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action/bridge' folder
add_action_files(
  DIRECTORY action/bridge
  FILES
  WriteOnSerial.action
  SetMotors.action
  SetTouchSensors.action
  SetSoundSensors.action
  SetIRSensors.action
  SetDisplay.action
  SetLEDs.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
)



###################################
## Catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cm_perception
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)