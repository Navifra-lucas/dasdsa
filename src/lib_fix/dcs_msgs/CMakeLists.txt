cmake_minimum_required(VERSION 3.0.2)
project(dcs_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
)

add_message_files(
  FILES
  DockingInfo.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime message_generation geometry_msgs std_msgs
)