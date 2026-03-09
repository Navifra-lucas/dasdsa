cmake_minimum_required(VERSION 3.0.2)
project(task_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    message_generation
)

add_message_files(
  FILES
  # navi
  Charging.msg
  LiftInfo.msg
  PlcInfo.msg
  BatteryInfo.msg
  TbotBatteryInfo.msg
  IOInfo.msg
  Loading.msg
  LiftTask.msg
)

# add_service_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES 
  std_msgs  
  geometry_msgs
  nav_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime message_generation geometry_msgs std_msgs nav_msgs
)
add_library(${PROJECT_NAME}  msg/task_msg.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})