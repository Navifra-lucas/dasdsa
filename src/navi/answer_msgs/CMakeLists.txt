cmake_minimum_required(VERSION 3.16)
project(answer_msgs)
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  DriveInfo.msg
  NodeInfo.msg
  EdgeInfo.msg
  MissionNodeInfo.msg
  MissionProgress.msg
  MotionStatus.msg
  NaviInfo.msg
  MissionAlign.msg
  MissionNodePath.msg
  MissionExplore.msg
  MissionResponse.msg
)

add_service_files(
  FILES
  CommonService.srv
  GlobalMap.srv
  GraphLocalization.srv
  MissionGraphNode.srv
  MissionGraphNodeList.srv
)

add_action_files(
  FILES
  MissionOrder.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  nav_msgs
)