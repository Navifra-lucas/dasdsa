cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(vanjee_lidar_sdk)

#=======================================
# Compile Type (Release, Debug)
#=======================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

#=======================================
# Custom Point Type (XYZI, XYZIRT, XYZIRTT)
#=======================================
set(POINT_TYPE XYZIRT)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
# 기본값은 유지하되, ROS1이 탐지되면 CATKIN으로 강제 전환
#=======================================
set(COMPILE_METHOD CATKIN)

#========================
# Dependencies: 먼저 ROS1/ROS2 탐지
#========================
find_package(roscpp 1.12 QUIET)    # ROS1
find_package(rclcpp QUIET)         # ROS2

# ROS1이 보이고 ROS2가 아니면 CATKIN으로 스위치
if(roscpp_FOUND AND NOT rclcpp_FOUND)
  set(COMPILE_METHOD CATKIN)
endif()

#========================
# C++ 표준
#========================
if($ENV{ROS_DISTRO} STREQUAL "humble")
  set(CMAKE_CXX_STANDARD 17)
else()
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#========================
# 옵션들 (그대로)
#========================
option(DISABLE_PCAP_PARSE "Disable PCAP file parse" ON)
if(DISABLE_PCAP_PARSE)
  message(=============================================================)
  message("-- Disable PCAP parse")
  message(=============================================================)
  add_definitions(-DDISABLE_PCAP_PARSE)
else()
  if(WIN32)
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
    find_package(PCAP REQUIRED)
    include_directories(${PCAP_INCLUDE_DIR})
    list(APPEND EXTERNAL_LIBS ${PCAP_LIBRARY})
  else()
    list(APPEND EXTERNAL_LIBS pcap)
  endif()
endif()

option(ENABLE_PCL_POINTCLOUD "Enable PCL Point Cloud" OFF)
if(ENABLE_PCL_POINTCLOUD)
  add_definitions(-DENABLE_PCL_POINTCLOUD)
  find_package(PCL REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
endif()

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(ENABLE_TRANSFORM)
  add_definitions(-DENABLE_TRANSFORM)
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

option(ENABLE_GTEST "Enable gtest value" OFF)
if(ENABLE_GTEST)
  add_definitions(-DENABLE_GTEST)
endif()

option(ENABLE_TBB "Enable TBB" OFF)
if(ENABLE_TBB)
  add_definitions(-DENABLE_TBB)
  find_package(TBB REQUIRED)
  include_directories(${TBB_INCLUDE_DIR})
endif()

option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
if(ENABLE_EPOLL_RECEIVE)
  add_definitions(-DENABLE_EPOLL_RECEIVE)
endif()

option(ENABLE_DOUBLE_RCVBUF "Enable double size of RCVBUF" OFF)
if(ENABLE_DOUBLE_RCVBUF)
  add_definitions(-ENABLE_DOUBLE_RCVBUF)
endif()

option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
if(ENABLE_WAIT_IF_QUEUE_EMPTY)
  add_definitions(-DENABLE_WAIT_IF_QUEUE_EMPTY)
endif()

option(ENABLE_STAMP_WITH_LOCAL "Enable stamp point cloud with local time" OFF)
if(ENABLE_STAMP_WITH_LOCAL)
  add_definitions(-DENABLE_STAMP_WITH_LOCAL)
endif()

option(ENABLE_GRAVITY_ACCELERATION_REMOVE "Enable Gravity acceleration remove" OFF)
if(ENABLE_GRAVITY_ACCELERATION_REMOVE)
  add_definitions(-DENABLE_GRAVITY_ACCELERATION_REMOVE)
endif()

#========================
# Project details / setup
#========================
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)
if(${POINT_TYPE} STREQUAL "XYZIRTT")
  add_definitions(-DPOINT_TYPE_XYZIRTT)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
elseif(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
endif()

#========================
# Others deps
#========================
find_package(yaml-cpp REQUIRED)

#========================
# Include directory (공통)
#========================
include_directories(${PROJECT_SOURCE_DIR}/src)

#========================
# Driver core
#========================
add_subdirectory(src/vanjee_driver)
find_package(vanjee_driver REQUIRED)
include_directories(${vanjee_driver_INCLUDE_DIRS})

#========================
# ROS1 (CATKIN) 경로
#========================
if(roscpp_FOUND AND COMPILE_METHOD STREQUAL "CATKIN")
  message(=============================================================)
  message("-- ROS Found. ROS1 + CATKIN build is On.")
  message(=============================================================)

  add_definitions(-DROS_FOUND -DRUN_IN_ROS_WORKSPACE)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    roslib
    std_msgs
    message_generation
  )

  ## 메시지 파일
  add_message_files(
    FILES
      DeviceCtrl.msg
      VanjeelidarPacket.msg
  )

  ## 메시지 생성
  generate_messages(
    DEPENDENCIES
      std_msgs
  )

  ## catkin 패키지 Export
  catkin_package(
    CATKIN_DEPENDS
      roscpp
      sensor_msgs
      roslib
      std_msgs
      message_runtime         # <<< runtime 의존성 반드시 포함
    INCLUDE_DIRS
      include
  )

  ## 올바른 include 경로 (⚠️ 뒤에 /ros_msg 붙이지 말 것)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

endif()

#========================
# ROS2 (COLCON) 경로
#========================
if(rclcpp_FOUND AND COMPILE_METHOD STREQUAL "COLCON")
  message(=============================================================)
  message("-- ROS2 Found. ROS2 + COLCON build is On.")
  message(=============================================================)

  add_definitions(-DROS2_FOUND)
  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(vanjee_lidar_msg REQUIRED)
  find_package(std_msgs REQUIRED)

  include_directories(
    include
  )
endif()

#========================
# Build Targets
#========================
add_executable(vanjee_lidar_sdk_node
  node/vanjee_lidar_sdk_node.cpp
  src/manager/node_manager.cpp
)

# --- 링크 공통 ---
target_link_libraries(vanjee_lidar_sdk_node
  ${YAML_CPP_LIBRARIES}
  ${vanjee_driver_LIBRARIES}
)
if(ENABLE_TBB)
  target_link_libraries(vanjee_lidar_sdk_node TBB::tbb)
endif()

# --- ROS1 전용: 링크/의존성/설치 ---
if(roscpp_FOUND AND COMPILE_METHOD STREQUAL "CATKIN")
  target_link_libraries(vanjee_lidar_sdk_node ${catkin_LIBRARIES})

  # 메시지 생성 타겟과 순서 보장 (핵심)
  add_dependencies(vanjee_lidar_sdk_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  install(TARGETS vanjee_lidar_sdk_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  install(DIRECTORY param/  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param)
  install(DIRECTORY rviz/   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
endif()

# --- ROS2 전용: 링크/의존성/설치 ---
if(rclcpp_FOUND AND COMPILE_METHOD STREQUAL "COLCON")
  ament_target_dependencies(vanjee_lidar_sdk_node
    rclcpp
    std_msgs
    sensor_msgs
    vanjee_lidar_msg
  )

  install(TARGETS
    vanjee_lidar_sdk_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch
    rviz
    config
    param
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()
endif()
