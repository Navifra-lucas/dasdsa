cmake_minimum_required(VERSION 3.0.2)
project(nc_dcs)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  core_msgs
  roslib
  message_generation
  dcs_msgs
  move_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs core_msgs roslib message_runtime tf2 tf2_ros tf2_geometry_msgs nav_msgs
  DEPENDS yaml-cpp
)

# --- [수정 1] Boost 설정 강화 ---
set(Boost_USE_STATIC_LIBS ON)      # 에러 로그의 v2s(Static)와 일치
set(Boost_USE_MULTITHREADED ON)    # 에러 로그의 mt(Multi-Thread)와 일치
set(Boost_USE_SYSTEM_RUNTIME OFF)

# log, log_setup 외에 의존성 컴포넌트를 명시적으로 모두 적어야 합니다.
find_package(Boost REQUIRED COMPONENTS 
    log 
    log_setup 
    thread 
    filesystem 
    date_time 
    system 
    chrono 
    regex
)

message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  # [수정 2] Boost 헤더 경로 명시 추가
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
)

add_executable(docking_control_node 
  src/docking_control_node.cpp
  src/sequence_loader.cpp
  src/fork_parameters_loader.cpp
  src/state_handler.cpp
  src/ros_interface.cpp
  src/action_executor.cpp
  src/error_checker.cpp
)

add_dependencies(docking_control_node 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# --- [수정 3] 링크 순서 및 pthread 추가 ---
target_link_libraries(docking_control_node 
    ${CORE_LIBRARY}       # libcore.a가 Boost를 쓰므로 가장 먼저 와야 함
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Boost_LIBRARIES}    # 위에서 찾은 모든 Boost 컴포넌트(.a 파일들)
    pthread               # [중요] 정적 링크 시 Linux 쓰레드 라이브러리 필수
)

# Install config files

catkin_install_python(PROGRAMS
  scripts/siyeon.py
  scripts/siyeon_0202.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install executables
install(TARGETS docking_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
        USE_SOURCE_PERMISSIONS)