cmake_minimum_required(VERSION 3.0.2)
project(core_astra)

file(STRINGS "${PROJECT_SOURCE_DIR}/VERSION" PACKAGE_VERSION)
string(REGEX REPLACE "([0-9]+)\\.[0-9]+\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.([0-9]+)\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION})

set(PROJECT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)
find_package(Poco COMPONENTS Foundation Util Net NetSSL Crypto JSON Redis Data)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs motor_msgs core_agent)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_compile_options(-std=c++17)

# --- Protobuf 추가 ---
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
file(GLOB PROTO_FILES proto/*.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

catkin_package()

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)
message(STATUS "exe_core_INCLUDE_DIRS: ${exe_core_INCLUDE_DIRS}")

include_directories(
  include
  ${Poco_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${YAML_CPP_INCLUDE_DIRS} 
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${exe_core_INCLUDE_DIRS}
)

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/core_astra/version.h @ONLY)

# --- Agent Manager Node (ROS + ZeroMQ) ---
add_executable(${PROJECT_NAME}
    src/agent_manager_server_ros.cpp
    src/zmq_handler.cpp
    src/core_astra_node.cpp
    src/message/astra_message.cpp
    src/initailizer/config_initializer.cpp
    src/initailizer/motor_status_initializer.cpp
    src/initailizer/zmq_initailizer.cpp
    src/initailizer/robot_status_initailizer.cpp
    src/initailizer/robot_pose_initailizer.cpp
    src/initailizer/robot_velocity_initializer.cpp
    src/initailizer/robot_collision_initializer.cpp
    src/initailizer/event_initializer.cpp
    src/initailizer/robot_safety_status_initializer.cpp
    src/action/action_robot_info.cpp
    src/action/action_get_ros_nodes.cpp
    src/action/action_initpose.cpp
    src/action/action_cmd_vel.cpp
    src/action/action_start_scan.cpp
    src/action/action_move.cpp
    src/action/action_bumper_control.cpp
    src/action/action_lccs_control.cpp
    src/action/action_lifter_control.cpp
    src/action/action_start.cpp
    ${PROTO_SRCS} ${PROTO_HDRS}
)


if(CMAKE_BUILD_TYPE MATCHES "Debug")
# Add AddressSanitizer compile options for the target only in Debug build
      target_compile_options(${PROJECT_NAME} PRIVATE -lasan -fsanitize=address)
        # Link AddressSanitizer runtime library
      target_link_libraries(${PROJECT_NAME}
      -fsanitize=address
      ${catkin_LIBRARIES}
      ${ZMQ_LIBRARIES}
      ${CORE_LIBRARY} ${catkin_LIBRARIES}  ${Poco_LIBRARIES} ${YAML_CPP_LIBRARIES} Boost::log_setup Boost::log
      ${Protobuf_LIBRARIES}
      pthread
      )
else()
      target_link_libraries(${PROJECT_NAME}
          ${catkin_LIBRARIES}
          ${ZMQ_LIBRARIES}
          ${CORE_LIBRARY} ${catkin_LIBRARIES}  ${Poco_LIBRARIES} ${YAML_CPP_LIBRARIES} Boost::log_setup Boost::log
          ${Protobuf_LIBRARIES}
          pthread
      )
endif()


target_precompile_headers(${PROJECT_NAME} PRIVATE include/core_astra/core_astra.h)

install( TARGETS
  ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)