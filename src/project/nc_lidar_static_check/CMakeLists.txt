cmake_minimum_required(VERSION 3.0.2)
project(nc_lidar_static_check)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  core_msgs
  pcl_conversions
  pcl_ros
)

find_package(PCL REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs core_msgs
)

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)

link_directories(
  ${build_navicore_SOURCE_DIR}/lib
  ${CMAKE_SOURCE_DIR}/lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/lidar_static_check.cpp
  src/lidar_static_check_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${CORE_LIBRARY}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  Boost::log_setup
  Boost::log
)
install( TARGETS
${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
