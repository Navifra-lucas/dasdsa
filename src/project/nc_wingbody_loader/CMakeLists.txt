cmake_minimum_required(VERSION 3.0.2)
project(nc_wingbody_loader)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs pcl_ros pcl_conversions
  DEPENDS PCL
)

# External navicore library setup
link_directories(${build_navicore_SOURCE_DIR}/lib ${CMAKE_SOURCE_DIR}/lib)
file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
)

add_executable(${PROJECT_NAME}_node
  src/wingbody_loader_node.cpp
  src/transform_manager.cpp
  src/plane_detector.cpp
  src/cargo_classifier.cpp
  src/pose_calculator.cpp
  src/visualization_manager.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${CORE_LIBRARY}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  Boost::log_setup
  Boost::log
)

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
