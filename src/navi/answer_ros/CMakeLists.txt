cmake_minimum_required(VERSION 3.16)
project(answer)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_definitions(-DANSWER_ROS1)
add_definitions(-DANSWER_NAVICORE)
add_definitions(-DBUILD_WITH_VISUALIZER)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(Ceres REQUIRED)
find_package(Poco COMPONENTS Foundation Util JSON Data)
find_package(OpenGL REQUIRED)
find_package(glm)
find_package(glfw3)
find_package(PNG)
find_package(JPEG)
find_package(assimp QUIET)
find_package(spdlog REQUIRED)


set(2D_LOCALIZER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/answer/2d_localizer)
set(2D_LOCALIZER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/answer/2d_localizer)

set(3D_LOCALIZER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/answer/3d_localizer)
set(3D_LOCALIZER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/answer/3d_localizer)

set(2D_SLAM_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/answer/2d_slam)
set(2D_SLAM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/answer/2d_slam)

set(IMU_POSE_ESTIMATOR_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/answer/imu_pose_estimator)
set(IMU_POSE_ESTIMATOR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/answer/imu_pose_estimator)

set(COMMON_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/answer/common)
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/answer/common)

set(LOGGER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/answer/logger)
set(LOGGER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/answer/logger)

set(VISUALIZER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/answer/visualizer)
set(VISUALIZER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/answer/visualizer)

# 3d_perception is now built from source via add_subdirectory
set(3D_PERCEPTION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/answer/3d_perception)

set(BLAZE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/navi/thirdparty/blaze/include)
set(OSQP_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/navi/thirdparty/osqp/lib)
set(OSQP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/navi/thirdparty/osqp/include/osqp)

set(NAVI_UTILS_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/navi/utils)
set(NAVI_UTILS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/navi/utils)

set(NAVI_PATH_PLANNER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/navi/path_planner)
set(NAVI_PATH_PLANNER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/navi/path_planner)

set(NAVI_MOTION_CONTROLLER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/navi/motion_controller)
set(NAVI_MOTION_CONTROLLER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/navi/motion_controller)

set(NAVI_INTERFACE_COMMON_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/answer_libs/navi/interface_common)
set(NAVI_INTERFACE_COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/navi/interface_common)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_msgs
  tf2_geometry_msgs
  message_filters
  laser_geometry
  roslib
  roslaunch
  std_srvs
  answer_msgs
  message_generation
  rostest
)

add_service_files(
  DIRECTORY src/ros1/srv
  FILES
  common_service.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  answer_msgs
)

# Include directories

set(ANSWER_SRCS
  src/ros1/answer_node_ros1.cpp
  src/ros1/answer_ros1.cpp
  # src/ros1/answer_3d_ros1.cpp
  src/ros1/callbacks/get_lidar_2d.cpp
  src/ros1/callbacks/get_lidar_3d.cpp
  src/ros1/callbacks/get_laser_scan_2d.cpp
  src/ros1/callbacks/get_odometry_2d.cpp
  src/ros1/callbacks/get_initial_pose.cpp
  src/ros1/callbacks/get_external_pose.cpp
  src/ros1/callbacks/get_clear_alarm.cpp
  src/ros1/callbacks/get_correction_trigger.cpp
  src/ros1/callbacks/get_map_save.cpp
  src/ros1/callbacks/get_imu.cpp
  src/ros1/callbacks/get_register_reflector.cpp
)

# Build 3d_perception from source
add_subdirectory(answer_libs/answer/3d_perception)

link_directories(
    ${2D_LOCALIZER_LIB_DIR}
    ${IMU_POSE_ESTIMATOR_LIB_DIR}
    ${3D_LOCALIZER_LIB_DIR}
    ${2D_SLAM_LIB_DIR}
    ${COMMON_LIB_DIR}
    ${LOGGER_LIB_DIR}
    ${VISUALIZER_LIB_DIR}
    ${OSQP_LIB_DIR}
    ${NAVI_UTILS_LIB_DIR}
    ${NAVI_PATH_PLANNER_LIB_DIR}
    ${NAVI_MOTION_CONTROLLER_LIB_DIR}
    ${NAVI_INTERFACE_COMMON_LIB_DIR}
)

set(ANSWER_LIBS
    ${catkin_LIBRARIES}
     -Wl,--start-group
    2d_localizer
    3d_localizer
    3d_perception
    imu_pose_estimator
    2d_slam
    common
    logger
    visualizer
     -Wl,--end-group
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    -ltbb
    -latomic
    ${OpenMP_CXX_FLAGS}
    spdlog::spdlog
    -lfmt
    OpenGL::GL
    glfw
    ${Poco_LIBRARIES}
    # assimp
    PNG::PNG
    JPEG::JPEG
    ceres
)


set(ANSWER_INCLUDE
    src/ros1/include
    ${catkin_INCLUDE_DIRS}
    PRIVATE
    ${2D_LOCALIZER_INCLUDE_DIR}
    ${IMU_POSE_ESTIMATOR_INCLUDE_DIR}
    ${3D_LOCALIZER_INCLUDE_DIR}
    ${2D_SLAM_INCLUDE_DIR}
    ${3D_PERCEPTION_INCLUDE_DIR}
    ${COMMON_INCLUDE_DIR}
    ${LOGGER_INCLUDE_DIR}
    ${VISUALIZER_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${Poco_INCLUDE_DIRS}   
)

set(NAVI_COMMON_SRC
    src/ros1/navi_common/msg_converter.cpp
    src/ros1/navi_common/visualizer.cpp
)
set(NAVI_COMMON_LIBRARIES
    ${catkin_LIBRARIES}
    ${Poco_LIBRARIES}
    ${PCL_LIBRARIES}
     -Wl,--start-group
    spdlog::spdlog
    -lfmt
    common
    logger
    utils
    osqp
    interface_common
    path_planner
    motion_controller
     -Wl,--end-group
)
set(NAVI_COMMON_INCLUDE
    src/ros1/include
    ${catkin_INCLUDE_DIRS}
    PRIVATE
    ${Poco_INCLUDE_DIRS}
    ${COMMON_INCLUDE_DIR}
    ${LOGGER_INCLUDE_DIR}
    ${BLAZE_INCLUDE_DIR}
    ${OSQP_INCLUDE_DIR}
    ${NAVI_UTILS_INCLUDE_DIR}
    ${NAVI_PATH_PLANNER_INCLUDE_DIR}
    ${NAVI_MOTION_CONTROLLER_INCLUDE_DIR}
    ${NAVI_INTERFACE_COMMON_INCLUDE_DIR}
)


# Add executables

add_executable(${PROJECT_NAME}_node ${ANSWER_SRCS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
    ${ANSWER_LIBS}
)

add_executable(perception_node 
  src/ros1/perception/perception_node_ros1.cpp
  src/ros1/perception/perception_ros1.cpp
)
add_dependencies(perception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(perception_node
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
target_link_libraries(perception_node
    PRIVATE
    ${ANSWER_LIBS}
)

add_executable(answer_core_bridge
    src/ros1/answer_core_bridge.cpp
    src/ros1/node.cpp
)
add_dependencies(answer_core_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(answer_core_bridge
    PUBLIC
    include
    include/answer
    ${catkin_INCLUDE_DIRS}
)
    # Link libraries
target_link_libraries(answer_core_bridge
    ${catkin_LIBRARIES}
    ${Poco_LIBRARIES}
)

add_executable(slam_start 
    src/ros1/slam_start.cpp    
)

target_include_directories(slam_start
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(slam_start ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_start
  ${ANSWER_LIBS}
)

add_executable(slam_terminate
    src/ros1/slam_terminate.cpp    
)

target_include_directories(slam_terminate
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(slam_terminate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_terminate
  ${ANSWER_LIBS}
)

add_executable(localization_start 
    src/ros1/localization_start.cpp    
)

target_include_directories(localization_start
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(localization_start ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_start
  ${ANSWER_LIBS}
)

add_executable(localization_terminate
    src/ros1/localization_terminate.cpp    
)

target_include_directories(localization_terminate
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(localization_terminate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_terminate
  ${ANSWER_LIBS}
)

add_executable(record_data
    src/ros1/record_data.cpp    
)

target_include_directories(record_data
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(record_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_data
  ${ANSWER_LIBS}
)

add_executable(log_level
    src/ros1/log_level.cpp    
)

target_include_directories(log_level
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(log_level ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(log_level
  ${ANSWER_LIBS}
)

add_executable(update_param
    src/ros1/update_param.cpp    
)

target_include_directories(update_param
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(update_param ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(update_param
  ${ANSWER_LIBS}
)

add_executable(edit_param
    src/ros1/edit_param.cpp    
)

target_include_directories(edit_param
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(edit_param ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(edit_param
  ${ANSWER_LIBS}
)

add_executable(sampling_match
    src/ros1/sampling_match.cpp
)

target_include_directories(sampling_match
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(sampling_match ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sampling_match
  ${ANSWER_LIBS}
)

add_executable(pallet_template_capture
    src/ros1/pallet_template_capture.cpp
)
target_include_directories(pallet_template_capture
    PUBLIC
    include
    ${ANSWER_INCLUDE}
)
add_dependencies(pallet_template_capture ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pallet_template_capture
  ${ANSWER_LIBS}
)

add_executable(docking_path_generator_node
  ${NAVI_COMMON_SRC}
  src/ros1/docking_path_generator/docking_path_generator_node.cpp
  src/ros1/docking_path_generator/docking_path_generator_ros1.cpp
  src/ros1/docking_path_generator/dpg_param_manager.cpp
  src/ros1/docking_path_generator/docking_info.cpp
  src/ros1/docking_path_generator/smart_path_generator.cpp
  src/ros1/docking_path_generator/virtual_mission.cpp
  src/ros1/docking_path_generator/pallet_mission.cpp
  src/ros1/docking_path_generator/wingbody_mission.cpp
  src/ros1/docking_path_generator/go_back_mission.cpp
  src/ros1/docking_path_generator/callbacks/get_robot_pose.cpp
  src/ros1/docking_path_generator/callbacks/get_odom.cpp
  src/ros1/docking_path_generator/callbacks/get_cmd_signal.cpp
  src/ros1/docking_path_generator/callbacks/get_cmd_docking.cpp
  src/ros1/docking_path_generator/callbacks/get_object_pose.cpp
)
target_include_directories(docking_path_generator_node
    PUBLIC
    include
    ${NAVI_COMMON_INCLUDE}
)
target_link_libraries(docking_path_generator_node
  ${NAVI_COMMON_LIBRARIES}
)
add_dependencies(docking_path_generator_node
  ${docking_path_generator_node_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)



# Install targets
install(TARGETS
    answer_core_bridge
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    perception_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  slam_start
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  slam_terminate
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  localization_start
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  localization_terminate
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  record_data
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
   log_level 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    update_param
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  edit_param
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  sampling_match
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pallet_template_capture
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  docking_path_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

#Install config files
install(DIRECTORY include/${PROJECT_NAME} 
    DESTINATION include/
)

#Install config files
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

#Install config files
install(DIRECTORY template
    DESTINATION share/${PROJECT_NAME}
)

#Install scripts files
install(DIRECTORY scripts 
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/answer/visualizer/data
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/answer/visualizer/data
    DESTINATION $ENV{HOME}//navifra_solution/navicore/configs/ 
)



##Install config files
#install(DIRECTORY config
    #DESTINATION lib/${PROJECT_NAME}
#)

# Install launch files
# install(DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
# )
