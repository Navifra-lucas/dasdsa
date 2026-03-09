cmake_minimum_required(VERSION 3.0.2)
project(nc_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

#OpenMP setting
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS}) 
  set(OpenMP_LIBS gomp)
endif()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  core_msgs
  tf2_ros
)


catkin_package(
  # INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
#  INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy std_msgs core_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장


file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)
find_package(TBB REQUIRED)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${exe_core_SOURCE_DIR}
  ${exe_core_SOURCE_DIR}/types 
)

add_executable(nc_simulator
  src/virtual_lidar_driver.cpp
  src/distance_ray_calculator.cpp  
)

set_target_properties(${PROJECT_NAME}
PROPERTIES
    COMPILE_FLAGS "${OpenMP_CXX_FLAGS}"
)

add_dependencies(nc_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nc_simulator 
  ${CORE_LIBRARY} 
  ${catkin_LIBRARIES} 
  ${OpenMP_CXX_LIBRARIES}  
  ${Boost_LIBRARIES} 
  Boost::log_setup 
  Boost::log 
  # -ltbb 
  TBB::tbb
  -latomic
  )

  set(SIM_BIN_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
  set(SIM_EXEC ${SIM_BIN_DIR}/nc_simulator)
  
  add_custom_command(TARGET nc_simulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${SIM_EXEC} ${SIM_BIN_DIR}/nc_simulator_front
    COMMAND ${CMAKE_COMMAND} -E copy ${SIM_EXEC} ${SIM_BIN_DIR}/nc_simulator_rear
    COMMAND ${CMAKE_COMMAND} -E copy ${SIM_EXEC} ${SIM_BIN_DIR}/nc_simulator_left
    COMMAND ${CMAKE_COMMAND} -E copy ${SIM_EXEC} ${SIM_BIN_DIR}/nc_simulator_right
  )

install( TARGETS
  nc_simulator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(CODE "
  execute_process(COMMAND \${CMAKE_COMMAND} -E copy
    \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nc_simulator\"
    \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nc_simulator_front\")
  execute_process(COMMAND \${CMAKE_COMMAND} -E copy
    \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nc_simulator\"
    \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nc_simulator_rear\")
  execute_process(COMMAND \${CMAKE_COMMAND} -E copy
    \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nc_simulator\"
    \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nc_simulator_left\")
  execute_process(COMMAND \${CMAKE_COMMAND} -E copy
    \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nc_simulator\"
    \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/nc_simulator_right\")
")
# install(DIRECTORY scripts/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
#         USE_SOURCE_PERMISSIONS)