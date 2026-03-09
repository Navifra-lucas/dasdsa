cmake_minimum_required(VERSION 3.0.2)
project(nc_lidar_merger)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

option(LICENSE "Enable a LICENSE" ON)
if(LICENSE)
  message("The LICENSE is enabled.")
  add_definitions(-DNAVIFRA_LICENSE_ON)
else()
  message("The LICENSE is disabled.")
endif()

#OpenCV setting
find_package(OpenCV REQUIRED)
find_package(TBB REQUIRED)

#OpenMP setting
# find_package(OpenMP REQUIRED)
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
  laser_geometry
  pcl_ros
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
# find_package(thirdparty_core)
# set(thirdparty_DIR ${thirdparty_core_INCLUDE_DIRS})

message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장


file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  
  # ${thirdparty_DIR}
  ${build_navicore_SOURCE_DIR}/include
  ${exe_core_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/nc_lidar_merger.cpp
)
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${core_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# file(GLOB TBB_LIBRARY "${build_navicore_SOURCE_DIR}/lib/libtbb.so")


set_target_properties(${PROJECT_NAME}
PROPERTIES
    COMPILE_FLAGS "${OpenMP_CXX_FLAGS}"
)
target_link_libraries( ${PROJECT_NAME} 
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${OpenMP_LIBS}
    ${OpenCV_LIBS}
    ${TBB_LIBRARY}
    -lssl -lcrypto
    Boost::log_setup Boost::log
    # -ltbb 
    TBB::tbb
    -latomic
)

add_executable(nc_lidar_info_publisher
  src/nc_lidar_info_publisher.cpp
)

target_link_libraries(nc_lidar_info_publisher 
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${OpenMP_LIBS}
    ${OpenCV_LIBS}
    Boost::log_setup Boost::log
    # -ltbb 
    TBB::tbb
    -latomic
)

add_executable(nc_lccs_lidar_merger
  src/nc_lccs_lidar_merger.cpp
)

target_link_libraries(nc_lccs_lidar_merger 
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${OpenMP_LIBS}
    ${OpenCV_LIBS}
    Boost::log_setup Boost::log
    # -ltbb 
    TBB::tbb
    -latomic
)

add_dependencies(nc_lidar_info_publisher ${core_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nc_lccs_lidar_merger ${core_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nc_lidar_merger ${core_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install( TARGETS
${PROJECT_NAME}
nc_lidar_info_publisher
nc_lccs_lidar_merger
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
