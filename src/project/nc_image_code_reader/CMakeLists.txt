cmake_minimum_required(VERSION 3.1.6)
project(nc_image_code_reader)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++17)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  core_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)

catkin_package(
  # INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
  # CATKIN_DEPENDS roscpp rospy std_msgs
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
if(core_SOURCE_DIR)
  set(CORE_LIBRARY core)
  set(NAVICORE_DIR ${core_INCLUDE_DIRS})
  message(STATUS "FIND core package")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${catkin_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)
include_directories(src/include)

add_executable(${PROJECT_NAME}
          src/nc_image_code_reader.cpp
          src/Tcp_scanner_port.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${CORE_LIBRARY}
  ${catkin_LIBRARIES}
  ${GST_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_DATE_TIME_LIBRARY} 
  Boost::log_setup
  Boost::log
)

add_dependencies(${PROJECT_NAME} ${core_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} core_msgs)

install( TARGETS
  ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
