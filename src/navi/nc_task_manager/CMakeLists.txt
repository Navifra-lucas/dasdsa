cmake_minimum_required(VERSION 3.16)
project(nc_task_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

# Append our module directory to CMake
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(NcNaviFraMacro)

set(Boost_USE_STATIC_LIBS ON)
find_package(Poco COMPONENTS Foundation Util Net JSON Redis Data)
find_package(Boost REQUIRED COMPONENTS log log_setup)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  core_msgs
  move_msgs
  core_agent
  task_msgs

)

catkin_package(
# INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
# INCLUDE_DIRS include
 LIBRARIES core_msgs
 CATKIN_DEPENDS roscpp rospy std_msgs core_msgs move_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장
find_package(behaviortree_cpp)


file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core_path
  ${NAVICORE_DIR}/core/types
  ${exe_core_INCLUDE_DIRS}
)

# Sources
file(GLOB SRCS_G 
  "src/*.cpp" 
  "src/topic/*.cpp"
  "src/data/*.cpp"
  "src/plugins/action/*.cpp"
  "src/plugins/condition/*.cpp"
  "src/planner/*.cpp"
)

NAVIFRA_SOURCES_AUTO(SRCS ${SRCS_G})

add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} 
${CORE_LIBRARY} 
    core_msgs
    move_msgs
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${Poco_LIBRARIES}
    ${behaviortree_cpp_LIBRARIES}
)

install( TARGETS
  ${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)   
