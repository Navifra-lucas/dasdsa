cmake_minimum_required(VERSION 2.8.3)
project(nc_playback_cpp)

add_compile_options(-std=c++17)

find_package(Boost REQUIRED COMPONENTS system thread filesystem timer date_time)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  core_msgs
  rosbag
)

catkin_package(
)


message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)


MESSAGE(STATIC "NAVICORE_DIR CORE_DIRCORE_DIRCORE_DIRCORE_DIR ${NAVICORE_DIR}")
include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${exe_core_INCLUDE_DIRS}
)
find_package(Poco COMPONENTS Foundation Util Net JSON Redis Data)

add_executable(${PROJECT_NAME} src/nc_playback_cpp.cpp
                               src/nc_rest_api_client.cpp
                               )
                               
target_link_libraries(${PROJECT_NAME}      
                        ${CORE_LIBRARY} 
                        glog
                        ${catkin_LIBRARIES} -L/usr/lib -lssl -lcrypto
                        ${Boost_LIBRARIES} 
                        ${Boost_DATE_TIME_LIBRARY} 
                        Boost::log_setup Boost::log
                        ${Poco_LIBRARIES}
                        )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

install( TARGETS ${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT ${PROJECT_NAME} 
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)