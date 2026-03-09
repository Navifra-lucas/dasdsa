cmake_minimum_required(VERSION 2.8.3)
project(nc_navigator)

add_compile_options(-std=c++17)

option(LICENSE "Enable a LICENSE" ON)
if(LICENSE)
  message("The LICENSE is enabled.")
  add_definitions(-DNAVIFRA_LICENSE_ON)
else()
  message("The LICENSE is disabled.")
  endif()


find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system thread filesystem timer date_time)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs
  move_msgs
  pcl_ros
  laser_geometry
)

catkin_package(
)


message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장


file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)


find_package(nc_mission_manager REQUIRED)
set(MISSION_LIBRARY nc_mission_manager)
set(MISSION_DIR ${nc_mission_manager_SOURCE_DIR}/include)

set(PLANNER_DIR ${nc_path_planner_SOURCE_DIR})
set(MOTION_DIR ${nc_motion_controller_SOURCE_DIR})
find_package(map_based_path_planner REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS}) 
  set(OpenMP_LIBS gomp)
endif()
include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${NAVICORE_DIR}/core_path
  ${NAVICORE_DIR}/core_motion
  ${NAVICORE_DIR}/core_motion/mpc
  ${MISSION_DIR}
  ${PLANNER_DIR}
  ${MOTION_DIR}
  ${MOTION_DIR}/include
  ${exe_core_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/nc_navigator.cpp
                               src/camera_controller.cpp
                               src/param_server.cpp
                               src/data_listener.cpp
                               src/cluster.cpp
                               src/ransac.cpp
                               )
                               
target_link_libraries(${PROJECT_NAME}      
                        ${CORE_LIBRARY} 
                        ${MISSION_LIBRARY}
                        glog
                        ${catkin_LIBRARIES} -L/usr/lib -lssl -lcrypto
                        ${PCL_LIBRARIES}
                        ${OpenCV_LIBRARIES} 
                        ${Boost_LIBRARIES} 
                        ${Boost_DATE_TIME_LIBRARY} 
                        Boost::log_setup Boost::log
                        ipopt
                        ${OpenMP_LIBS}
                        map_based_path_planner
                        nc_mission_manager
                        yaml-cpp 
                        )

 target_precompile_headers(${PROJECT_NAME} PRIVATE include/nc_navigator/nc_navigator_pch.h)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

install( TARGETS ${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT ${PROJECT_NAME} 
)

# 날짜 및 Git 해시 추출
execute_process(
    COMMAND date "+%y%m%d"
    OUTPUT_VARIABLE BUILD_DATE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND git rev-parse --short=7 HEAD
    OUTPUT_VARIABLE GIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(VERSION_STRING "${BUILD_DATE}.${GIT_HASH}")
message(STATUS "Build version: ${VERSION_STRING}")

# 파일 생성
set(VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/build_version.txt")
file(WRITE ${VERSION_FILE} "${VERSION_STRING}\n")

# install 시 포함
install(FILES ${VERSION_FILE} DESTINATION share)