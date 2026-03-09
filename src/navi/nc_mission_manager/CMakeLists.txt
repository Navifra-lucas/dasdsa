cmake_minimum_required(VERSION 2.8.3)
project(nc_mission_manager)

add_compile_options(-std=c++17)

file(STRINGS "${PROJECT_SOURCE_DIR}/VERSION" PACKAGE_VERSION)
string(REGEX REPLACE "([0-9]+)\\.[0-9]+\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.([0-9]+)\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION})

set(PROJECT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})

message("${PROJECT_SOURCE_DIR}/../.git")
find_package(Git)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/../.git")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE SOURCE_GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
else()
    set(SOURCE_GIT_COMMIT_HASH "Unknown")
endif()
configure_file(
    "${PROJECT_SOURCE_DIR}/mission_config.h.in"
    "${PROJECT_SOURCE_DIR}/include/mission_config.h"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  core_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
  LIBRARIES 
  CATKIN_DEPENDS  core_msgs 
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# find_package(Boost REQUIRED COMPONENTS log log_setup)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS}) 
  set(OpenMP_LIBS gomp)
endif()
###########
## Build ##
###########

message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장


file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)


message(STATUS "natenc_path_planner_SOURCE_DIR : ${nc_path_planner_SOURCE_DIR}")

set(PLANNER_DIR ${nc_path_planner_SOURCE_DIR})
message(STATUS "FIND nc_path_planner_SOURCE_DIR")
find_package(map_based_path_planner REQUIRED)
include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${NAVICORE_DIR}/core_path
  ${NAVICORE_DIR}/core_motion
  ${NAVICORE_DIR}/core_motion/mpc
  ${NAVICORE_DIR}/core_motion/motion_manager
  ${NAVICORE_DIR}/core_motion/mpc_motion_controller
  ${PLANNER_DIR} 
  ${exe_core_INCLUDE_DIRS}
  ${nc_path_planner_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
             
# 라이브러리화 cpp 파일 추가  
file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_library(${PROJECT_NAME} STATIC ${SRC_FILES}) #STATIC/SHARED

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


set_target_properties(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}  
  ${CORE_LIBRARY}
  glog
  map_based_path_planner 
  ${OpenCV_LIBRARIES} 
  # ${Boost_LIBRARIES} 
  # Boost::log_setup Boost::log
  ${OpenMP_LIBS}

)

# for mpc Module
target_link_libraries(${PROJECT_NAME} ipopt)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}    
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)