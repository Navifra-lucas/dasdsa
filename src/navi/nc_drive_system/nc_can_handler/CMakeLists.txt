cmake_minimum_required(VERSION 3.13)

project(nc_can_handler)

# Add this line to set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


find_package(Threads REQUIRED)

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/libcore.*.a")
set(CORE_DIR ${build_navicore_SOURCE_DIR}/include/core)
if(core_SOURCE_DIR)
  set(CORE_LIBRARY core)
  set(CORE_DIR ${core_INCLUDE_DIRS})
  message(STATUS "FIND core package")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CORE_DIR}
    ${CORE_DIR}/types
)

# Sources
file(GLOB_RECURSE SRCS_MASTER CONFIGURE_DEPENDS 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/**/*.cpp
)

message(STATUS "NaviCAN sources: ${SRCS_MASTER}")

add_library(${PROJECT_NAME} STATIC ${SRCS_MASTER})

target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
    ${CORE_LIBRARY} 
    ${catkin_LIBRARIES}
    Threads::Threads
)

# 헤더 파일
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 라이브러리
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
