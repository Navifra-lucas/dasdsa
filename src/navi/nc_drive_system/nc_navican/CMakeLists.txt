cmake_minimum_required(VERSION 3.13)

project(navican)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(CanopenMacro)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# Add this line to set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Find Boost libraries
# find_package(Boost REQUIRED COMPONENTS log log_setup)

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/libcore.*.a")
set(CORE_DIR ${build_navicore_SOURCE_DIR}/include/core)
if(core_SOURCE_DIR)
  set(CORE_LIBRARY core)
  set(CORE_DIR ${core_INCLUDE_DIRS})
  message(STATUS "FIND core package")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navican
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CORE_DIR}
    ${CORE_DIR}/types
)

# Sources
file(GLOB_RECURSE SRCS_MASTER_G CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
message(STATUS "NaviCAN sources: ${SRCS_MASTER_G}")
NAVIFRA_SOURCES_AUTO(SRCS_MASTER ${SRCS_MASTER_G})

add_library(navican STATIC ${SRCS_MASTER})
target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/NaviCan>
)
target_link_libraries(navican 
    lely-coapp 
    lely-io2 
    lely-ev 
    lely-co 
    lely-can 
    lely-util 
    lely-libc 
    rt 
    yaml-cpp 
    ${CORE_LIBRARY} 
    # Boost::log 
    # Boost::log_setup
)

# Add example directory
add_subdirectory(example)

install(
DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
DIRECTORY config/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
