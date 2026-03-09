cmake_minimum_required(VERSION 3.0.2)
project(nc_navican_remote)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  navican
)

# Add this line to set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup system thread)
# Boost.Beast is header-only but requires Boost.System and Boost.Asio

# RapidJSON is header-only, no need to find_package
find_package(Threads REQUIRED)

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
# set(CORE_DIR ${build_navicore_SOURCE_DIR}/include/core)
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)

set(NAVICAN_LIBRARY navican)

catkin_package(
 INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
 CATKIN_DEPENDS roscpp navican
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NAVICAN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
)

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_executable(${PROJECT_NAME}
    ${SOURCE_FILES}
)

add_dependencies(${PROJECT_NAME} navican)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CORE_LIBRARY}
  ${NAVICAN_LIBRARY}
  ${Boost_LIBRARIES}
  Threads::Threads
)

install(
TARGETS ${PROJECT_NAME}
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)