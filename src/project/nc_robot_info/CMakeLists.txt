cmake_minimum_required(VERSION 3.0.2)
project(nc_robot_info)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  core_msgs
  rosgraph_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nc_robot_info
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)

message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장


file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${robot_driver_INCLUDE_DIRS}
  ${NAVICORE_DIR}
${NAVICORE_DIR}/core
${NAVICORE_DIR}/core/types
  
)

add_executable(${PROJECT_NAME} src/nc_robot_info.cpp)
target_link_libraries(${PROJECT_NAME} 
  ${CORE_LIBRARY} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  Boost::log_setup 
  Boost::log)
    
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

# include_directories(/usr/local/include)
# link_directories(/usr/local/lib)

install( TARGETS ${PROJECT_NAME} 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT ${PROJECT_NAME} 
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)