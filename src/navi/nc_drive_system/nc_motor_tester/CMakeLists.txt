cmake_minimum_required(VERSION 3.0.2)
project(nc_motor_tester)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  core_msgs
  geometry_msgs
  nav_msgs
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nc_motor_tester
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)
find_package(Poco COMPONENTS Foundation Util Net JSON Redis Data)

set(CMAKE_MODULE_PATH 
		${${CMAKE_MODULE_PATH}})
#JSONCPP setting
# pkg_check_modules(JSONCPP jsoncpp)
# link_libraries(${JSONCPP_LIBRARIES})

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
${JSONCPP_INCLUDE_DIRS}
)

add_executable(nc_motor_semical_tester src/nc_motor_semical_tester.cpp)
add_executable(nc_motor_tuning_tester src/nc_motor_tuning_tester.cpp)
target_link_libraries(nc_motor_semical_tester 
  ${CORE_LIBRARY} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  Boost::log_setup 
  Boost::log
  ${JSONCPP_LIBRARIES}
  ${Poco_LIBRARIES}
)

target_link_libraries(nc_motor_tuning_tester 
  ${CORE_LIBRARY} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  Boost::log_setup 
  Boost::log
  ${JSONCPP_LIBRARIES}
  ${Poco_LIBRARIES}
)
    
add_dependencies(nc_motor_semical_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(nc_motor_tuning_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

# include_directories(/usr/local/include)
# link_directories(/usr/local/lib)

install(TARGETS
  nc_motor_semical_tester
  nc_motor_tuning_tester  # ← 추가
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT ${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
