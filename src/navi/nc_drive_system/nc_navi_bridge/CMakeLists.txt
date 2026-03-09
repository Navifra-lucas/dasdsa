cmake_minimum_required(VERSION 3.0.2)
project(nc_navi_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  navican
  motor_msgs
)

# Add this line to set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
# set(CORE_DIR ${build_navicore_SOURCE_DIR}/include/core)
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)
# if(core_SOURCE_DIR)
#   set(CORE_LIBRARY core)
#   set(CORE_DIR ${core_INCLUDE_DIRS})
#   message(STATUS "FIND core package")
# endif()


# Add this line to set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(NAVICAN_LIBRARY navican)

catkin_package(
 INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
 CATKIN_DEPENDS roscpp std_msgs navican motor_msgs
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

add_executable(${PROJECT_NAME} 
  src/NaviCANBridge.cpp
)
add_dependencies(${PROJECT_NAME} navican motor_msgs) 
target_link_libraries(${PROJECT_NAME}  
  ${catkin_LIBRARIES}
  ${CORE_LIBRARY} 
  ${NAVICAN_LIBRARY}
  ${Boost_LIBRARIES}
  ${Boost_DATE_TIME_LIBRARY}

)

# target_link_libraries(${PROJECT_NAME} ${CORE_LIBRARY} ${catkin_LIBRARIES} -lssl -lcrypto ${ZLIB_LIBRARIES} Boost::log_setup Boost::log)
# install(TARGETS ${PROJECT_NAME}    
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install( TARGETS
${PROJECT_NAME} 
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)