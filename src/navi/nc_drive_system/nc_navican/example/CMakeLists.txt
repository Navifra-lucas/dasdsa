cmake_minimum_required(VERSION 3.0.2)
project(fake_slave)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  navican
)

#catkin_package(
#  INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
#  CATKIN_DEPENDS roscpp std_msgs navican
#)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fake_slave 
  fakeSlave.cpp
  motion_generator.cpp
)
target_link_libraries(fake_slave ${catkin_LIBRARIES})

install( TARGETS
${PROJECT_NAME} 
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

