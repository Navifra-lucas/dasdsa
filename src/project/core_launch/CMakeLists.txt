cmake_minimum_required(VERSION 2.8.3)
project(core_launch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  launch/batterysim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
