cmake_minimum_required(VERSION 3.0.2)
project(fork_docking_repeater)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  core_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/fork_repeater.py
  scripts/fork_up_down_repeater.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
        USE_SOURCE_PERMISSIONS)
