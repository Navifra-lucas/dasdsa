cmake_minimum_required(VERSION 3.13)

project(nc_navicat)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

#########################################
## SOEM EtherCAT Library build         ##
#########################################

# FetchContent를 사용하여 SOEM 다운로드
include(FetchContent)

FetchContent_Declare(
  soem
  GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM.git
  GIT_TAG        v2.0.0
  SOURCE_DIR     ${CMAKE_CURRENT_BINARY_DIR}/SOEM
)

# SOEM 다운로드
FetchContent_GetProperties(soem)
if(NOT soem_POPULATED)
  message(STATUS "Downloading SOEM library...")
  FetchContent_Populate(soem)
endif()

# SOEM CMakeLists.txt의 cmake_minimum_required() 버전 변경
set(SOEM_CMAKE_FILE ${soem_SOURCE_DIR}/CMakeLists.txt)

if(EXISTS ${SOEM_CMAKE_FILE})
  message(STATUS "cmake_minimum_required() changed for compatibility...")
  
  file(READ ${SOEM_CMAKE_FILE} SOEM_CMAKE_CONTENT)
  
  string(REGEX REPLACE 
    "cmake_minimum_required\\(VERSION [0-9]+\\.[0-9]+[^)]*\\)"
    "cmake_minimum_required(VERSION 3.13)"
    SOEM_CMAKE_CONTENT_FIXED
    "${SOEM_CMAKE_CONTENT}"
  )
  
  file(WRITE ${SOEM_CMAKE_FILE} "${SOEM_CMAKE_CONTENT_FIXED}")
  
endif()

# SOEM 빌드 옵션 설정
set(SOEM_BUILD_SAMPLES OFF CACHE BOOL "Build SOEM samples" FORCE) # ON 시, 더 높은 cmake 버전 필요
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared library" FORCE)
set(PROJECT_IS_TOP_LEVEL OFF CACHE BOOL "Project is top level" FORCE)

# SOEM 빌드
message(STATUS "Adding SOEM as subdirectory...")
add_subdirectory(${soem_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/soem-build)

# SOEM 헤더 디렉토리 설정 (SOEM의 새로운 구조에 맞게)
set(SOEM_INCLUDE_DIRS
  ${soem_SOURCE_DIR}/include
  ${soem_SOURCE_DIR}/osal
  ${soem_SOURCE_DIR}/osal/linux
  ${soem_SOURCE_DIR}/oshw/linux
  ${CMAKE_CURRENT_BINARY_DIR}/soem-build/include
)

message(STATUS "SOEM library configured successfully")
message(STATUS "SOEM Source Dir: ${soem_SOURCE_DIR}")
message(STATUS "SOEM Binary Dir: ${CMAKE_CURRENT_BINARY_DIR}/soem-build")
message(STATUS "SOEM Include Dirs: ${SOEM_INCLUDE_DIRS}")
message(STATUS "SOEM Libraries: ${SOEM_LIBRARIES}")
message(STATUS "SOEM Build Samples: ${BUILD_SOEM_EXAMPLES}")


###########################################
## SOEM EtherCAT Examples build          ##
###########################################

# SOEM 예제
if(BUILD_SOEM_EXAMPLES)
  message(STATUS "Buinding SOEM Examples")

  # slaveinfo 예제 프로그램
  add_executable(soem_slaveinfo 
    ${soem_SOURCE_DIR}/test/linux/slaveinfo/slaveinfo.c
  )
  target_include_directories(soem_slaveinfo PRIVATE ${SOEM_INCLUDE_DIRS})
  target_link_libraries(soem_slaveinfo ${SOEM_LIBRARIES} pthread rt)
  add_dependencies(soem_slaveinfo soem_lib)
  
  # simple_test 예제 프로그램
  add_executable(soem_simple_test
    ${soem_SOURCE_DIR}/test/linux/simple_test/simple_test.c
  )
  target_include_directories(soem_simple_test PRIVATE ${SOEM_INCLUDE_DIRS})
  target_link_libraries(soem_simple_test ${SOEM_LIBRARIES} pthread rt)
  add_dependencies(soem_simple_test soem_lib)
  
  message(STATUS "SOEM examples will be built:")
  message(STATUS "  - soem_slaveinfo")
  message(STATUS "  - soem_simple_test")
endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# catkin_package는 SOEM 디렉토리가 존재하기 전에 실행되므로 SOEM_INCLUDE_DIRS는 제외
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SOEM_INCLUDE_DIRS}
)

## Declare a C++ library (commented out to avoid missing source file errors)
add_library(${PROJECT_NAME}
  src/NaviCATCore.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${SOEM_INCLUDE_DIRS}
    ${soem_I_DIR}/osal/linux
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# SOEM 라이브러리가 빌드되었을 때 의존성 추가
if(TARGET soem_lib)
  add_dependencies(${PROJECT_NAME} soem_lib)
endif()

# Link libraries to the main library
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${SOEM_LIBRARIES}
  pthread
  rt
)

## Declare a C++ executable (commented out to avoid missing source file errors)
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/nc_navicat_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# SOEM 라이브러리가 빌드되었을 때 의존성 추가
# if(TARGET soem_lib)
#   add_dependencies(${PROJECT_NAME}_node soem_lib)
# endif()

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${SOEM_LIBRARIES}
#   pthread
#   rt
# )

# EtherCAT 모터 제어 예제는 제거
# option(BUILD_ETHERCAT_MOTOR_EXAMPLE "Build EtherCAT motor control example" OFF)
# 
# if(BUILD_ETHERCAT_MOTOR_EXAMPLE)
#   add_executable(ethercat_motor_jog 
#     src/examples/ethercat_motor_jog.cpp
#   )
#   target_include_directories(ethercat_motor_jog PRIVATE ${SOEM_INCLUDE_DIRS})
#   target_link_libraries(ethercat_motor_jog 
#     ${catkin_LIBRARIES}
#     ${SOEM_LIBRARIES} 
#     pthread 
#     rt
#   )
#   
#   if(TARGET soem_lib)
#     add_dependencies(ethercat_motor_jog soem_lib)
#   endif()
# endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# SOEM 헤더 파일 인스톨
install(DIRECTORY ${soem_SOURCE_DIR}/soem/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/soem
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${soem_SOURCE_DIR}/osal/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/osal
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${soem_SOURCE_DIR}/oshw/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/oshw
  FILES_MATCHING PATTERN "*.h"
)
# SOEM 라이브러리 인스톨
install(TARGETS ${SOEM_LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# SOEM 예제 인스톨 (빌드한 경우)
if(BUILD_SOEM_EXAMPLES)
  install(TARGETS soem_slaveinfo soem_simple_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nc_navicat.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)