cmake_minimum_required(VERSION 3.1.6)
project(nc_bms_driver)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++17)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup )
find_package(Poco COMPONENTS Foundation Util Net NetSSL Crypto JSON Redis Data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  core_msgs    
  core_agent
)
catkin_package(
  # INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
  # CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)
if(core_SOURCE_DIR)
  set(CORE_LIBRARY core)
  set(NAVICORE_DIR ${core_INCLUDE_DIRS})
  message(STATUS "FIND core package")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${catkin_INCLUDE_DIRS}
  ${Poco_INCLUDE_DIRS}   
  )
include_directories(src/include)

include_directories(
  ${CMAKE_SOURCE_DIR}/navi/nc_drive_system/nc_custom_driver/include
)

add_executable(${PROJECT_NAME}
          src/can_driver_base.cpp
          src/nc_bms_driver.cpp
          src/communicator.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${CORE_LIBRARY}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_DATE_TIME_LIBRARY} 
  Boost::log_setup
  Boost::log
  ${Poco_LIBRARIES}
 )

 add_dependencies(${PROJECT_NAME} ${core_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_dependencies(${PROJECT_NAME} ${CORE_LIBRARY} core_msgs)
# set(BUILD_PCAN OFF)
option(BUILD_PCAN "Disable a BUILD_PCAN" OFF)

message(STATUS "BUILD_PCANCHECK ",${BUILD_PCAN})
if (EXISTS /usr/lib/libpcanbasic.so)
  add_definitions(-DNAVIFRA_PEAK_CAN_USB_FOUND)
  target_link_libraries(${PROJECT_NAME} /usr/lib/libpcanbasic.so )

elseif(BUILD_PCAN)
  message(STATUS "build navicore pcanfind")
  add_definitions(-DNAVIFRA_PEAK_CAN_USB_FOUND)
  include_directories(
    ${build_navicore_SOURCE_DIR}/libpcanbasic
    )
  target_link_libraries(${PROJECT_NAME} ${build_navicore_SOURCE_DIR}/libpcanbasic/lib/libpcanbasic.so )
endif()

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

install( TARGETS
  ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
