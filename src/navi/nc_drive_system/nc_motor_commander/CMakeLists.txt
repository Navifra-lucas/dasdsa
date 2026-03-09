cmake_minimum_required(VERSION 3.0.2)
project(nc_motor_commander)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)

option(LICENSE "Enable a LICENSE" ON)
if(LICENSE)
  message("The LICENSE is enabled.")
  add_definitions(-DNAVIFRA_LICENSE_ON)
else()
  message("The LICENSE is disabled.")
endif()

file(STRINGS "${PROJECT_SOURCE_DIR}/VERSION" PACKAGE_VERSION)
string(REGEX REPLACE "([0-9]+)\\.[0-9]+\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.([0-9]+)\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION})

set(PROJECT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})

message("${PROJECT_SOURCE_DIR}/../.git")
find_package(Git)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/../.git")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        OUTPUT_VARIABLE DRIVER_SOURCE_GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
else()
    set(DRIVER_SOURCE_GIT_COMMIT_HASH "Unknown")
endif()
# configure_file(
#     "${PROJECT_SOURCE_DIR}/driver_config.h.in"
#     "${PROJECT_SOURCE_DIR}/include/driver_config.h"
# )

option(LICENSE "Enable a LICENSE" ON)
if(LICENSE)
  message("The LICENSE is enabled.")
  add_definitions(-DNAVIFRA_LICENSE_ON)
else()
  message("The LICENSE is disabled.")
endif()
# core setting

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  core_msgs
  motor_msgs    
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  serial
)
find_package(ZLIB)
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
  CATKIN_DEPENDS roscpp rospy std_msgs
)
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations

message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장


file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")

set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)
message(STATUS "CORE_DIRcheck : ${NAVICORE_DIR}")


include_directories(src)
include_directories(include)
include_directories(
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${NAVICORE_DIR}/core_motion
    
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR} 
  ${PROJECT_SOURCE_DIR}/include
  ${ZLIB_INCLUDE_DIRS}
  ${exe_core_INCLUDE_DIRS}
)

# 라이브러리화 cpp 파일 추가  
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
# add_library(${PROJECT_NAME} STATIC ${SRC_FILES})  #STATIC/SHARED
add_executable(${PROJECT_NAME} ${SRC_FILES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
file(COPY include DESTINATION ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# set(BUILD_PCAN OFF)
option(BUILD_PCAN "Disable a BUILD_PCAN" OFF)

message(STATUS "BUILD_PCANCHECK " ${BUILD_PCAN})
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


if (EXISTS /usr/local/lib/libBISS_1SL_interface.so)
  add_definitions(-DNAVIFRA_BISS_SSI_FOUND)
  target_link_libraries(${PROJECT_NAME} /usr/local/lib/libBISS_1SL_interface.so)
endif()

target_link_libraries(${PROJECT_NAME} ${CORE_LIBRARY} ${catkin_LIBRARIES} -lssl -lcrypto ${ZLIB_LIBRARIES} Boost::log_setup Boost::log)
install( TARGETS
  ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if(BUILD_PCAN)
  # PCAN 라이브러리 파일 찾기
  file(GLOB PCAN_LIBRARY "${build_navicore_SOURCE_DIR}/libpcanbasic/lib/libpcanbasic.so.*")

  # 파일확인
  if(EXISTS "${PCAN_LIBRARY}")
      message(STATUS "Found PCAN library: ${PCAN_LIBRARY}")

      # 선택된 PCAN 라이브러리 설치
      install(FILES ${PCAN_LIBRARY}
          DESTINATION ${CMAKE_INSTALL_LIBDIR}
          RENAME libpcanbasic.so  # 원하는 이름으로 변경
      )
  else()
      message(WARNING "No PCAN library found!")
  endif()

  # PCAN 헤더 파일 설치 (libpcanbasic 디렉토리 안에 있는 경우)
  install(DIRECTORY ${build_navicore_SOURCE_DIR}/libpcanbasic/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libpcanbasic
      FILES_MATCHING PATTERN "*.h"
  )
endif() 