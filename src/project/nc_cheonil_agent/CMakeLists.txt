cmake_minimum_required(VERSION 3.0.2)
project(nc_cheonil_agent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(STRINGS "${PROJECT_SOURCE_DIR}/VERSION" PACKAGE_VERSION)
string(REGEX REPLACE "([0-9]+)\\.[0-9]+\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.([0-9]+)\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION})

set(PROJECT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})

# Append our module directory to CMake
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(NcCoreMacro)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup serialization)

find_package(Poco REQUIRED COMPONENTS Foundation Util Net JSON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  core_msgs
  core_agent
)

find_package(yaml-cpp REQUIRED)

catkin_package(
   CATKIN_DEPENDS roscpp core_msgs core_agent
)

message(STATUS "catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")

###########
## Build ##
###########

# 컴파일러 옵션 설정
if (CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wno-narrowing)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wno-narrowing)
endif()

# JSONCPP 설정 (필요 시 주석 해제)
# pkg_check_modules(JSONCPP jsoncpp)
# link_libraries(${JSONCPP_LIBRARIES})

message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(
  ${build_navicore_SOURCE_DIR}/lib
  ${CMAKE_SOURCE_DIR}/lib
)

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
  ${Boost_INCLUDE_DIRS}
)

execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/nc_cheonil_agent/version.h @ONLY)

# Sources
file(GLOB SRCS_G
  "src/*.cpp"
  "src/data/*.cpp"
  "src/Modbus/*.cpp"
  "src/Modbus/RTU/*.cpp"
  "src/Modbus/TCP/*.cpp"
  "src/Serial/*.cpp"
  "src/emul/*.cpp"
  "src/manager/*.cpp"
  "src/state/*.cpp"
)

NAVIFRA_SOURCES_AUTO(SRCS ${SRCS_G})

add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Poco_LIBRARIES}
  ${CORE_LIBRARY}
  Boost::log_setup
  Boost::log
  yaml-cpp
)

# Tests
file(GLOB SRCS_TEST_G "tests/*.cpp")
NAVIFRA_SOURCES_AUTO(SRCS_TEST ${SRCS_TEST_G})

# 테스트 실행 파일 주석 처리된 상태 (필요 시 활성화)
# add_executable(${PROJECT_NAME}_unittest ${SRCS_TEST})
# target_link_libraries(${PROJECT_NAME}_unittest ${catkin_LIBRARIES} ${Poco_LIBRARIES} ${CORE_LIBRARY} gtest_main Boost::log_setup Boost::log)
# add_test(NAME ${PROJECT_NAME}_unittest COMMAND ${PROJECT_NAME}_unittest)
# enable_testing()

# Install targets
install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
    USE_SOURCE_PERMISSIONS
)
