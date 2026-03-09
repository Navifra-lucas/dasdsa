cmake_minimum_required(VERSION 3.0.2)
project(core_agent) 

file(STRINGS "${PROJECT_SOURCE_DIR}/VERSION" PACKAGE_VERSION)
string(REGEX REPLACE "([0-9]+)\\.[0-9]+\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.([0-9]+)\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION})

set(PROJECT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")


# Append our module directory to CMake
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(CoreAgentMacro)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  core_msgs
)
find_package(yaml-cpp REQUIRED)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)
find_package(Poco REQUIRED COMPONENTS Foundation Util Net JSON Redis Data)


# Find the Paho MQTT C++ library
find_package(PahoMqttCpp REQUIRED)

file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS Boost core_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${NAVICORE_DIR}
${NAVICORE_DIR}/core
${NAVICORE_DIR}/core/types
${YAML_CPP_INCLUDE_DIRS} 
  ${GTEST_INCLUDE_DIRS}  # 추가된 부분
  tests/            # 모킹 헤더 파일이 포함된 디렉토리
  ${PROJECT_SOURCE_DIR}/../../project/nc_wia_agent/include 
)

# Sources
file(GLOB SRCS_G 
  "src/data/*.cpp"
  "src/action/*.cpp"
  "src/redis/*.cpp"
  "src/core/*.cpp"
  "src/manager/*.cpp"
  "src/mqtt/*.cpp"
  "src/util/*.cpp"
  "src/modbus/*.cpp"
  "src/modbus/port/*.cpp"
  "src/*.cpp"  
)

NAVIFRA_SOURCES_AUTO(SRC ${SRCS_G})

add_library(${PROJECT_NAME} STATIC ${SRC})
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_precompile_headers(${PROJECT_NAME}  PRIVATE include/core_agent/core_agent.h)
#target_compile_definitions(${PROJECT_NAME} PUBLIC "${PROJECT_NAME}:DEBUG=$<CONFIG:Debug>")

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  PahoMqttCpp::paho-mqttpp3
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# Configuration
set(config_install_dir "cmake/")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/cmake")
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")

# Include module with function 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${version_config}" COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
)

# Add an "uninstall" target 
configure_file("${CMAKE_CURRENT_LIST_DIR}/cmake/uninstall_target.cmake.in" 
                "${CMAKE_CURRENT_BINARY_DIR}/cmake/uninstall_target.cmake" IMMEDIATE @ONLY) 
                
# Sources
# file(GLOB SRCS_TEST_G 
#   "src/data/*.cpp"
#   "src/action/*.cpp"
#   "src/redis/*.cpp"
#   "src/core/*.cpp"
#   "src/manager/*.cpp"
#   "src/util/*.cpp"
#   "tests/data/*.cpp"
#   "tests/message/*.cpp"
#   "tests/manager/*.cpp"
# )

# NAVIFRA_SOURCES_AUTO(SRCS_TEST ${SRCS_TEST_G})

# add_executable(${PROJECT_NAME}_unittest ${SRCS_TEST})
# target_link_libraries(${PROJECT_NAME}_unittest  ${catkin_LIBRARIES} ${Poco_LIBRARIES} ${Boost_LIBRARIES} ${CORE_LIBRARY} gtest_main gmock_main ${YAML_CPP_LIBRARIES})
# target_precompile_headers(${PROJECT_NAME}_unittest PRIVATE include/core_agent/core_agent.h )

# add_test(NAME ${PROJECT_NAME}_unittest COMMAND ${PROJECT_NAME}_unittest)
# enable_testing()


# INSTALL 
install(
    TARGETS ${PROJECT_NAME} 
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
    ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
    RUNTIME DESTINATION "${CATKIN_GLOBAL_BIN_DESTINATION}"
    INCLUDES DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)

install(
    DIRECTORY "include/${PROJECT_NAME}"
    DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
    FILES_MATCHING PATTERN "*.h"
)

install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)

install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    DESTINATION "${config_install_dir}"
)
