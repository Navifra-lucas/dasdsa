cmake_minimum_required(VERSION 3.0.2)
project(nc_io_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  core_msgs 
  task_msgs
  motor_msgs
)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)
find_package(Poco COMPONENTS Foundation Util Net Data)

catkin_package(
  DEPENDS  
  roscpp
)


link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장
file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)
message(STATUS "exe_core_INCLUDE_DIRS: ${exe_core_INCLUDE_DIRS}")

include_directories(
    include
    ${Poco_INCLUDE_DIRS}   
    ${catkin_INCLUDE_DIRS}
    ${NAVICORE_DIR}
    ${NAVICORE_DIR}/core
    ${NAVICORE_DIR}/core/types
)

add_compile_options(-std=c++17)

SET(Poco_INCLUDE_DIRS "/usr/local/include/Poco")

# file(GLOB SOURCES "src/*.cpp")

file(GLOB SOURCES
      "src/*.cpp"
      "src/controller/*.cpp" 
      "src/data/*.cpp"
      "src/manager/*.cpp" 
      "src/plc/*.cpp"
      "src/ros/*.cpp"
      "src/state/*.cpp"                                                          
)

add_executable(${PROJECT_NAME} ${SOURCES})

add_dependencies(${PROJECT_NAME} ${core_msgs_EXPORTED_TARGETS} task_msgs)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
# Add AddressSanitizer compile options for the target only in Debug build
      target_compile_options(${PROJECT_NAME} PRIVATE -lasan -fsanitize=address)

      target_link_libraries(${PROJECT_NAME}
        -fsanitize=address 
        ${catkin_LIBRARIES}
        ${Poco_LIBRARIES}
        core
        Boost::log_setup Boost::log
      )
else()
      target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Poco_LIBRARIES}
        core
        Boost::log_setup Boost::log
      )
endif()

add_executable(dummy_plc_server
  dummy_PLC/dummy_plc_server_main.cpp
)

target_include_directories(dummy_plc_server PRIVATE
  dummy_PLC
)

target_link_libraries(dummy_plc_server
  pthread
)

install(TARGETS ${PROJECT_NAME}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

