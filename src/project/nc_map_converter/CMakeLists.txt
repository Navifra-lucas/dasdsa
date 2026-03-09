cmake_minimum_required(VERSION 3.0.2)
project(nc_map_converter)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  core_msgs    REQUIRED
  pcl_conversions
  pcl_ros
  sensor_msgs
  tf2
)
find_package(TBB REQUIRED)
find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

#OpenCV setting
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()
link_directories(${YAMLCPP_LIBRARY_DIRS}

)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS system thread filesystem timer date_time log log_setup)


catkin_package(
    # INCLUDE_DIRS
    # LIBRARIES
    # CATKIN_DEPENDS        
    #     core_msgs
    #     roscpp
    #     tf2
)

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

include_directories(
  include
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${catkin_INCLUDE_DIRS}
  ${YAMLCPP_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
)

find_package(Poco COMPONENTS Foundation Util Net JSON Redis Data)

add_executable(nc_map_converter 
                src/nc_map_converter.cpp
                src/redis/nc_redis_reader.cpp
                src/net/nc_rest_api_client.cpp
)

add_dependencies(nc_map_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(nc_map_converter PROPERTIES OUTPUT_NAME nc_map_converter)

target_link_libraries(nc_map_converter
    ${catkin_LIBRARIES}
    ${CORE_LIBRARY}
    ${OpenCV_LIBRARIES} ${OpenMP_CXX_LIBRARIES}
    ${Boost_LIBRARIES} ${Boost_DATE_TIME_LIBRARY}  
    ${PCL_LIBRARIES}
    ${Poco_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
    Boost::log_setup Boost::log
    TBB::tbb -latomic
)

add_executable(pcd2grid src/pcd2ocgrid/main.cpp)
add_dependencies(pcd2grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd2grid
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
    ${CORE_LIBRARY}
    ${Boost_LIBRARIES} ${Boost_DATE_TIME_LIBRARY}  
    ${PCL_LIBRARIES}
    TBB::tbb -latomic
)

add_library(map_server_image_loader src/map_server/image_loader.cpp)
add_dependencies(map_server_image_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server_image_loader
    ${CORE_LIBRARY}
    ${BULLET_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
    ${Boost_LIBRARIES} ${Boost_DATE_TIME_LIBRARY}  
     ${PCL_LIBRARIES}
     Boost::log_setup Boost::log
     TBB::tbb -latomic
)

add_executable(map_server src/map_server/main.cpp)
add_dependencies(map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server
    map_server_image_loader
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
    ${CORE_LIBRARY}
    TBB::tbb -latomic
)

add_executable(map_server-map_saver src/map_server/map_saver.cpp)
add_dependencies(map_server-map_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(map_server-map_saver PROPERTIES OUTPUT_NAME map_saver)
target_link_libraries(map_server-map_saver
    ${catkin_LIBRARIES}
    ${CORE_LIBRARY}
    ${Boost_LIBRARIES} 
    ${Boost_DATE_TIME_LIBRARY}  
    ${PCL_LIBRARIES}
    glog
    Boost::log_setup Boost::log
    TBB::tbb -latomic
)

install( TARGETS
  nc_map_converter
  pcd2grid
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( TARGETS pcd2grid
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  COMPONENT pcd2grid
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install executables and/or libraries
install(TARGETS map_server-map_saver map_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS map_server_image_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)