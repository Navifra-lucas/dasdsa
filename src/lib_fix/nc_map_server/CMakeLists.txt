cmake_minimum_required(VERSION 3.0.2)
project(nc_brain_map_server)
add_compile_options(-std=c++17)

 
#OpenMP setting
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS}) 
  set(OpenMP_LIBS gomp)
endif()

#OpenCV setting
find_package(OpenCV REQUIRED)

#Boost setting
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS system thread filesystem timer date_time log log_setup)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()
link_directories(${YAMLCPP_LIBRARY_DIRS}

)

#Eigen3 setting
set(CMAKE_MODULE_PATH 
		${${CMAKE_MODULE_PATH}})
find_package(Eigen3 3.3 REQUIRED)

#JSONCPP setting
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

#Poco setting
find_package(Poco COMPONENTS Foundation Util Net JSON Redis Data)
if(Poco_FOUND)
    MESSAGE("FOUND Poco Libraries BUILD ")
    SET(Poco_INCLUDE_DIRS "/usr/local/include/Poco")
    MESSAGE(" Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs  
  core_msgs    REQUIRED
  nav_msgs
  pcl_ros
  sensor_msgs
  tf2
)

# find_package(Bullet REQUIRED)
# find_package(SDL REQUIRED)
# find_package(SDL_image REQUIRED)



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


catkin_package(
    INCLUDE_DIRS
        #include
    LIBRARIES
    CATKIN_DEPENDS
        core_msgs
        roscpp
        nav_msgs
        tf2
)

###########
## Build ##
###########
# find_package(thirdparty_core)
# set(thirdparty_DIR ${thirdparty_core_INCLUDE_DIRS})


message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

link_directories(${build_navicore_SOURCE_DIR}/lib
${CMAKE_SOURCE_DIR}/lib)
  
# 정규 표현식을 사용하여 libcore 라이브러리를 찾아 변수에 저장


file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/*.a")
set(NAVICORE_DIR ${build_navicore_SOURCE_DIR}/include)


file(GLOB LOCALIZER_LIBRARY "${WS_DIR}/navi_include/lib/libnc_localizer.a")
if(EXISTS ${nc_localizer})
  message("The LIB_USE is disabled.")
  set(LOCALIZER_LIBRARY nc_localizer)
else()
  message("The LIB_USE is enabled.")
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${core_msgs_INCLUDE_DIRS}
    # ${BULLET_INCLUDE_DIRS}
    # ${SDL_INCLUDE_DIR}
    # ${SDL_IMAGE_INCLUDE_DIRS}
	  ${core_INCLUDE_DIRS}
    ${YAMLCPP_LIBRARIES}
    ${OpenCV_INCLUDE_DIRS}
    ${Poco_INCLUDE_DIRS}  
    ${NAVICORE_DIR}
${NAVICORE_DIR}/core
${NAVICORE_DIR}/core/types
    
    ${thirdparty_DIR}

)

## nc_brain_map_server ##
add_executable(nc_brain_map_server src/nc_brain_map_server_main.cpp  )
add_dependencies(nc_brain_map_server  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(nc_brain_map_server PROPERTIES OUTPUT_NAME nc_brain_map_server)
target_link_libraries(nc_brain_map_server
    ${catkin_LIBRARIES}
    ${CORE_LIBRARY}
    ${YAMLCPP_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    Boost::log_setup Boost::log
)

install( TARGETS
    # nc_map_merger
    nc_brain_map_server
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)