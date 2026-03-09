cmake_minimum_required(VERSION 3.1)
project(nc_calibrator)

add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(TBB REQUIRED)

find_package(Poco COMPONENTS Foundation Util Net JSON Redis Data)
file(GLOB SLAM_LIBRARY "${build_navicore_SOURCE_DIR}/lib/libnavifra_slam.a")
message("SLAM_LIBRARY: ${SLAM_LIBRARY}")
message("navifra_slam: ${navifra_slam}")
set(SLAM_DIR ${build_navicore_SOURCE_DIR}/include/navifra_slam)
if(navifra_slam_SOURCE_DIR)
  message("The SLAM_LIBRARYLIB_USE is disabled.")
  set(SLAM_LIBRARY navifra_slam)
  set(SLAM_DIR ${navifra_slam_SOURCE_DIR})
else()
  message("The SLAM_LIBRARYLIB_USE is enabled.")
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS}) 
  set(OpenMP_LIBS gomp)
endif()


# core setting 
find_package(catkin REQUIRED
  COMPONENTS
    laser_geometry
    message_filters
    nav_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
    core_msgs
    visualization_msgs
    roslib
    pcl_ros
)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup )

catkin_package(
  CATKIN_DEPENDS
  core_msgs
    # nav_msgs
    # rosbag
    # roscpp
    # sensor_msgs
    # std_srvs
    # tf2
    # tf2_msgs
    # tf2_ros
    # visualization_msgs
    # roslib
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


file(GLOB LOCALIZER_LIBRARY "${build_navicore_SOURCE_DIR}/lib/libnc_localizer.a")
set(LOCALIZER_DIR ${build_navicore_SOURCE_DIR}/include/nc_localizer)
if(nc_localizer_SOURCE_DIR)
  message("The LIB_USE is disabled.")
  set(LOCALIZER_LIBRARY nc_localizer)
  set(LOCALIZER_DIR ${nc_localizer_SOURCE_DIR}/include)
else()
  message("The LIB_USE is enabled.")
endif()



find_package(yaml-cpp REQUIRED)
include(CheckSymbolExists)

include_directories(include)
include_directories(
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${LOCALIZER_DIR}
  ${LOCALIZER_DIR}/include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} 
  ${LOCALIZER_LIBRARY_INCLUDE_DIRS}
  ${thirdparty_DIR}
  ${PARAM_CLIENT_DIR}
  ${YAML_CPP_INCLUDE_DIRS} 
  ${SLAM_DIR}
  ${SLAM_DIR}/include
  ${exe_core_INCLUDE_DIRS}
)
include_directories(src/include)

check_include_file(unistd.h HAVE_UNISTD_H)
if (HAVE_UNISTD_H)
  add_definitions(-DHAVE_UNISTD_H)
endif (HAVE_UNISTD_H)

check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)
if (HAVE_DRAND48)
  add_definitions(-DHAVE_DRAND48)
endif (HAVE_DRAND48)


file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
# add_dependencies(nc_calibrator ${nc_calibrator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(nc_calibrator  src/nc_calibrator.cpp)
add_dependencies(nc_calibrator ${catkin_EXPORTED_TARGETS})

target_link_libraries( nc_calibrator    
  ${LOCALIZER_LIBRARY}       
  ${SLAM_LIBRARY}  
  ${CORE_LIBRARY}   
  nabo
  pointmatcher 
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PARAM_CLIENT_LIBRARY}
  ${YAML_CPP_LIBRARIES}
  glog
  ${OpenCV_LIBRARIES} ${OpenMP_CXX_LIBRARIES} 
  ${Boost_DATE_TIME_LIBRARY}  ${PCL_LIBRARIES} 
  Boost::log_setup 
  Boost::log
  -lssl 
  -lcrypto
  # -ltbb 
  TBB::tbb
  -latomic
  ${CERES_LIBRARIES}
  ${Poco_LIBRARIES}
  ${Eigen_LIBRARIES}
  ceres
  # yaml-cpp-pm
) 

add_executable(nc_auto_repeat_driver src/nc_auto_repeat_driver.cpp)

add_dependencies(nc_auto_repeat_driver ${catkin_EXPORTED_TARGETS})

target_link_libraries(nc_auto_repeat_driver 
  ${CORE_LIBRARY}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} nabo
  Boost::log_setup
  Boost::log
  ${Poco_LIBRARIES}

)


install( TARGETS  ${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( TARGETS  nc_auto_repeat_driver
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)