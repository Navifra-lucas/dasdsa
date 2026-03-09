cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)

# Source files
set(3D_PERCEPTION_SOURCES
    src/pallet_detector.cpp
    src/pallet_icp_matcher.cpp
    src/wingbody_detector.cpp
    src/segmentation/segmentation.cpp
)

# Create static library (same name as the old prebuilt .a)
add_library(3d_perception STATIC ${3D_PERCEPTION_SOURCES})

# Include directories - reuse the existing header paths from parent
target_include_directories(3d_perception
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include/answer/3d_perception
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include/answer/3d_localizer
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include/answer/common
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include/answer/logger
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../include/answer/visualizer
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Poco_INCLUDE_DIRS}
)

# Link dependencies
target_link_libraries(3d_perception
    PUBLIC
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Poco_LIBRARIES}
        spdlog::spdlog
        -lfmt
        -ltbb
        -latomic
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(3d_perception PUBLIC OpenMP::OpenMP_CXX)
endif()
