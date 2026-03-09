
cmake_minimum_required(VERSION 3.0.2)
project(nc_path_planner)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Release)
add_subdirectory (map_based_path_planner)

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
        OUTPUT_VARIABLE SOURCE_GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
else()
    set(SOURCE_GIT_COMMIT_HASH "Unknown")
endif()
configure_file(
    "${PROJECT_SOURCE_DIR}/planner_config.h.in"
    "${PROJECT_SOURCE_DIR}/include/planner_config.h"
)

# install(DIRECTORY       include     # 헤더 파일들을 통째로 옮긴다
#         DESTINATION     ${build_navicore_SOURCE_DIR}/include/${PROJECT_NAME})

# install(DIRECTORY       ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}    # 헤더 파일들을 통째로 옮긴다
#         DESTINATION     ${build_navicore_SOURCE_DIR}/include)