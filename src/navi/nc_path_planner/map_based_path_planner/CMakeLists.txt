cmake_minimum_required(VERSION 3.0.2)
project(map_based_path_planner)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Release)




find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  core_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR} 
 CATKIN_DEPENDS  core_msgs nav_msgs
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


include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${NAVICORE_DIR}
  ${NAVICORE_DIR}/core
  ${NAVICORE_DIR}/core/types
  ${NAVICORE_DIR}/core_path
  ${exe_core_INCLUDE_DIRS}
)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(${PROJECT_NAME} STATIC ${SRC_FILES}) #STATIC/SHARED


file(GLOB_RECURSE EXPORT_HEADER_FILES CONFIGURE_DEPENDS
${CMAKE_CURRENT_SOURCE_DIR}/*.hpp )

set (EXPORT_HEADER_DIR ${PROJECT_NAME})

set (GLOBAL_HEADER_DIR ${CMAKE_BINARY_DIR}/include)
# ---------- 매크로 정의
  
# 헤더를 복사하는 매크로
macro (copy_headers MODULE_NAME OUT_DIR_PREFIX FILE_LIST)
  
    foreach (HEADER_FILE ${FILE_LIST})
        # 파일 명을 취득
        get_filename_component (FNAME ${HEADER_FILE} NAME)
        # 파일 명의 일부를 사용하여 타겟 명을 생성
        get_filename_component (TEMP_TARGET ${HEADER_FILE} NAME_WE)
        set (TEMP_TARGET copy_header_${OUT_DIR_PREFIX}_${TEMP_TARGET})
        # 복사하는 명령어
        add_custom_target (
            ${TEMP_TARGET}
            COMMAND
            ${CMAKE_COMMAND} -E copy ${HEADER_FILE}
                                     ${GLOBAL_HEADER_DIR}/nc_path_planner/${OUT_DIR_PREFIX}/src/${FNAME}
        )
        # 의존 관계를 지정(라이브러리 빌드 전에 복사가 실행되도록 한다)
        add_dependencies (${MODULE_NAME} ${TEMP_TARGET})
    endforeach (HEADER_FILE ${FILE_LIST})
  
endmacro (copy_headers MODULE_NAME OUT_DIR_PREFIX FILE_LIST)
  
copy_headers (${PROJECT_NAME} ${EXPORT_HEADER_DIR} "${EXPORT_HEADER_FILES}")


set_target_properties(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )
set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CORE_LIBRARY}
)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
