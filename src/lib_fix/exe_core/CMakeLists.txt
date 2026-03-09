cmake_minimum_required(VERSION 3.0.2)
project(exe_core)
add_compile_options(-std=c++17 -O3)

set(CMAKE_BUILD_TYPE Release)

add_compile_options(-Wunused)

find_package(catkin REQUIRED)

set(Boost_USE_STATIC_LIBS ON)

find_package(Boost REQUIRED COMPONENTS system thread filesystem timer date_time log log_setup )
find_package(PkgConfig REQUIRED)
catkin_package(
   INCLUDE_DIRS  ${PROJECT_SOURCE_DIR} 
   types  
   mission_manager
   msg_information
   param_repository
)
message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")
file(GLOB CORE_LIBRARY "${build_navicore_SOURCE_DIR}/lib/libcore*.a")
set(CORE_DIR ${build_navicore_SOURCE_DIR}/include/core)

include_directories(types
                    ${PROJECT_SOURCE_DIR}                 
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${CORE_DIR}
                    ${CORE_DIR}/types
                    )

set(CMAKE_CXX_FLAGS "-L/usr/lib/x86_64-linux-gnu")

add_definitions(-D$ENV{ROS_DISTRO})

set (${PROJECT_NAME}_SRC
            core_calculator/core_calculator.cpp
            types/tf/transform_convertor.cpp
            types/polygon/polygon.cpp
            param_repository/param_repository.cpp
            mission_manager/mission_repository.cpp   
            mission_manager/job/job.cpp
            mission_manager/job/worker.cpp
            types/node/navi_node.cpp
            types/pos/pos.cpp
            debug/debug_visualizer.cpp
            )

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SRC}) #STATIC,SHARED

target_compile_options(${PROJECT_NAME} PRIVATE)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    ${PROJECT_NAME} 
    ${Boost_LIBRARIES} 
    ${Boost_DATE_TIME_LIBRARY} 
    ${CORE_LIBRARY}
    -latomic
    )

set_target_properties(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib OUTPUT_NAME "${PROJECT_NAME}")
set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib OUTPUT_NAME "${PROJECT_NAME}")


set (EXPORT_FORDER_LIST  mission_manager msg_information param_repository types core_calculator)

set (GLOBAL_HEADER_DIR ${CMAKE_BINARY_DIR}/include)

foreach (EXPORT_FORDER_FILE ${EXPORT_FORDER_LIST})

    set (EXPORT_HEADER_DIR ${EXPORT_FORDER_FILE})

    file(GLOB EXPORT_HEADER_FILES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/${EXPORT_HEADER_DIR}/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${EXPORT_HEADER_DIR}/*.h)
    
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
                                        ${GLOBAL_HEADER_DIR}/${PROJECT_NAME}/${OUT_DIR_PREFIX}/${FNAME}
            )
            # 의존 관계를 지정(라이브러리 빌드 전에 복사가 실행되도록 한다)
            add_dependencies (${MODULE_NAME} ${TEMP_TARGET})
        endforeach (HEADER_FILE ${FILE_LIST})
    
    endmacro (copy_headers MODULE_NAME OUT_DIR_PREFIX FILE_LIST)
    
    copy_headers (${PROJECT_NAME} ${EXPORT_HEADER_DIR} "${EXPORT_HEADER_FILES}")

endforeach (EXPORT_FORDER_FILE ${EXPORT_FORDER_LIST})


add_subdirectory (mission_manager)
add_subdirectory (msg_information)
add_subdirectory (types)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

install(TARGETS ${PROJECT_NAME} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)