cmake_minimum_required(VERSION 3.16)
project(nc_brain_agent)

file(STRINGS "${PROJECT_SOURCE_DIR}/VERSION" PACKAGE_VERSION)
string(REGEX REPLACE "([0-9]+)\\.[0-9]+\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.([0-9]+)\\.[0-9]+.*" "\\1" CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION})
string(REGEX REPLACE "[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION})

set(PROJECT_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Append our module directory to CMake
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(NcAgentMacro)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost REQUIRED COMPONENTS log log_setup)

find_package(Poco COMPONENTS Foundation Util Net NetSSL Crypto JSON Redis Data)
## Find catkin and any catkin packages
add_compile_options(-std=c++17)

if(Poco_FOUND)
    MESSAGE("FOUND Poco Libraries BUILD Brain Agent")
    find_package(catkin REQUIRED COMPONENTS 
                roscpp
                rosbag
                roslib
                std_msgs   
                core_msgs
                core_agent
                tf2_ros
                tf
    )
    
    find_package(yaml-cpp REQUIRED)

    ## Declare a catkin package
    catkin_package(
      LIBRARIES rosbag_playtime
      DEPENDS  
      roscpp
      rosbag
      roslib 
      core_msgs
      tf2_ros
      tf
    )

    SET(Poco_INCLUDE_DIRS "/usr/local/include/Poco")

    MESSAGE(" Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")



    message(STATUS "build_navicore_SOURCE_DIR: ${build_navicore_SOURCE_DIR}")

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
      ${YAML_CPP_INCLUDE_DIRS} 
      ${NAVICORE_DIR}
      ${NAVICORE_DIR}/core
      ${NAVICORE_DIR}/core/types
      ${exe_core_INCLUDE_DIRS}
    )

    execute_process(
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(PAKAGE_FILE_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

    configure_file(version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/nc_brain_agent/version.h @ONLY)
    configure_file(nc_brain_agent_macro.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/nc_brain_agent/nc_brain_agent_macro.h @ONLY)

    # Sources
    file(GLOB SRCS_G 
            "src/*.cpp" 
            "src/action/async/*.cpp"
            "src/redis/*.cpp" 
            "src/message/*.cpp"
            "src/data/*.cpp"
            "src/utils/*.cpp"
            "src/net/*.cpp"
            "src/action/*.cpp"
            "src/ros/*.cpp"            
            "src/service/*.cpp"
            "src/initializer/*.cpp"
            "src/web/common/*.cpp"            
            "src/web/router/*.cpp"                                                            
            )

    NAVIFRA_SOURCES_AUTO(SRCS ${SRCS_G})

    add_executable(${PROJECT_NAME}  ${SRCS})
    MESSAGE(STATIC "${core_LIBRARIES} = ${core_LIBRARIES}")

    
    add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_precompile_headers(${PROJECT_NAME}  PRIVATE include/nc_brain_agent/nc_brain_agent.h)
    ## 임시
    add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/launch/configs
            ${PAKAGE_FILE_PATH}/launch/configs
        COMMENT "Copying test files to output directory"
    )
    ###

    if(CMAKE_BUILD_TYPE MATCHES "Debug")
    # Add AddressSanitizer compile options for the target only in Debug build
          target_compile_options(${PROJECT_NAME} PRIVATE -lasan -fsanitize=address)
            # Link AddressSanitizer runtime library
          target_link_libraries(${PROJECT_NAME}  -fsanitize=address ${CORE_LIBRARY} ${catkin_LIBRARIES} ${Poco_LIBRARIES} ${YAML_CPP_LIBRARIES} Boost::log_setup Boost::log)
    else()
          target_link_libraries(${PROJECT_NAME}  ${CORE_LIBRARY} ${catkin_LIBRARIES} ${Poco_LIBRARIES} ${YAML_CPP_LIBRARIES} Boost::log_setup Boost::log)
    endif()
    
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    enable_testing()

    set(TEST_FILES_OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/)
    set(TEST_FILES ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/)

    configure_file(nc_brain_map_test.h.in ${CMAKE_CURRENT_SOURCE_DIR}/tests/nc_brain_map_test.h @ONLY)
    
    # Sources
    # file(GLOB SRCS_TEST_G 
    #         "tests/*.cpp" 
    #         "src/action/async/*.cpp"
    #         "src/redis/*.cpp" 
    #         "src/message/*.cpp"
    #         "src/data/*.cpp"
    #         "src/utils/*.cpp"
    #         "src/net/*.cpp"
    #         "src/ros/*.cpp"
    #         "src/action/*.cpp"    
    #         "src/action/ros/*.cpp" 
    #         "src/service/*.cpp"
    #         "src/web/common/*.cpp"
    #         "src/web/router/*.cpp"     
    #         )

    # NAVIFRA_SOURCES_AUTO(SRCS_TEST ${SRCS_TEST_G})

    # add_executable(${PROJECT_NAME}_unittest
    #                 ${SRCS_TEST} 
    #                 src/nc_robot_agent_activity.cpp
    #                 src/nc_robot_agent.cpp        
    #                 src/nc_robot_agent_message.cpp
    #                 src/nc_robot_agent_init.cpp                    
    #                 src/ros/nc_ros_launch_manager.cpp
    # )

    # target_link_libraries(${PROJECT_NAME}_unittest 
    # ${CORE_LIBRARY} 
    # ${catkin_LIBRARIES} 
    # ${Poco_LIBRARIES} 
    # gtest_main 
    # ${YAML_CPP_LIBRARIES} 
    # Boost::log_setup 
    # Boost::log
    # target_compile_definitions(${PROJECT_NAME}_unittest PUBLIC CPP_UNIT_TEST)
    # target_precompile_headers(${PROJECT_NAME}_unittest PRIVATE include/nc_brain_agent/nc_brain_agent.h)
    # add_test(NAME ${PROJECT_NAME}_unittest COMMAND ${PROJECT_NAME}_unittest)

    # add_custom_command(
    #     TARGET ${PROJECT_NAME}_unittest POST_BUILD
    #     COMMAND ${CMAKE_COMMAND} -E copy_directory
    #         ${TEST_FILES}
    #         ${TEST_FILES_OUTPUT}
    #     COMMENT "Copying test files to output directory"
    # )

    install( TARGETS
      ${PROJECT_NAME} 
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

    )
    install(
      DIRECTORY launch/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
    
endif()
