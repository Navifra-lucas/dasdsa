cmake_minimum_required(VERSION 3.10)
project(dummy_plc_server)

# C++ 표준 설정 (POCO 호환성을 위해 C++17 사용)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# POCO 라이브러리 찾기
find_package(Poco REQUIRED COMPONENTS Net Foundation)

# 실행 파일 생성
add_executable(dummy_plc_server
    dummy_plc_server_main.cpp
    dummy_plc_server.cpp
)

# 라이브러리 링크
target_link_libraries(dummy_plc_server
    ${Poco_LIBRARIES}
    pthread
)

# 헤더 파일 경로
target_include_directories(dummy_plc_server PRIVATE
    ${Poco_INCLUDE_DIRS}
)