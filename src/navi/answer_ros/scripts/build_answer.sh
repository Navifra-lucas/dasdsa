#!/bin/bash

# 현재 디렉토리 확인
SCRIPT_DIR="./src/ANSWER/"

# ROS_VERSION 환경 변수 확인
if [ -z "$ROS_VERSION" ]; then
    echo "ROS_VERSION is not set. Defaulting to ROS 1."
    ROS_VERSION=1
fi

# CMakeLists 파일 선택
if [ "$ROS_VERSION" == "2" ]; then
    echo "Using CMakeLists_ros2.txt"
    cp "$SCRIPT_DIR/CMakeLists_ros2.txt" "$SCRIPT_DIR/CMakeLists.txt"
else
    echo "Using CMakeLists_ros1.txt"
    cp "$SCRIPT_DIR/CMakeLists_ros1.txt" "$SCRIPT_DIR/CMakeLists.txt"
fi

echo "CMakeLists.txt has been updated based on ROS version."
echo "Navigator Submodule update"
git submodule update --init --recursive
if [ "$ROS_VERSION" == "2" ]; then
    if [ -d devel ]; then 
        echo "Maybe you have built the ROS1 package before. Cleaning up the workspace."
        rm -rf build devel install log
    fi
    echo "Build ROS2 package"
    colcon build --parallel-workers 1 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
else
    if [ -d log ]; then 
        echo "Maybe you have built the ROS2 package before. Cleaning up the workspace."
        rm -rf build devel install log
    fi
    echo "Build ROS1 package"
    catkin_make -DCMAKE_BUILD_TYPE=Release install
fi