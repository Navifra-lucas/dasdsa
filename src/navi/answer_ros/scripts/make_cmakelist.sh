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