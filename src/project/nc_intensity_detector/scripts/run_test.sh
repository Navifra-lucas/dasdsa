#!/bin/bash

# nc_intensity_detector 테스트 실행 스크립트

echo "=========================================="
echo "nc_intensity_detector 테스트 시작"
echo "=========================================="

# ROS 환경 설정
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

# 워크스페이스 환경 설정
if [ -f "$HOME/navifra_solution/navicore/devel/setup.bash" ]; then
    source $HOME/navifra_solution/navicore/devel/setup.bash
elif [ -f "$HOME/navifra_solution/navicore/install/setup.bash" ]; then
    source $HOME/navifra_solution/navicore/install/setup.bash
fi

# ROS 마스터 확인
if ! rostopic list &>/dev/null; then
    echo "ROS 마스터가 실행되지 않았습니다. roscore를 시작합니다..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
else
    ROSCORE_PID=""
fi

echo ""
echo "테스트 방법:"
echo "1. 자동 테스트 (런치 파일 사용):"
echo "   roslaunch nc_intensity_detector test_intensity_detector.launch"
echo ""
echo "2. 수동 테스트:"
echo "   터미널 1: rosrun nc_intensity_detector nc_intensity_detector_node"
echo "   터미널 2: rosrun nc_intensity_detector test_intensity_publisher.py"
echo "   터미널 3: rosrun nc_intensity_detector check_detection_result.py"
echo ""
echo "3. 검출 시작/중지:"
echo "   rostopic pub /nc_intensity_detector/start std_msgs/Bool \"data: true\""
echo "   rostopic pub /nc_intensity_detector/stop std_msgs/Bool \"data: true\""
echo ""
echo "4. 결과 확인:"
echo "   rostopic echo /nc_intensity_detector/pose"
echo ""

# 자동 테스트 실행 여부 확인
read -p "자동 테스트를 실행하시겠습니까? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "자동 테스트를 시작합니다..."
    roslaunch nc_intensity_detector test_intensity_detector.launch
fi

# roscore 종료 (필요한 경우)
if [ ! -z "$ROSCORE_PID" ]; then
    kill $ROSCORE_PID 2>/dev/null
fi

