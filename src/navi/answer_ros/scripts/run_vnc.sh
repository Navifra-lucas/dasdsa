#!/bin/bash

# 사용자에게 디스플레이 번호 입력 받기
read -p "Enter display number (e.g., 5): " DISPLAY_NUM

# 포트 번호 계산 (5900 + DISPLAY_NUM)
VNC_PORT=$((5900 + DISPLAY_NUM))

# VNC 서버 시작
tigervncserver -xstartup /usr/bin/xterm -localhost :$DISPLAY_NUM

# noVNC 프록시 시작
./noVNC/utils/novnc_proxy --vnc localhost:$VNC_PORT
