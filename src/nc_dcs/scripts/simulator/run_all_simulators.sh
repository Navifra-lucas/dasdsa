#!/bin/bash

# nc_dcs 시뮬레이터 일괄 실행 스크립트

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$HOME/navifra_solution/navicore"

echo "========================================"
echo "nc_dcs Simulator Package Launcher"
echo "========================================"

# Source ROS workspace
cd $WORKSPACE_DIR
source devel/setup.zsh

echo "Starting simulators..."
echo ""

# Launch fork simulator
echo "[1/3] Starting Fork Simulator..."
rosrun nc_dcs fork_simulator.py &
FORK_PID=$!
sleep 0.5

# Launch path plan simulator
echo "[2/3] Starting Path Plan Simulator..."
rosrun nc_dcs path_plan_simulator.py &
PATH_PID=$!
sleep 0.5

# Launch perception simulator
echo "[3/3] Starting Perception Simulator..."
rosrun nc_dcs perception_simulator.py &
PERC_PID=$!
sleep 0.5

echo ""
echo "========================================"
echo "All simulators started!"
echo "  Fork Simulator PID: $FORK_PID"
echo "  Path Plan Simulator PID: $PATH_PID"
echo "  Perception Simulator PID: $PERC_PID"
echo "========================================"
echo ""
echo "To send a test mission, run:"
echo "  rosrun nc_dcs test_launcher.py"
echo ""
echo "Press Ctrl+C to stop all simulators"
echo ""

# Wait for user interrupt
trap "echo ''; echo 'Stopping simulators...'; kill $FORK_PID $PATH_PID $PERC_PID 2>/dev/null; exit" INT

wait
