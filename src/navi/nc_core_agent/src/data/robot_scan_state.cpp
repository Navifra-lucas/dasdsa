#include "core_agent/core_agent.h"

#include <core_agent/data/robot_scan_state.h>

using namespace NaviFra;

const std::string RobotScanStatus::KEY = "RobotScanStatus";

RobotScanStatus::RobotScanStatus()
{
    state_ = STATE_SCAN_OFF;
}

RobotScanStatus::~RobotScanStatus()
{
}

void RobotScanStatus::setState(STATE_SCAN state)
{
    state_ = state;
}

void RobotScanStatus::On()
{
    setState(STATE_SCAN_ON);
}

void RobotScanStatus::Off()
{
    setState(STATE_SCAN_OFF);
}
