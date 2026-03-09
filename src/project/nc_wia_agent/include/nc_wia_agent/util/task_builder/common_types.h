#pragma once

#include "nc_wia_agent/data/robot_basic_status.h"

namespace NaviFra {

enum NcActionType
{
    ACTION_MOVE = 1,
    ACTION_DOCKING = 6,
    ACTION_STANDBY = 7,
    ACTION_DOCKING_OUT = 18,
    ACTION_LIFT = 20,
    ACTION_FORK = 23,
    ACTION_CHARGE = 91,
    ACTION_WINGBODY_PERCEPTION = 9999
};

enum MoveDirection
{
    MOVE_FORWARD = 1000,
    MOVE_BACKWARD = 2000,
    CRAB_FORWARD = 1001,
    CRAB_BACKWARD = 2001
};

enum PalletType
{
    Pallet_Fender = 0,
    Pallet_Door = 1
};

struct ActionData {
    NaviFra::Pos o_last_pos;
    std::vector<GoalInfo> goal_info;
    int last_drive_direction;
    bool b_curve_pending = false;
    float f_last_curve_speed = 0.0f;
    float f_last_curve_radius = 0.0f;
    NaviFra::Pos o_next_pos;  // 다음 move 노드의 goal_pos
    bool b_has_next_goal = false;
};
}  // namespace NaviFra