#ifndef NAVIFRA_AGENT_TYPES_H
#define NAVIFRA_AGENT_TYPES_H

#include <core_agent/data/node_path.h>
#include <core_agent/data/robot_rect.h>
#include <core_agent/data/velocity.h>

#include <string>
#include <map>

namespace NaviFra {
namespace TypeCalibration {
const static std::string FOWARD = "forward";
const static std::string BACKWARD = "backward";
const static std::string LEFT = "left";
const static std::string RIGHT = "right";
const static std::string LIDAR = "lidar";
}  // namespace TypeCalibration

struct ProcessResult {
    bool success;
    std::string message;
};

struct Position {
    float x;
    float y;
    float z;
    float deg;
};

struct Orientation {
    float x;
    float y;
    float z;
    float w;
};

struct Pose {
    Position position;
    Orientation orientation;
};

enum MOTOR
{
    MOTOR_TRACTION_FL = 0,
    // MOTOR_TRACTION_FR,
    // MOTOR_TRACTION_RL,
    MOTOR_TRACTION_RR,
    MOTOR_STEER_FL,
    // MOTOR_STEER_FR,
    // MOTOR_STEER_RL,
    MOTOR_STEER_RR,
    MOTOR_LENGTH
};

enum MOTOR_INFO
{
    MOTOR_INFO_CURRENT = 0,
    MOTOR_INFO_TARGET_RPM,
    MOTOR_INFO_FEEDBACK_RPM,
    MOTOR_INFO_ENCODER,
    MOTOR_INFO_LENGTH
};

enum ROBOT_TYPE
{
    ROBOT_TYPE_DD = 0,
    ROBOT_TYPE_QD,
    ROBOT_TYPE_SD,
    ROBOT_TYPE_QD_OCT
};

enum DRIVE_DD
{
    DRIVE_DD_LEFT = 0,
    DRIVE_DD_RIGHT
};

enum DRIVE_QD
{
    DRIVE_QD_LEFT = 0,
    DRIVE_QD_RIGHT,
};

enum DRIVE_QD_OCT
{
    DRIVE_QD_OCT_FL = 0,
    DRIVE_QD_OCT_FR,
    DRIVE_QD_OCT_RL,
    DRIVE_QD_OCT_RR,
};

enum DRIVE_SD
{
    DRIVE_SD_FRONT = 0,
};

enum DRIVE_WHEEL
{
    DRIVE_WHEEL_FL = 0,
    DRIVE_WHEEL_FR,
    DRIVE_WHEEL_RL,
    DRIVE_WHEEL_RR,
};

struct RobotSize {
    int width;
    int height;
};

struct Setting {
    int map;
    int totalNodes;
    int teachedNodes;
};

struct JigInfo {
    int nRotation;
    int nLift;
    bool bClamp;
    bool bHold;
    bool bProduct;
    bool error;
    bool communication;
};

struct robotDrivingInfo {
    bool isGoalStart;
    bool isGoalComplete;
    bool robotError;
    bool isMoving;
};
struct DockingNode {
    float f_x;
    float f_y;
    float f_angle_deg;
    float f_linear_speed;
    int n_drive_type;
    /* data */
};

struct ReadRegister {
    int command_num;                  // PDU_READ_REGISTER_COMMAND_NUM
    int command;                      // PDU_READ_REGISTER_COMMAND
    int target_node;                  // PDU_READ_REGISTER_TARGET_NODE
    int x_position;                   // PDU_READ_REGISTER_X_POSITION
    int y_position;                   // PDU_READ_REGISTER_Y_POSITION
    int angle_position;               // PDU_READ_REGISTER_ANGLE_POSITION
    int speed_limit;                  // PDU_READ_REGISTER_SPEED_LIMIT
    int jog_enable;                   // PDU_READ_REGISTER_JOG_ENABLE
    int jog_speed;                    // PDU_READ_REGISTER_JOG_SPEED
    int jog_angle_speed;              // PDU_READ_REGISTER_JOG_ANGLE_SPEED
    int jog_steer_deg;                // PDU_READ_REGISTER_JOG_STEER_DEG
    int speed_type;                   // PDU_READ_REGISTER_SPEED_TYPE
    int none_speed;                   // PDU_READ_REGISTER_NONE_SPEED
    int empty_speed;                  // PDU_READ_REGISTER_EMPTY_SPEED
    int load_state;                   // PDU_READ_REGISTER_LOAD_STATE
    int auto_on;                      // PDU_READ_REGISTER_AUTO_ON
    int battery;                      // PDU_READ_REGISTER_BATTERY
    int fork_up_down_position;        // PDU_READ_REGISTER_FORK_UP_DOWN_POSITION
    int fork_up_down_complete;        // PDU_READ_REGISTER_FORK_UP_DOWN_COMPLETE
    int tilting_up_down;              // PDU_READ_REGISTER_TILTING_UP_DOWN
    int fork_width;                   // PDU_READ_REGISTER_FORK_WIDTH
    int pallet_touch;                 // PDU_READ_REGISTER_PALLET_TOUCH
    int lsc_1;                        // PDU_READ_REGISTER_LSC_1
    int lsc_2;                        // PDU_READ_REGISTER_LSC_2
    int lsc_3;                        // PDU_READ_REGISTER_LSC_3
    int lsc_4;                        // PDU_READ_REGISTER_LSC_4
    int charge_state;                 // PDU_READ_REGISTER_CHARGE_STATE
    int job_cancel;                   // PDU_READ_REGISTER_JOB_CANCEL
    int pallet_id_remove;             // PDU_READ_REGISTER_PALLET_ID_REMOVE
    int plc_alarm;                    // PDU_READ_REGISTER_PLC_ALARM
};

struct ReadCoil {
    bool read_front_safety_scanner;
    bool read_left_safety_scanner;
    bool read_right_safety_scanner;
};

struct ErrorPose {
    float x;
    float y;
    float deg;
};

struct DockingType {
    bool b_start_pause;
    bool b_flag;
    bool b_in;
    bool b_state;
    /* data */
};

struct Job {
    std::string current_task_id;
    std::string previous_task_id;
};

struct Map {
    std::string current_area_id;
    std::string current_map_id;
    std::map<std::string, std::string> area;
};

}  // namespace NaviFra
#endif  // NC_AGENT_TYPES_H