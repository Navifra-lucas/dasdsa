#ifndef NAVIFRA_CUSTOM_DRIVER_STRUCT_H_
#define NAVIFRA_CUSTOM_DRIVER_STRUCT_H_

#include <chrono>
#include <string>
namespace NaviFra {
// 0: DD, 1: QD , 2: SD
enum KINEMATICS
{
    DD = 0,
    QD = 1,
    SD = 2,
    QDOCT = 3,
};

enum PCAN_MODE
{
    PCAN_TONGYI = 0,
    PCAN_CURTIS = 1,
    PCAN_LEEDSHINE = 2,
    PCAN_ROBOTEQ = 3,
    PCAN_SWAZM = 4,
    PCAN_LEEDSHINE_NEWCOMODO = 5,
    PCAN_MAXON_NEWCOMODO = 6,
    PCAN_LEEDSHINE_FAB = 7,
    PCAN_SAMYANG = 8,
};

enum PCAN_BAUDRATE
{
    NC_PCAN_BAUD_125K = 796,
    NC_PCAN_BAUD_250K = 284,
    NC_PCAN_BAUD_500K = 28,
    NC_PCAN_BAUD_800K = 22,
    NC_PCAN_BAUD_1M = 20,
};

enum PCAN_BUS
{
    NC_PCAN_USBBUS1 = 81,
    NC_PCAN_USBBUS2 = 82,
};

enum CAN_MODE
{
    CAN_COMODO = 0,
    CAN_ROBOTEQ = 1,
    CAN_SWAZM = 2,
    CAN_CURTIS = 3,
    CAN_LEEDSHINE = 4,
    CAN_MAXON = 5,
    CAN_SWAZM46 = 6,
    CAN_JIMU = 7,
    CAN_RUSSELL = 8,
    CAN_AMC = 9,
    CAN_SIM = 999,
};

enum SERIAL_MODE
{
    SERIAL_COMODO = 0,
    SERIAL_UNMANNED = 1,
    SERIAL_ZLTECH = 2,
    SERIAL_LEADSHINE = 3
};

enum SERIAL_BAUDRATE
{
    NC_SERIAL_BAUD_9600 = 9600,
    NC_SERIAL_BAUD_19200 = 19200,
    NC_SERIAL_BAUD_38400 = 38400,
    NC_SERIAL_BAUD_57600 = 57600,
    NC_SERIAL_BAUD_115200 = 115200,
};

struct Wheel_Cmd_t {
    // command DD && QD & SD
    float f_FL_target_rpm = 0;
    float f_FR_target_rpm = 0;
    float f_RL_target_rpm = 0;
    float f_RR_target_rpm = 0;
    float f_FL_target_deg = 0;
    float f_FR_target_deg = 0;
    float f_RL_target_deg = 0;
    float f_RR_target_deg = 0;

    bool b_steer_control_always = false;

    // etc
    float f_robot_target_speed = 0;
};

struct PcanParameters_t  // pcan
{
    int n_pcan_mode = PCAN_MODE::PCAN_CURTIS;
    int n_pcan_baudrate = PCAN_BAUDRATE::NC_PCAN_BAUD_500K;
    int n_pcan_bus = PCAN_BUS::NC_PCAN_USBBUS1;

    int n_FL_traction_pcan_id = 0x01;
    int n_FR_traction_pcan_id = 0x02;
    int n_RL_traction_pcan_id = 0x03;
    int n_RR_traction_pcan_id = 0x04;
    int n_FL_steer_pcan_id = 0x05;
    int n_FR_steer_pcan_id = 0x06;
    int n_RL_steer_pcan_id = 0x07;
    int n_RR_steer_pcan_id = 0x08;
    int n_FL_abssteer_pcan_id = 0x09;
    int n_FR_abssteer_pcan_id = 0x0A;
    int n_RL_abssteer_pcan_id = 0x0B;
    int n_RR_abssteer_pcan_id = 0x0C;
};

struct CanParameters_t  // can
{
    int n_can_mode = CAN_MODE::CAN_COMODO;
    int n_turn_table_abssteer_can_id = 12;
    // int n_can_baudrate = CAN_BAUDRATE::NC_CAN_BAUD_500K;
    // int n_can_bus = CAN_BUS::NC_CAN_USBBUS1;
    // int n_project_id = LEADSHINE_MODE::DEFAULT;
    int n_FL_traction_can_id = 0x01;
    int n_FR_traction_can_id = 0x03;
    int n_RL_traction_can_id = 0x04;
    int n_RR_traction_can_id = 0x02;
    int n_FL_steer_can_id = 0x05;
    int n_FR_steer_can_id = 0x06;
    int n_RL_steer_can_id = 0x07;
    int n_RR_steer_can_id = 0x08;
    int n_FL_abssteer_can_id = 0x09;
    int n_FR_abssteer_can_id = 0x0A;
    int n_RL_abssteer_can_id = 0x0B;
    int n_RR_abssteer_can_id = 0x0C;
};

struct SerialParameters_t  // serial
{
    std::string s_serial_tty = "/dev/ttyUSB_Serial";
    int n_serial_mode = SERIAL_MODE::SERIAL_COMODO;
    int n_serial_baudrate = SERIAL_BAUDRATE::NC_SERIAL_BAUD_57600;
    int n_serial_rec_buf = 256;
};


struct WheelParameters_t {
    float f_FL_wheel_x_m = 0.3;
    float f_FR_wheel_x_m = 0.3;
    float f_RL_wheel_x_m = -0.3;
    float f_RR_wheel_x_m = -0.3;
    float f_FL_wheel_y_m = 0;
    float f_FR_wheel_y_m = 0;
    float f_RL_wheel_y_m = 0;
    float f_RR_wheel_y_m = 0;
    float f_FL_wheel_axis_offset_m = 0;
    float f_FR_wheel_axis_offset_m = 0;
    float f_RL_wheel_axis_offset_m = 0;
    float f_RR_wheel_axis_offset_m = 0;
    float f_FL_wheel_diameter_m = 0.300;
    float f_FR_wheel_diameter_m = 0.300;
    float f_RL_wheel_diameter_m = 0.300;
    float f_RR_wheel_diameter_m = 0.300;
};

struct TractionParameters_t {
    bool b_FL_traction_target_rpmd = true;
    bool b_FR_traction_target_rpmd = true;
    bool b_RL_traction_target_rpmd = true;
    bool b_RR_traction_target_rpmd = true;
    bool b_FL_traction_feedback_rpmd = true;
    bool b_FR_traction_feedback_rpmd = true;
    bool b_RL_traction_feedback_rpmd = true;
    bool b_RR_traction_feedback_rpmd = true;
    bool b_FL_traction_encoderd = true;
    bool b_FR_traction_encoderd = true;
    bool b_RL_traction_encoderd = true;
    bool b_RR_traction_encoderd = true;

    float f_FL_traction_gear_ratio = 29.3;
    float f_FR_traction_gear_ratio = 29.3;
    float f_RL_traction_gear_ratio = 29.3;
    float f_RR_traction_gear_ratio = 29.3;
    float f_FL_traction_encoder_pulse = 4000;
    float f_FR_traction_encoder_pulse = 4000;
    float f_RL_traction_encoder_pulse = 4000;
    float f_RR_traction_encoder_pulse = 4000;
    float f_FL_traction_rpm_min = 0;
    float f_FR_traction_rpm_min = 0;
    float f_RL_traction_rpm_min = 0;
    float f_RR_traction_rpm_min = 0;
    float f_FL_traction_rpm_max = 250;
    float f_FR_traction_rpm_max = 250;
    float f_RL_traction_rpm_max = 250;
    float f_RR_traction_rpm_max = 250;
    float f_traction_overcurrent_amp_std = 130;
    float f_traction_overcurrent_time_std = 3;
    std::string s_target_unit = "rpm";
    int n_target_pulse = 1;
};

// interface
struct InterfaceParameters_t {
    // PCAN
    bool b_pcan_use = false;
    PcanParameters_t st_pcan_param;

    // CAN
    bool b_can_use = false;
    bool b_can_sim_use = false;
    CanParameters_t st_can_param;

    // Serial
    bool b_serial_use = false;
    SerialParameters_t st_serial_param;
};

// driver
struct DriverParameters_t {
    WheelParameters_t st_wheel_param;
    TractionParameters_t st_traction_param;

    bool b_abs_steer_encoder_use = false;
    bool b_use_imu_fusion = true;

    float f_brake_threshold_rpm = 5;
    float f_traction_overcurrent_amp_std = 130;
    float f_traction_overcurrent_time_std = 3;
    float f_steer_overcurrent_amp_std = 130;
    float f_steer_overcurrent_time_std = 3;
};

struct IMU_Data_t {
    float f_roll;
    float f_pitch;
    float f_yaw;
    float f_angular_vel_x;
    float f_angular_vel_y;
    float f_angular_vel_z;
    float f_linear_acc_x;
    float f_linear_acc_y;
    float f_linear_acc_z;
};

}  // namespace NaviFra
#endif
