#ifndef NAVIFRA_DRIVER_STRUCT_H_
#define NAVIFRA_DRIVER_STRUCT_H_

#include "core_msgs/NaviAlarm.h"
#include "motor_msgs/MotorData.h"

#include <chrono>
#include <cstdint>
#include <string>

namespace NaviFra {

enum MOTOR : int32_t
{
    MOTOR_TRACTION_FL = 0,
    MOTOR_TRACTION_FR,
    MOTOR_TRACTION_RL,
    MOTOR_TRACTION_RR,
    MOTOR_STEER_FL,
    MOTOR_STEER_FR,
    MOTOR_STEER_RL,
    MOTOR_STEER_RR,
    MOTOR_ABS_ENCODER_FL,
    MOTOR_ABS_ENCODER_FR,
    MOTOR_ABS_ENCODER_RL,
    MOTOR_ABS_ENCODER_RR,
    MOTOR_UPPER_TURNTABLE,

    MOTOR_LENGTH,
    MOTOR_ALL = 999
};

using MotorId = MOTOR;
using MotorDataMap = std::map<MotorId, motor_msgs::MotorData>;

enum KINEMATICS
{
    DD = 0,
    QD = 1,
    SD = 2,
    OD = 3,
};

enum MotorType
{
    Traction = 0,
    Steer,
    AbsEncoder,
    TurnTable,
};

// sensor value data struct
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

struct Motor_ERROR {
    bool b_FL_error_update = false;
    std::chrono::steady_clock::time_point tp_FL_error = std::chrono::steady_clock::now();
    std::string s_FL_error_text = "";  // for debug
    int n_FL_error_code = 0;

    bool b_FR_error_update = false;
    std::chrono::steady_clock::time_point tp_FR_error = std::chrono::steady_clock::now();
    std::string s_FR_error_text = "";  // for debug
    int n_FR_error_code = 0;

    bool b_RL_error_update = false;
    std::chrono::steady_clock::time_point tp_RL_error = std::chrono::steady_clock::now();
    std::string s_RL_error_text = "";  // for debug
    int n_RL_error_code = 0;

    bool b_RR_error_update = false;
    std::chrono::steady_clock::time_point tp_RR_error = std::chrono::steady_clock::now();
    std::string s_RR_error_text = "";  // for debug
    int n_RR_error_code = 0;

    bool b_UTT_error_update = false;
    std::chrono::steady_clock::time_point tp_UTT_error = std::chrono::steady_clock::now();
    std::string s_UTT_error_text = "";  // for debug
    int n_UTT_error_code = 0;

    bool b_etc_error_update = false;
    std::chrono::steady_clock::time_point tp_etc_error = std::chrono::steady_clock::now();
    std::string s_etc_error_text = "";  // for debug
    int n_etc_error_code = 0;
};

struct Wheel_Data_t {
    // encoder
    bool b_traction_encoder_init = false;
    bool b_traction_encoder_update = false;
    double d_traction_encoder = 0;
    double d_traction_encoder_acc = 0;
    // feedback
    bool b_traction_feedback_update = false;
    float f_traction_feedback_rpm = 0;
    float f_traction_current = 0;
    float f_traction_voltage = 0;
    float f_traction_error = 0;
    bool b_steer_feedback_update = false;
    float f_steer_target_rpm = 0;
    float f_steer_feedback_rpm = 0;
    float f_steer_current = 0;
    float f_steer_angle_deg = 0;
    float f_steer_error = 0;
    bool b_abssteer_feedback_update = false;
    float f_abssteer_angle_deg = 0;
    bool b_is_enable = false;
    bool b_is_error = false;
    bool b_feedback_brake_on = false;
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

struct SteerParameters_t {
    bool b_FL_target_steer_angled = true;
    bool b_FR_target_steer_angled = true;
    bool b_RL_target_steer_angled = true;
    bool b_RR_target_steer_angled = true;
    bool b_FL_feedback_steer_angled = true;
    bool b_FR_feedback_steer_angled = true;
    bool b_RL_feedback_steer_angled = true;
    bool b_RR_feedback_steer_angled = true;

    float f_steer_stop_thr_deg = 5.0;
    float f_steer_start_thr_deg = 10.0;
    float f_FL_steer_gear_ratio = 1.0;
    float f_FR_steer_gear_ratio = 1.0;
    float f_RL_steer_gear_ratio = 1.0;
    float f_RR_steer_gear_ratio = 1.0;

    float f_FL_steer_max_angle_deg = 120;
    float f_FR_steer_max_angle_deg = 120;
    float f_RL_steer_max_angle_deg = 120;
    float f_RR_steer_max_angle_deg = 120;
    float f_FL_steer_min_angle_deg = -120;
    float f_FR_steer_min_angle_deg = -120;
    float f_RL_steer_min_angle_deg = -120;
    float f_RR_steer_min_angle_deg = -120;
    float f_FL_steer_encoder_pulse = 10000;
    float f_FR_steer_encoder_pulse = 10000;
    float f_RL_steer_encoder_pulse = 10000;
    float f_RR_steer_encoder_pulse = 10000;
    float f_FL_steer_rpm_min = 0;
    float f_FR_steer_rpm_min = 0;
    float f_RL_steer_rpm_min = 0;
    float f_RR_steer_rpm_min = 0;
    float f_FL_steer_rpm_max = 250;
    float f_FR_steer_rpm_max = 250;
    float f_RL_steer_rpm_max = 250;
    float f_RR_steer_rpm_max = 250;

    float f_steer_pid_convergence_deg = 0.02;
    float f_steer_speed_std_ms = 0.5;
    float f_steer_low_speed_pid_kp_gain = 10;
    float f_steer_low_speed_pid_ki_gain = 0;
    float f_steer_low_speed_pid_kd_gain = 0;
    float f_steer_high_speed_pid_kp_gain = 10;
    float f_steer_high_speed_pid_ki_gain = 0;
    float f_steer_high_speed_pid_kd_gain = 0;
    float f_steer_overcurrent_amp_std = 130;
    float f_steer_overcurrent_time_std = 3;
    int n_steer_control_mode = 0;
    std::string s_target_unit = "rpm";
    int n_target_pulse = 1;
};

struct UpperParameters_t {
    bool b_turntable_use = false;
    bool b_turntable_reference_position_is_initial_position = false;
    int n_turntable_reference_position = 0;
    float f_turntable_gear_ratio = 1.0;
    float f_turntable_ppr = 10000;
    float f_turntable_overcurrent_amp_std = 130;
    float f_turntable_overcurrent_time_std = 3;
};

struct EtcParameters_t {
    bool b_abs_steer_encoder_use = false;
    bool b_FL_steer_absfeedback_angled = true;
    bool b_FR_steer_absfeedback_angled = true;
    bool b_RL_steer_absfeedback_angled = true;
    bool b_RR_steer_absfeedback_angled = true;
    float f_FL_abssteer_offset = 0.0;
    float f_FR_abssteer_offset = 0.0;
    float f_RL_abssteer_offset = 0.0;
    float f_RR_abssteer_offset = 0.0;
    float f_FL_abssteer_pulse = 0.0;
    float f_FR_abssteer_pulse = 0.0;
    float f_RL_abssteer_pulse = 0.0;
    float f_RR_abssteer_pulse = 0.0;
    float f_FL_abssteer_gearratio = 1.0;
    float f_FR_abssteer_gearratio = 1.0;
    float f_RL_abssteer_gearratio = 1.0;
    float f_RR_abssteer_gearratio = 1.0;
    int n_rpm_warning_std = 100;
    int n_rpm_warning_count_std = 30;
};

enum MB5U_BAUDRATE
{
    NC_MB5U_BAUD_B4800 = 12,
    NC_MB5U_BAUD_B9600 = 13,
    NC_MB5U_BAUD_B19200 = 14,
    NC_MB5U_BAUD_B38400 = 15,
    NC_MB5U_BAUD_B115200 = 4098,
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

struct MB5UParameters_t  // MB5UParameters
{
    int n_MB5U_baudrate = MB5U_BAUDRATE::NC_MB5U_BAUD_B9600;
    std::string s_FL_MB5U_serial_num = "0000";
    std::string s_RR_MB5U_serial_num = "0000";

    // MB5U dd
    int n_dd_FL_MB5U_channel = 1;
    int n_dd_RR_MB5U_channel = 2;

    // MB5U qd
    int n_qd_FL_MB5U_channel = 1;
    int n_qd_RR_MB5U_channel = 2;

    // MB5U qd
    int n_sd_FL_MB5U_channel = 1;
};

struct SerialParameters_t  // serial
{
    std::string s_serial_tty = "/dev/ttyUSB_Serial";
    int n_serial_mode = SERIAL_MODE::SERIAL_COMODO;
    int n_serial_baudrate = SERIAL_BAUDRATE::NC_SERIAL_BAUD_57600;
    int n_serial_rec_buf = 256;
};

// interface
struct InterfaceParameters_t {
    // SIM
    bool b_sim_use = true;

    // PCAN
    bool b_pcan_use = false;

    // MB5U
    bool b_MB5U_use = false;
    MB5UParameters_t st_MB5U_param;

    // CAN
    bool b_can_use = false;

    // Serial
    bool b_serial_use = false;
    SerialParameters_t st_serial_param;
};

// driver
struct DriverParameters_t {
    int n_kinematics_type = 0;  // 0: DD, 1: QD , 2: SD, 3: OD

    WheelParameters_t st_wheel_param;
    TractionParameters_t st_traction_param;
    SteerParameters_t st_steer_param;
    UpperParameters_t st_upper_param;
    EtcParameters_t st_etc_param;

    std::string s_target_frame_id = "odom";
    std::string s_child_frame_id = "base_link";

    float f_max_linear_vel_ms = 1;
    float f_max_angular_vel_degs = 15;
    float f_linear_max_accel_mss = 0.5;
    float f_linear_max_decel_mss = 0.5;
    float f_rot_max_accdecel_degss = 30;
    float f_linear_resonance_ms = 0;
    float f_linear_resonance_filter_size_ms = 0;
    float f_heartbeat_ms = 300;
    float f_control_period_ms = 20;
    float f_encoder_alarm_reset_time = 5.0;
    float f_brake_threshold_rpm = 5;
    bool b_tf_enable = true;
    bool b_use_imu_fusion = true;
    bool b_imuodom_enable = false;
    bool b_accdecel_enable = true;
    bool b_use_brake = false;
    bool b_external_brake_control = false;
    bool b_link_brake_and_servo = false;
    float f_brake_release_time_ms = 200;
    float f_brake_lock_wait_time_ms = 2000;
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
    bool b_use_imu = false;
};

namespace MotorCommander {

template <MOTOR MotorTraitId>
struct MotorTraits;

// MotorTraits 매크로 정의
#define DEFINE_TRACTION_MOTOR_TRAITS(POSITION)                                                                        \
    template <>                                                                                                       \
    struct MotorTraits<MOTOR_TRACTION_##POSITION> {                                                                   \
        static constexpr MotorId motor_id = MOTOR_TRACTION_##POSITION;                                                \
        static constexpr bool is_traction = true;                                                                     \
        static constexpr bool is_steering = false;                                                                    \
        static constexpr bool is_abs_encoder = false;                                                                 \
        static constexpr bool is_upper_turntable = false;                                                             \
                                                                                                                      \
        static constexpr int fault_code = core_msgs::NaviAlarm::ERROR_##POSITION##_TRACTION_MOTOR_FAULT;              \
        static constexpr int timeout_code = core_msgs::NaviAlarm::ERROR_##POSITION##_TRACTION_MOTOR_FEEDBACK_TIMEOUT; \
        static constexpr int overcurrent_code = core_msgs::NaviAlarm::ERROR_##POSITION##_TRACTION_MOTOR_OVER_CURRENT; \
        static constexpr int safety_stop_code = core_msgs::NaviAlarm::ERROR_##POSITION##_TRACTION_MOTOR_SAFETY_STOP;  \
        static constexpr int stall_code = core_msgs::NaviAlarm::ERROR_##POSITION##_STEERING_MOTOR_FEEDBACK_TIMEOUT;              \
                                                                                                                      \
        static constexpr const char* fault_text = "ERROR_" #POSITION "_TRACTION_MOTOR_FAULT";                         \
        static constexpr const char* timeout_text = "ERROR_" #POSITION "_TRACTION_MOTOR_FEEDBACK_TIMEOUT";            \
        static constexpr const char* overcurrent_text = "ERROR_" #POSITION "_TRACTION_MOTOR_OVER_CURRENT";            \
        static constexpr const char* safety_stop_text = "ERROR_" #POSITION "_TRACTION_MOTOR_SAFETY_STOP";             \
        static constexpr const char* stall_text = "ERROR_" #POSITION "_TRACTION_MOTOR_STALL_TIMEOUT";                 \
                                                                                                                      \
        static inline void setError(NaviFra::Motor_ERROR& error, int code, const std::string& text)                   \
        {                                                                                                             \
            error.b_##POSITION##_error_update = true;                                                                 \
            error.n_##POSITION##_error_code = code;                                                                   \
            error.s_##POSITION##_error_text = text;                                                                   \
        }                                                                                                             \
    };

#define DEFINE_STEERING_MOTOR_TRAITS(POSITION)                                                                        \
    template <>                                                                                                       \
    struct MotorTraits<MOTOR_STEER_##POSITION> {                                                                      \
        static constexpr MotorId motor_id = MOTOR_STEER_##POSITION;                                                   \
        static constexpr bool is_traction = false;                                                                    \
        static constexpr bool is_steering = true;                                                                     \
        static constexpr bool is_abs_encoder = false;                                                                 \
        static constexpr bool is_upper_turntable = false;                                                             \
                                                                                                                      \
        static constexpr int fault_code = core_msgs::NaviAlarm::ERROR_##POSITION##_STEERING_MOTOR_FAULT;              \
        static constexpr int timeout_code = core_msgs::NaviAlarm::ERROR_##POSITION##_STEERING_MOTOR_FEEDBACK_TIMEOUT; \
        static constexpr int overcurrent_code = core_msgs::NaviAlarm::ERROR_##POSITION##_STEERING_MOTOR_OVER_CURRENT; \
        static constexpr int safety_stop_code = core_msgs::NaviAlarm::ERROR_##POSITION##_STEERING_MOTOR_SAFETY_STOP;  \
        static constexpr int stall_code = core_msgs::NaviAlarm::ERROR_##POSITION##_STEERING_MOTOR_FEEDBACK_TIMEOUT;   \
                                                                                                                      \
        static constexpr const char* fault_text = "ERROR_" #POSITION "_STEERING_MOTOR_FAULT";                         \
        static constexpr const char* timeout_text = "ERROR_" #POSITION "_STEERING_MOTOR_FEEDBACK_TIMEOUT";            \
        static constexpr const char* overcurrent_text = "ERROR_" #POSITION "_STEERING_MOTOR_OVER_CURRENT";            \
        static constexpr const char* safety_stop_text = "ERROR_" #POSITION "_STEERING_MOTOR_SAFETY_STOP";             \
        static constexpr const char* stall_text = "ERROR_" #POSITION "_STEERING_MOTOR_STALL_TIMEOUT";                 \
                                                                                                                      \
        static inline void setError(NaviFra::Motor_ERROR& error, int code, const std::string& text)                   \
        {                                                                                                             \
            error.b_##POSITION##_error_update = true;                                                                 \
            error.n_##POSITION##_error_code = code;                                                                   \
            error.s_##POSITION##_error_text = text;                                                                   \
        }                                                                                                             \
    };

#define DEFINE_ABS_ENCODER_MOTOR_TRAITS(POSITION)                                                                       \
    template <>                                                                                                         \
    struct MotorTraits<MOTOR_ABS_ENCODER_##POSITION> {                                                                  \
        static constexpr MotorId motor_id = MOTOR_ABS_ENCODER_##POSITION;                                               \
        static constexpr bool is_traction = false;                                                                      \
        static constexpr bool is_steering = false;                                                                      \
        static constexpr bool is_abs_encoder = true;                                                                    \
        static constexpr bool is_upper_turntable = false;                                                               \
                                                                                                                        \
        static constexpr int timeout_code = core_msgs::NaviAlarm::ERROR_##POSITION##_STEERING_ABSOLUTE_ENCODER_TIMEOUT; \
                                                                                                                        \
        static constexpr const char* timeout_text = "ERROR_" #POSITION "_STEERING_ABSOLUTE_ENCODER_TIMEOUT";            \
                                                                                                                        \
        static inline void setError(NaviFra::Motor_ERROR& error, int code, const std::string& text)                     \
        {                                                                                                               \
            error.b_##POSITION##_error_update = true;                                                                   \
            error.n_##POSITION##_error_code = code;                                                                     \
            error.s_##POSITION##_error_text = text;                                                                     \
        }                                                                                                               \
    };

// Traction Motors
DEFINE_TRACTION_MOTOR_TRAITS(FL)
DEFINE_TRACTION_MOTOR_TRAITS(FR)
DEFINE_TRACTION_MOTOR_TRAITS(RL)
DEFINE_TRACTION_MOTOR_TRAITS(RR)

// Steering Motors
DEFINE_STEERING_MOTOR_TRAITS(FL)
DEFINE_STEERING_MOTOR_TRAITS(FR)
DEFINE_STEERING_MOTOR_TRAITS(RL)
DEFINE_STEERING_MOTOR_TRAITS(RR)

// Absolute Encoders
DEFINE_ABS_ENCODER_MOTOR_TRAITS(FL)
DEFINE_ABS_ENCODER_MOTOR_TRAITS(FR)
DEFINE_ABS_ENCODER_MOTOR_TRAITS(RL)
DEFINE_ABS_ENCODER_MOTOR_TRAITS(RR)

// Upper - Turntable
template <>
struct MotorTraits<MOTOR_UPPER_TURNTABLE> {
    static constexpr MotorId motor_id = MOTOR_UPPER_TURNTABLE;
    static constexpr bool is_traction = false;
    static constexpr bool is_steering = false;
    static constexpr bool is_abs_encoder = false;
    static constexpr bool is_upper_turntable = true;

    static constexpr int fault_code = core_msgs::NaviAlarm::ERROR_TURNTABLE_MOTOR_FAULT;
    static constexpr int timeout_code = core_msgs::NaviAlarm::ERROR_TURNTABLE_MOTOR_TIMEOUT;
    static constexpr int overcurrent_code = core_msgs::NaviAlarm::ERROR_TURNTABLE_MOTOR_OVER_CURRENT;
    static constexpr int safety_stop_code = core_msgs::NaviAlarm::ERROR_TURNTABLE_MOTOR_SAFETY_STOP;

    static constexpr const char* fault_text = "ERROR_TURNTABLE_MOTOR_FAULT";
    static constexpr const char* timeout_text = "ERROR_TURNTABLE_MOTOR_TIMEOUT";
    static constexpr const char* overcurrent_text = "ERROR_TURNTABLE_MOTOR_OVER_CURRENT";
    static constexpr const char* safety_stop_text = "ERROR_TURNTABLE_MOTOR_SAFETY_STOP";

    // No error codes defined for TurnTable motor
    static inline void setError(NaviFra::Motor_ERROR& error, int code, const std::string& text)
    {
        error.b_UTT_error_update = true;
        error.n_UTT_error_code = code;
        error.s_UTT_error_text = text;
    }
};

}  // namespace MotorCommander
}  // namespace NaviFra
#endif
