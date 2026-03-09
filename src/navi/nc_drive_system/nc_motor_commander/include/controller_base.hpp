#ifndef DRIVER_CONTROLLER_HPP_
#define DRIVER_CONTROLLER_HPP_

#include "Error/ErrorChecker.hpp"
#include "Upper/Lift.hpp"
#include "Upper/Turntable.hpp"
#include "cmd/cmd.hpp"
#include "core/util/logger.hpp"
#include "core_calculator/core_calculator.hpp"
#include "core_msgs/MotorInfo.h"
#include "core_msgs/NaviAlarm.h"
#include "data_struct.hpp"
#include "kinematics/kinematic_calculator.hpp"
#include "motor_msgs/MotorBrake.h"
#include "motor_msgs/MotorCmd.h"
#include "motor_msgs/MotorDataInfo.h"
#include "motor_msgs/MotorInfo.h"
#include "motor_msgs/MotorTarget.h"
#include "motor_msgs/MotorTargetInfo.h"
#include "pos/pos.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"

#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <any>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
using namespace NaviFra;

namespace NaviFra {

class Controller {
public:
    Controller()
        : nhp_("~")
    {
    }

    virtual ~Controller() { finalize(); }

    void initialize(void);
    void finalize(void);

    void RegisterTalker();
    void RegisterListener();
    bool Notify(const std::string& name, const std::any& var);
    bool RegisterCallbackFunc(const std::string& name, const std::function<void(const std::any&)>& func);

    /**
     * @brief interface 통신 상태 확인
     *
     */
    virtual void InterfaceCheck(){};

    /**
     * @brief command from navigation
     *
     */
    virtual void WriteCommand(const Cmd& o_cmd){};
    virtual void EncoderZero(std::string& str_data){};
    virtual void Initialize(bool b_only_param_update){};
    virtual void SteerAlotOpen(bool b_data){};
    virtual void SpinturnSteerDirection(int n_data){};
    virtual void SetQuadCmd(std::string& str_data){};
    virtual void MotorErrorReset(std::string& str_data){};
    virtual void SetMotorInterLock(bool b_data){};

    virtual void updateMotorData(const MotorDataMap& motor_data) = 0;
    virtual void calculateOdom() = 0;

    virtual void ReceivedEncoderData(const std::any& any_type_var){};
    virtual void ReceivedMotorData(const std::any& any_type_var){};
    virtual void ReceivedImuData(const std::any& any_type_var){};
    void ReceivedError(const std::any& any_type_var);

    void SetBrake(bool b_brake_on) { b_brake_on_external_feedback_ = b_brake_on; }
    void SetInterfaceParam(const InterfaceParameters_t& st_interface_param) { st_interface_param_ = st_interface_param; }
    void SetDriverParam(const DriverParameters_t& st_driver_param)
    {
        st_driver_param_ = st_driver_param;
        SetMinMaxRPM();
    }
    void SetMotorData(MotorId motor_id, const motor_msgs::MotorData& data)
    {
        std::lock_guard<std::mutex> lock(mtx_data_);
        motor_data_[motor_id] = data;
    }
    MotorDataMap GetMotorData()
    {
        std::lock_guard<std::mutex> lock(mtx_data_);
        return motor_data_;
    }

    void CalOdomAndVel(NaviFra::SimplePos& o_d_pos, const NaviFra::SimplePos& o_vel);

    float GetHz(std::vector<std::chrono::steady_clock::time_point>& vec_time);
    float GetMS(std::vector<std::chrono::steady_clock::time_point>& vec_time);
    void RecvkMotorData(const motor_msgs::MotorDataInfo::ConstPtr& msg);
    void RecvImuData(const sensor_msgs::Imu::ConstPtr& msg);

    void SetMinMaxRPM();
    double SteerDEGtoRPM(MotorId motor_id, float f_steer_target_angle, float f_steer_feedback_angle);
    float LimitRPM(float f_target_rpm, int n_rpm_max, int n_rpm_min);

    float ConvertFromRPM(int n_type, float f_target_rpm);
    float ConvertToRPM(int n_type, float f_feedback_data);

    // NaviCAN MotorCmd Service
    void resetError() { callMotorCmd("resetError"); }
    void ControlOff(bool b_data) { callMotorCmd("disable"); }
    void EnableMotor(MotorId motor_id) { callMotorCmd("enable", motor_id); }
    void DisableMotor(MotorId motor_id) { callMotorCmd("disable", motor_id); }
    void MotorDriverInit(const std::string& str_data) { callMotorCmd("enable"); }
    void StopCommand();

    Wheel_Cmd_t st_wheel_cmd_;
    std::array<Wheel_Data_t, MOTOR_LENGTH> st_wheel_data_{};
    MotorDataMap motor_data_{};

    std::array<int, MOTOR_LENGTH> n_rpm_max{};
    std::array<int, MOTOR_LENGTH> n_rpm_min{};

    bool b_brake_on_flag_ = true;
    bool b_brake_on_feedback_ = false;
    bool b_brake_on_external_feedback_ = false;
    bool b_init_ = false;
    bool b_steer_align_ = false;
    float f_robot_angular_speed_pre_ = 0.0;

    // PID 제어기에서 사용하는 변수
    std::array<double, MOTOR_LENGTH> integrals{};
    std::array<double, MOTOR_LENGTH> previous_errors{};

    InterfaceParameters_t st_interface_param_;
    DriverParameters_t st_driver_param_;

    KinematicCalculator o_kinematic_calculator_;
    NaviFra::Pos o_odom_pos_;
    std::string quad_cmd_str_;
    std::string st_gain_data_;

    std::chrono::steady_clock::time_point brake_release_checktime_ = std::chrono::steady_clock::now();
    std::vector<std::chrono::steady_clock::time_point> vec_f_FL_traction_motor_feedback_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_FR_traction_motor_feedback_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_RL_traction_motor_feedback_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_RR_traction_motor_feedback_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_FL_steer_motor_feedback_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_FR_steer_motor_feedback_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_RL_steer_motor_feedback_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_RR_steer_motor_feedback_hz;

    std::vector<std::chrono::steady_clock::time_point> vec_f_FL_steer_abs_encoder_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_FR_steer_abs_encoder_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_RL_steer_abs_encoder_hz;
    std::vector<std::chrono::steady_clock::time_point> vec_f_RR_steer_abs_encoder_hz;
    std::mutex mtx_data_;
    std::mutex mtx_imu_data_;

    float f_target_w_degs_ = 0;
    float f_feedback_w_degs_ = 0;
    float f_feedback_vel_ms_ = 0;
    float f_feedback_acc_mss_ = 0;
    bool b_use_imu_ = false;
    float f_imu_yaw_ = 0;
    float f_imu_yaw_acc_deg_ = 0;
    int32_t pre_error_t_ = 0;
    int32_t pre_error_s_ = 0;
    
    std::chrono::system_clock::time_point tp_imu_use_ = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point tp_imu_check_ = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point tp_imu_reset_ = std::chrono::system_clock::now();
    std::vector<float> vec_accel_;

protected:
    ros::Publisher pub_rpm_;
    ros::Publisher pub_brake_;
    ros::Publisher pub_warning_;
    ros::Publisher pub_imu_reset_;
    ros::Subscriber sub_motor_data_;
    ros::Subscriber sub_imu_data_;
    ros::ServiceClient motor_cmd_req_;

    std::unique_ptr<NaviFra::MotorCommander::Upper::Turntable> upper_turntable_;
    std::unique_ptr<NaviFra::MotorCommander::Upper::Lift> upper_lift_;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    std::atomic<bool> update_running_handler_{false};
    std::atomic<bool> error_running_handler_{false};
    std::thread update_handler_thread_;
    std::thread error_handler_thread_;

    std::atomic<bool> has_error_{false};

    std::unique_ptr<NaviFra::MotorCommander::ErrorChecker> error_checker_;

    std::map<std::string, std::function<void(const std::any&)>> callbacks_;

    void startThreadUpdate();
    void stopThreadUpdate();

    void startThreadError();
    void stopThreadError();

    void callMotorCmd(const std::string& command, MotorId motor_id = MOTOR_ALL);
};
}  // namespace NaviFra
#endif
