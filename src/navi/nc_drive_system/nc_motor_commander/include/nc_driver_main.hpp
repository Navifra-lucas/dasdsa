#ifndef NAVIFRA_DRIVER_HPP_
#define NAVIFRA_DRIVER_HPP_

#include "controllerDD.hpp"
#include "controllerOD.hpp"
#include "controllerQD.hpp"
#include "controllerSD.hpp"
#include "controller_base.hpp"
#include "core_msgs/BatteryInfo.h"
#include "core_msgs/NaviAlarm.h"
#include "driver_param.hpp"
#include "geometry_msgs/Twist.h"
#include "motor_msgs/MotorDataInfo.h"
#include "motor_msgs/MotorInfo.h"
#include "motor_sim_driver.hpp"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "util/license_check.hpp"

#include <any>

using namespace NaviFra;

class Driver {
public:
    Driver(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~Driver();

    void RegistTalker();
    void RegistListener();
    void RegistCallback();
    void Shutdown();

    void Initialize(bool b_only_param_update);
    void HeartBeatLoop();
    Cmd GetSpeedLimit(Cmd& o_new_cmd, Cmd& o_pre_cmd);
    void SetMotorAlarm(NaviFra::Motor_ERROR& o_motor_error);
    NaviFra::Motor_ERROR GetMotorAlarm();

    // -------------------------------------------------------------
    // ------------------------- Subscribe -------------------------
    // -------------------------------------------------------------
    void RecvCmdvel(const geometry_msgs::Twist::ConstPtr& msg);
    void RecvQuadCmd(const std_msgs::String::ConstPtr& msg);
    void RecvEncoderZero(const std_msgs::String::ConstPtr& msg);
    void RecvParamUpdate(const std_msgs::String::ConstPtr& msg);
    void RecvESTOP(const std_msgs::Bool::ConstPtr& msg);
    void RecvTurnCmd(const std_msgs::Float32::ConstPtr& msg);
    void RecvTurnBrake(const std_msgs::Bool::ConstPtr& msg);
    void RecvBrake(const std_msgs::Bool::ConstPtr& msg);
    void RecvControlOff(const std_msgs::Bool::ConstPtr& msg);
    void RecvSteerOpen(const std_msgs::Bool::ConstPtr& msg);
    void RecvSteeringDirection(const std_msgs::Int16::ConstPtr& msg);
    void RecvAlarmUpdate(const core_msgs::NaviAlarm::ConstPtr& msg);
    void RecvMotorDriverInit(const std_msgs::String::ConstPtr& msg);
    void RecvErrorReset(const std_msgs::String::ConstPtr& msg);
    void RecvMotorDriverInterlock(const std_msgs::Bool::ConstPtr& msg);
    void RecvBatteryInfo(const core_msgs::BatteryInfo::ConstPtr& msg);

    // -------------------------------------------------------------
    // ----------------------- Regist Callbck ----------------------
    // -------------------------------------------------------------
    void ReceivedOdom(const std::any& any_type_var);
    void ReceivedMotorInfo(const std::any& any_type_var);
    void ReceivedError(const std::any& any_type_var);
    void ReceivedImuInfo(const std::any& any_type_var);

    void SetGazeboUse(bool val);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    // ros publisher
    ros::Publisher pub_odom_;
    ros::Publisher pub_motor_info_;
    ros::Publisher pub_estop_state_;
    ros::Publisher pub_brake_cmd_;
    ros::Publisher pub_imu_info_;
    ros::Publisher pub_turntable_feed_;
    ros::Publisher pub_turntable_speed_;
    ros::Publisher pub_cmd_vel_;

    // ros subscriber
    ros::Subscriber sub_cmd_vel_;
    ros::Subscriber sub_quad_cmd_;
    ros::Subscriber sub_enc_zero_;
    ros::Subscriber sub_param_;
    ros::Subscriber sub_turn_table_cmd_;
    ros::Subscriber sub_turn_table_brake_;
    ros::Subscriber sub_brake_;
    ros::Subscriber sub_estop_;
    ros::Subscriber sub_control_off_;
    ros::Subscriber sub_steer_open_;
    ros::Subscriber sub_steering_direction_;
    ros::Subscriber sub_alarm_;
    ros::Subscriber sub_motor_driver_init_;
    ros::Subscriber sub_motor_error_reset_;
    ros::Subscriber sub_motor_driver_interlock_;
    ros::Subscriber sub_battery_info_;

    Controller* o_controller_ = nullptr;
    MotorSimDriver* o_sim_motor_driver_ = nullptr;
    DriverParam o_param_;

    InterfaceParameters_t st_interface_param_;
    DriverParameters_t st_driver_param_;

    std::thread th_heartbeat_;
    std::chrono::steady_clock::time_point checktime_hearbeat_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point checktime_turn_table_hearbeat_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point checktime_odom_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point checktime_motorinfo_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point checktime_brake_ = std::chrono::steady_clock::now();

    std::mutex mtx_cmd_;
    std::mutex mtx_info_;
    std::mutex mtx_brake_;
    std::mutex mtx_error_;

    NaviFra::Cmd o_cmd_;

    int n_kinematics_type_pre_ = 0;
    float f_linear_accel_vel_d_ = 1;
    float f_linear_decel_vel_d_ = 1;
    float f_angular_accde_vel_d_ = 50;
    float f_turn_table_vel_ = 0;
    std::string quad_str_ = "";
    NaviFra::Motor_ERROR o_motor_error_;
    bool b_error_ = false;
    bool b_param_init_ = false;
    bool b_turn_feedback_brakeon_ = true;
    bool b_motor_driver_interlock_ = false;
    bool b_estop_ = false;
    bool b_terminate_ = false;
    bool b_init_select_ = false;
    bool b_brake_on_feedback_ = false;
    bool b_charging_ = false;

    bool b_gazebo_use_ = false;

    // Mileage
    ros::Publisher pub_mileage_;
    ros::Publisher pub_mileage_local_;
    std::vector<double> last_pos_;
    std::string pose_dir_;
    int count_mileage_ = 0;
};
#endif
