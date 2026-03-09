
#ifndef NAVIFRA_MOTOR_SIM_DRIVER_HPP_
#define NAVIFRA_MOTOR_SIM_DRIVER_HPP_

#include "driver_param.hpp"
#include "data_struct.hpp"
#include "motor_msgs/MotorTargetInfo.h"
#include "motor_msgs/MotorTarget.h"
#include "motor_msgs/MotorBrake.h"
#include "motor_msgs/MotorDataInfo.h"
#include "motor_msgs/MotorData.h"
#include "motor_msgs/MotorCmd.h"
#include "std_msgs/Bool.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <mutex>

namespace NaviFra {
class MotorSimDriver{
public:
    MotorSimDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~MotorSimDriver();

    void Initialize(bool b_only_param_update);
    void Start_Loop();
    void SubscribeMotorTarget(const motor_msgs::MotorTargetInfo::ConstPtr& msg);
    void SubscribeMotorBrake(const motor_msgs::MotorBrake::ConstPtr& msg);
    void SubscribeMotorError(const std_msgs::Bool::ConstPtr& msg);
    bool MotorCmdCallback(motor_msgs::MotorCmd::Request& request, motor_msgs::MotorCmd::Response& response);
    float ConvertToRPM(int n_type, float f_feedback_data);

private:
    bool b_terminate_ = false;
    bool b_init_ = false;
    bool b_error_ = false;
    Wheel_Cmd_t st_cmd_;
    Wheel_Data_t st_data_t_fl_, st_data_t_fr_, st_data_t_rl_, st_data_t_rr_, st_data_s_fl_, st_data_s_fr_, st_data_s_rl_, st_data_s_rr_;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher pub_motor_data_;
    ros::Subscriber sub_motor_target_;
    ros::Subscriber sub_motor_brake_;
    ros::Subscriber sub_motor_error_;
    ros::ServiceServer srv_motor_cmd_;

    motor_msgs::MotorTargetInfo o_motor_target_info_;
    motor_msgs::MotorBrake o_motor_brake_;
    std::mutex mtx_data_;
    std::mutex mtx_brake_;

    DriverParam o_param_;
    std::thread th_readLoop_;
    InterfaceParameters_t st_interface_param_;
    DriverParameters_t st_driver_param_;

};

};  
#endif