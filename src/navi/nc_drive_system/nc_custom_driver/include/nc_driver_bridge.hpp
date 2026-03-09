#ifndef NAVIFRA_CUSTOM_DRIVER_HPP_
#define NAVIFRA_CUSTOM_DRIVER_HPP_

#include "util/license_check.hpp"
#include "driver_param.hpp"
#include "interface/data_struct.hpp"
#include "interface/communicator.hpp"
#include "interface/can/can_driver_amc_dd.hpp"
#include "interface/can/can_driver_curtis_sd.hpp"
#include "interface/pcan/pcan_driver_samyang_dd.hpp"
#include "interface/pcan/pcan_driver_curtis_sd.hpp"
// #include "interface/pcan/pcan_driver_curtis_3d_sd.hpp"
#include "core_msgs/NaviAlarm.h"
#include "core_msgs/MotorInfo.h"
#include "motor_msgs/MotorInfo.h"
#include "motor_msgs/MotorTargetInfo.h"
#include "motor_msgs/MotorTarget.h"
#include "motor_msgs/MotorBrake.h"
#include "motor_msgs/MotorDataInfo.h"
#include "motor_msgs/MotorData.h"
#include "motor_msgs/MotorCmd.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <boost/any.hpp>

using namespace std;
using namespace NaviFra;

class Driver {
    enum StatusWord
    {
        SW_READY_TO_SWITCH_ON = 0,
        SW_SWITCHED_ON = 1,
        SW_OPERATION_ENABLED = 2,
        SW_FAULT = 3,
        SW_VOLTAGE_ENABLED = 4,
        SW_QUICK_STOP = 5,
        SW_SWITCH_ON_DISABLED = 6,
        SW_WARNING = 7,
        SW_MANUFACTURER_SPECIFIC0 = 8,
        SW_REMOTE = 9,
        SW_TARGET_REACHED = 10,
        SW_INTERNAL_LIMIT = 11,
        SW_OPERATION_MODE_SPECIFIC0 = 12,
        SW_OPERATION_MODE_SPECIFIC1 = 13,
        SW_MANUFACTURER_SPECIFIC1 = 14,
        SW_MANUFACTURER_SPECIFIC2 = 15
    };
public:
    Driver(ros::NodeHandle& nh, ros::NodeHandle& nhp);
    ~Driver();

    void RegistTalker();
    void RegistListener();
    void Shutdown();

    void Initialize(bool b_only_param_update);
    void HandleMotorData(const boost::any& any_type_var);  // 콜백 처리
    void HeartBeatLoop();
    

    // -------------------------------------------------------------
    // ------------------------- Subscribe -------------------------
    // -------------------------------------------------------------
    void SubscribeMotorTarget(const motor_msgs::MotorTargetInfo::ConstPtr& msg);
    void SubscribeMotorBrake(const motor_msgs::MotorBrake::ConstPtr& msg);
    void SubscribeMotorWarning(const std_msgs::String::ConstPtr& msg);
    bool MotorCmdCallback(motor_msgs::MotorCmd::Request& request, motor_msgs::MotorCmd::Response& response);
    void RecvMotorDriverInit(const std_msgs::String::ConstPtr& msg);
    string GetStatusText(int n_state);

    std::vector<Communicator*> vec_communicator_;

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    // ros publisher
    ros::Publisher pub_motor_data_;
    // ros::Publisher pub_motor_info_;
    
    // ros subscriber
    ros::Subscriber sub_motor_target_;
    ros::Subscriber sub_motor_brake_;
    ros::Subscriber sub_motor_driver_init_;
    // ros::Subscriber sub_motor_warning_;
    ros::ServiceServer srv_motor_cmd_;

    // Controller* o_controller_ = nullptr;
    // MotorSimDriver* o_sim_motor_driver_ = nullptr;
    Communicator* o_communicator_ = nullptr;
    // DriverParam o_param_;

    DriverParam o_param_;
    InterfaceParameters_t st_interface_param_;
    DriverParameters_t st_driver_param_;

    std::thread th_heartbeat_;
    std::chrono::steady_clock::time_point checktime_hearbeat_ = std::chrono::steady_clock::now();

    std::mutex mtx_cmd_;
    std::mutex mtx_info_;
    std::mutex mtx_brake_;
    std::mutex mtx_error_;
    std::mutex mtx_data_;

    motor_msgs::MotorTargetInfo o_motor_target_info_;
    motor_msgs::MotorDataInfo motor_data_info_;  // 데이터 저장용
    std_msgs::String o_motor_warning_info_;  // 예상 타입

    string quad_str_ = "";
    bool b_error_ = false;
    bool b_param_init_ = false;
    bool b_init_select_ = false;

    bool b_init_ = false;
    int n_kinematics_type_pre_ = 0;

    Wheel_Cmd_t st_cmd_;
};
#endif
