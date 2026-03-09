/*
 * @date	: Apr. 13, 2022
 * @author	: "Donghwan kwon(nate)" (cofounder@navifra.com)"
 * @warning	:
 * Copyright(C) 2022 NaviFra Coperation. All Rights are Reserved.
 */

 #ifndef DRIVER_CUSTOM_COMMUNICATION_H_
 #define DRIVER_CUSTOM_COMMUNICATION_H_
 
 #include "cmd/cmd.hpp"
 #include "motor_msgs/MotorData.h"
 #include "core_msgs/NaviAlarm.h"
 #include "motor_msgs/MotorDataInfo.h"
 #include "motor_msgs/MotorInfo.h"
 #include "motor_msgs/MotorTargetInfo.h"
 #include "motor_msgs/MotorTarget.h"
 #include "motor_msgs/MotorDataInfo.h"
 #include "motor_msgs/MotorData.h"
 #include "motor_msgs/MotorCmd.h"
 
 // #include "driver_param.hpp"
 #include "core/util/logger.hpp"
 #include "interface/data_struct.hpp"
 
 #include <boost/any.hpp>
 #include <boost/thread.hpp>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <std_msgs/String.h>
 
 #include <future>
 #include <thread>
 
 #include "ros/ros.h"
 #include <mutex>
 
 using namespace std;
 
 namespace NaviFra {
 
 
enum PDO_ID
{
    RPDO1 = 1,
    RPDO2 = 2,
    RPDO3 = 3,
    RPDO4 = 4,
    TPDO1 = 5,
    TPDO2 = 6,
    TPDO3 = 7,
    TPDO4 = 8,
};

enum MotorID
{
    FL_TRACTION = 0,
    FR_TRACTION = 1,
    RL_TRACTION = 2,
    RR_TRACTION = 3,
    FL_STEER = 4,
    FR_STEER = 5,
    RL_STEER = 6,
    RR_STEER = 7,
    FL_ABS_STEER = 8
};

enum CanID
{
    CAN_RPDO1_ID = 0x200,
    CAN_RPDO2_ID = 0x300,
    CAN_RPDO3_ID = 0x400,
    CAN_RPDO4_ID = 0x500,
    CAN_TPDO1_ID = 0x180,
    CAN_TPDO2_ID = 0x280,
    CAN_TPDO3_ID = 0x380,
    CAN_TPDO4_ID = 0x480,
    CAN_SDO_REQUEST_ID = 0x600,
    CAN_SDO_RESPONSE_ID = 0x580,
    CAN_NMT_STATUS_ID = 0x700
};

enum MotorStatus
{
    STATUS_NOT_READY = 0x00,  // Not Ready to Switch On
    STATUS_SWITCH_ON_DISABLED = 0x40,  // Switch On Disabled
    STATUS_READY_SWITCH_ON = 0x1,  // Ready to Switch On
    STATUS_SWITCHED_ON = 0x3,  // Switched On
    STATUS_OP_ENABLED = 0x7,  // Operation Enabled
    STATUS_FAULT = 0x08,  // Fault
};
// 상태 정의를 위한 enum
enum ControlWord
{
    CONTROL_WORD_SHUTDOWN = 0x06,  // Shutdown: 0xxx x110, 비트 2,6,8이 활성화된 상태
    CONTROL_WORD_SWITCH_ON = 0x07,  // Switch On: 0xxx x111, 비트 3이 활성화된 상태
    CONTROL_WORD_DISABLE_VOLTAGE = 0x00,  // Disable Voltage: 0xxx xx0x, 비트 7,9,10,12가 비활성화된 상태
    CONTROL_WORD_QUICK_STOP = 0x02,  // Quick Stop: 0xxx x01x, 비트 7,10,11이 활성화된 상태
    CONTROL_WORD_DISABLE_OPERATION = 0x07,  // Disable Operation: 0xxx 0111, 비트 5가 활성화된 상태
    CONTROL_WORD_ENABLE_OPERATION = 0x0F,  // Enable Operation: 0xxx 1111, 비트 14,16이 활성화된 상태
    CONTROL_WORD_FAULT_RESET = 0x80  // Fault Reset: 0xxx xxxx에서 1xxx xxxx로 전환, 비트 14, 15가 활성화된 상태
};

enum NMTCommand
{
    RESET_NODE = 0x81,
    RESET_COMMUNICATION = 0x82,
    PRE_OPERATIONAL = 0x80,
    STOP = 0x02,
    OPERATIONAL = 0x01,
};

enum MotorState
{
    STATE_SWITCH_ON_DISABLE,
    STATE_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON,
    STATE_OPERATION_ENABLE,
    STATE_QUICK_STOP_ACTIVE,
    STATE_FAULT_REACTION_ACTIVE,
    STATE_RESET_MOTOR,
    STATE_FAULT,
    STATE_COMPLETE,
    STATE_TIMEOUT
};
  
 class Communicator {
 public:
    Communicator(){};
    ~Communicator();
 
    virtual void Initialize(){};
    virtual void InterfaceOpen(){};
    virtual void ReInitializeCheck(){};
    virtual void Write(const Wheel_Cmd_t& st_cmd){};
    virtual void Write2(const Cmd& o_cmd){};
    virtual void Start_Loop(){};
    virtual void Read_Loop(){};
    virtual void EncoderZero(string& str_data){};
    virtual void NaviStatus(string& str_data){};
    virtual void SetMotorGain(string& str_data){};
    virtual void SetMotorIFGain(string& str_data){};
    virtual void Stop(){};
    virtual void ControlOff(bool data){};
    virtual void TurnControlOff(bool data){};
    virtual void SteerAlotOpen(bool data){};
    virtual void MotorDriverInit(string& str_data){};
    virtual void MotorErrorReset(string& str_data){};
    virtual void SetMotorInterLock(bool data){};
    virtual void sendControlWord(ControlWord command, int motor_id){};
    virtual void WriteCan(const int& id, const vector<int>& data){};
    void SetBrakeOn(bool b_data){b_brake_on_ = b_data;};
    void SetServoOn(bool b_data){b_servo_on_ = b_data;};
 
    void SetBrake(bool data) { b_brake_on_feedback_ = data; };
    void SetInterfaceParam(const InterfaceParameters_t& st_interface_param);
    void SetDriverParam(const DriverParameters_t& st_driver_param);
    void Terminate();  // 추가
     
    bool Notify(const std::string& str_cbf_name, const boost::any& any_type_var);
    bool RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func);
    virtual std::map<int16_t, motor_msgs::MotorData> GetMotorData() { return std::map<int16_t, motor_msgs::MotorData>(); }
 
    std::thread th_readLoop_;
    std::mutex rw_mtx;
    bool b_terminate_ = false;
    NaviFra::InterfaceParameters_t st_interface_param_;
    NaviFra::DriverParameters_t st_driver_param_;
    bool b_brake_on_feedback_;
    bool b_brake_on_ = false;
    bool b_servo_on_ = true;
 
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
 
    std::map<std::string, std::function<void(const boost::any&)>> map_callback_pt_;
 };
 }  // namespace NaviFra
 #endif
 