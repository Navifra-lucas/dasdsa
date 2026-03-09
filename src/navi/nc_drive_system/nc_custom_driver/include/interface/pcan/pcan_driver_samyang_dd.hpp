
#ifndef NAVIFRA_PCAN_DRIVER_SAMYANG_DD_HPP_
#define NAVIFRA_PCAN_DRIVER_SAMYANG_DD_HPP_

#include "pcan_driver_base.hpp"
#include "motor_msgs/MotorData.h"
#include "util/logger.hpp"

#include <asm/types.h>
#include <boost/any.hpp>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <mutex>
#include <thread>

namespace NaviFra {

#pragma pack(1)
union PCAN_SAMYANG_DD_RPDO_1  // 송신부
{
    struct {
        uint16_t un16_control_word;
        int8_t n8_operation_mode = 0x03;
        uint32_t un32_target_velocity = 0x00;
    };
    uint8_t byte_space[7];
};

union PCAN_SAMYANG_DD_SDO_WRITE  // 송신부
{
    struct {
        uint8_t n8_command = 0x21;
        uint16_t n16_index;
        uint8_t n8_subindex;
        int32_t n32_data;
    };
    uint8_t byte_space[8];
};

union PCAN_SAMYANG_DD_TPDO_1  // 수신부
{
    struct {
        uint16_t un16_status;
        uint8_t un8_operation;
    };
    uint8_t byte_space[8];
};

union PCAN_SAMYANG_DD_TPDO_2  // 수신부
{
    struct {
        int32_t n32_encoder;
        int32_t n32_velocity;
    };
    uint8_t byte_space[8];
};

union PCAN_SAMYANG_DD_TPDO_3  // 수신부
{
    struct {
        uint32_t un32_voltage;
        int16_t n16_current;
        uint16_t un16_error_code;
    };
    uint8_t byte_space[8];
};

union PCAN_SAMYANG_DD_SDO_READ  // 수신부
{
    struct {
        uint8_t n8_command = 0x80;
        uint16_t n16_index;
        uint8_t n8_subindex;
        int32_t n32_data;
    };
    uint8_t byte_space[8];
};

#pragma pack(8)

class Pcan_driver_samyang_dd : public Pcan_driver {
public:
    Pcan_driver_samyang_dd()
        : running_(true)
        , workerThread_(&Pcan_driver_samyang_dd::watchdog, this){};
    ~Pcan_driver_samyang_dd()
    {
        running_ = false;  // 플래그를 false로 설정하여 스레드 종료 유도
        if (workerThread_.joinable()) {
            workerThread_.join();  // 스레드가 종료될 때까지 대기
        }
    };

    void Initialize();
    void ReInitializeCheck();
    void Write(const Wheel_Cmd_t& st_cmd);
    void Stop();
    // void SetBrakePolarity();
    // void SetBrakeCommand();
    // void SetBrakeConfig();
    void PcanCallback(const boost::any& any_type_var);
    void MotorSwitchOn(int n_address, int n_type = 0);
    // void ControlBrake(bool data);
    void SetMotorGain(string& str_data);
    void SetMotorIFGain(string& str_data);
    void ControlOff(bool data);
    void DoRpdoMapping(int node_id, int transmission_type);
    void DoTpdoMapping(int node_id, int transmission_type);
    void MotorErrorReset(string& str_data);
    void MotorDriverInit(string& str_data);
    void watchdog();
    void CheckError(std::map<int16_t, motor_msgs::MotorData> motor_data);
    void Callback(std::map<int16_t, motor_msgs::MotorData> motor_data);
    PCAN_SAMYANG_DD_SDO_READ SendSDOMsg(int node_id, int command, int index, int sub_index, int data);

    virtual bool IsInitializing() final;
    // void TurnControlOff(bool data);

    /*Kerry Add*/
    // 특정 모터 데이터를 가져오기 위한 getter
    motor_msgs::MotorData getMotorData(int motor_id);
    void SetMotorData(int motor_id, const motor_msgs::MotorData& data);

    // 모터 업데이트 및 처리 로직
    void UpdateMotorData(int motor_id, const motor_msgs::MotorData& update_data, int tpdo_type);
    void ProcessNmtMessage(int motor_id, const TPCANMsg& o_canmsg);
    void ResetMotorError(int motor_id);
    void sendControlWord(ControlWord command, int motor_id);

    virtual std::map<int16_t, motor_msgs::MotorData> GetMotorData() final;
    /*Kerry Add*/

    bool b_control_off_ = false;
    bool b_init_ = false;
    bool b_brake_release_ = false;
    int n_encoder_1_ = 0;
    int n_encoder_2_ = 0;
    float f_current_1_ = 0;
    float f_current_2_ = 0;

    int n_brake_on_ = 0;
    int n_brake_off_ = 0;

    double d_encoder_1_acc_ = 0;
    double d_encoder_2_acc_ = 0;

    std::mutex mtx_lock_;

    std::chrono::system_clock::time_point tp_zeroset_ = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point tp_fl_traction_ = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point tp_rr_traction_ = std::chrono::system_clock::now();

    uint8_t un8_heartbeat_ = 0;
    TPCANMsg o_canmsg_fl_;
    TPCANMsg o_canmsg_rr_;

private:
    PcanResponseManager sdo_response_manager_, nmt_response_manager_;

    std::map<int16_t, motor_msgs::MotorData> motor_data_;  // 모터 데이터를 관리하는 벡터
    std::mutex data_mutex_;  // 멀티스레드 안전을 위한 mutex
    std::mutex func_mutex_;  // 멀티스레드 안전을 위한 mutex
    std::atomic<bool> running_;  // 스레드 종료를 위한 플래그 (스레드 간 안전성 보장)
    std::thread workerThread_;  // 스레드 멤버 변수
};

};  // namespace NaviFra
#endif
