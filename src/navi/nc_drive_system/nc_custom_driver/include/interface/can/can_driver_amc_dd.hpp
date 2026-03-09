
#ifndef NAVIFRA_CAN_DRIVER_AMC_DD_HPP_
#define NAVIFRA_CAN_DRIVER_AMC_DD_HPP_

#include "interface/can/can_driver_base.hpp"
#include "motor_msgs/MotorDataInfo.h"
#include "motor_msgs/MotorData.h"

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

#define CAN_LINEAR_ACCERATION 0x33
#define CAN_ANGULAR_VELOCITY 0x34
#define CAN_EULER_ANGLE 0x35
#define FAULT_PARAM_RESTORE_ERROR        0x0001  // Bit 0: Parameter Restore Error
#define FAULT_STORE_ERROR                0x0002  // Bit 1: Parameter Store Error
#define FAULT_INVALID_HALL_STATE         0x0004  // Bit 2: Invalid Hall State
#define FAULT_PHASE_SYNC_ERROR           0x0008  // Bit 3: Phase Sync Error
#define FAULT_MOTOR_OVER_TEMP            0x0010  // Bit 4: Motor Over Temperature
#define FAULT_PHASE_DETECT_ERROR         0x0020  // Bit 5: Phase Detection Fault
#define FAULT_FEEDBACK_SENSOR_ERROR      0x0040  // Bit 6: Feedback Sensor Error
#define FAULT_MOTOR_OVER_SPEED           0x0080  // Bit 7: Motor Over Speed
#define FAULT_MAX_POSITION_EXCEEDED      0x0100  // Bit 8: Max Measured Position
#define FAULT_MIN_POSITION_EXCEEDED      0x0200  // Bit 9: Min Measured Position
#define FAULT_COMM_ERROR_NODE_GUARDING   0x0400  // Bit 10: Comm. Error (Node Guarding)
#define FAULT_PWM_INPUT_BROKEN           0x0800  // Bit 11: PWM Input Broken Wire
#define FAULT_MOTION_ENGINE_ERROR        0x1000  // Bit 12: Motion Engine Error
#define FAULT_MOTION_ENGINE_ABORT        0x2000  // Bit 13: Motion Engine Abort
#define FAULT_SHUNT_REGULATOR_ACTIVE     0x4000  // Bit 14: Shunt Regulator Active (상태지만 감지 가능)
#define FAULT_PHASE_DETECT_DONE          0x8000  // Bit 15: Phase Detect Done (보통 상태지만 활용 가능)


namespace NaviFra {
#pragma pack(1)

union CAN_AMC_DD_WRITE_1  // 송신부
{
    struct {
        uint16_t un16_control_word;
        int8_t n8_operation_mode = 3;
    };
    uint8_t byte_space[3];
};

union CAN_AMC_DD_WRITE_2  // 송신부
{
    struct {
        int32_t n32_target_velocity;
    };
    uint8_t byte_space[4];
};

union CAN_AMC_DD_SDO_WRITE  // 송신부
{
    struct {
        uint8_t n8_command = 0x21;
        uint16_t n16_index;
        uint8_t n8_subindex;
        int32_t n32_data;
    };
    uint8_t byte_space[8];
};

union CAN_AMC_DD_READ_1  // 수신부
{
    struct {
        uint16_t un16_status;
        uint16_t un16_error_code;
        int32_t n32_traction_amps;
    };
    uint8_t byte_space[8];
};

union CAN_AMC_DD_READ_2  // 수신부
{
    struct {
        int32_t n32_encoder;
    };
    uint8_t byte_space[8];
};

union CAN_AMC_DD_READ_3  // 수신부
{
    struct {
        int32_t n32_velocity;
    };
    uint8_t byte_space[8];
};

union CAN_AMC_DD_SDO_READ  // 수신부
{
    struct {
        uint8_t n8_command = 0x60;
        uint16_t n16_index;
        uint8_t n8_subindex;
        int32_t n32_data;
    };
    uint8_t byte_space[8];
};

#pragma pack(8)

class Can_driver_amc_dd : public Can_driver {
public:
    Can_driver_amc_dd()
        : running_(true)
        , workerThread_(&Can_driver_amc_dd::watchdog, this){};
    ~Can_driver_amc_dd()
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
    void CanCallback(const boost::any& any_type_var);
    void MotorSwitchOn(int n_address, int n_type = 0);
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
    CAN_AMC_DD_SDO_READ SendSDOMsg(int node_id, int command, int index, int sub_index, int data);

    virtual bool IsInitializing() final;

    /*Kerry Add*/
    // 특정 모터 데이터를 가져오기 위한 getter
    motor_msgs::MotorData getMotorData(int motor_id);
    void SetMotorData(int motor_id, const motor_msgs::MotorData& data);

    // 모터 업데이트 및 처리 로직
    void UpdateMotorData(int motor_id, const motor_msgs::MotorData& update_data, int tpdo_type);
    void ProcessNmtMessage(int motor_id, const can_frame& o_canmsg);
    void ResetMotorError(int motor_id);
    void sendControlWord(ControlWord command, int motor_id);

    virtual std::map<int16_t, motor_msgs::MotorData> GetMotorData() final;
    /*Kerry Add*/

    motor_msgs::MotorDataInfo motor_data_info_;  // 클래스 멤버로 이동
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
    can_frame o_canmsg_fl_;
    can_frame o_canmsg_rr_;


    // 남은 변수 사용하는지 판단 후 제거 필요
    int n_encoder_f_ = 0;
    int n_encoder_r_ = 0;
    float f_FL_current_ = 0;
    float f_RR_current_ = 0;

    double d_FL_gain_p_ = 0.3;
    double d_RR_gain_p_ = 0.3;
    double d_FL_gain_i_ = 0.12;
    double d_RR_gain_i_ = 0.12;
    double d_FL_gain_d_ = 0.0;
    double d_RR_gain_d_ = 0.0;

    double d_FL_inertia_ = 0.0;
    double d_RR_inertia_ = 0.0;
    double d_FL_friction_ = 0.0;
    double d_RR_friction_ = 0.0;

    int32_t n32_encoder_f_ = 0;
    int32_t n32_encoder_r_ = 0;
    double d_encoder_f_acc_ = 0;
    double d_encoder_r_acc_ = 0;

    std::chrono::steady_clock::time_point tp_f_encoder_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_r_encoder_ = std::chrono::steady_clock::now();

    // std::chrono::steady_clock::time_point tp_turntable_ = std::chrono::steady_clock::now();
    // std::chrono::steady_clock::time_point tp_encoder_ = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_initial_ = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_can_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_fault_ = std::chrono::steady_clock::now();
    int n_fault_count_ = 0;

    std::chrono::steady_clock::time_point tp_FL_current_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_RR_current_ = std::chrono::steady_clock::now();

    std::vector<float> vec_f_FL_vel_;
    std::vector<float> vec_f_RR_vel_;

private:
    CanResponseManager sdo_response_manager_, nmt_response_manager_;

    std::map<int16_t, motor_msgs::MotorData> motor_data_;  // 모터 데이터를 관리하는 벡터
    std::mutex data_mutex_;  // 멀티스레드 안전을 위한 mutex
    std::mutex func_mutex_;  // 멀티스레드 안전을 위한 mutex
    std::atomic<bool> running_;  // 스레드 종료를 위한 플래그 (스레드 간 안전성 보장)
    std::thread workerThread_;  // 스레드 멤버 변수
};

};  // namespace NaviFra
#endif
