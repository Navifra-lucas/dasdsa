
#ifndef NAVIFRA_PCAN_DRIVER_C_3D_SD_HPP_
#define NAVIFRA_PCAN_DRIVER_C_3D_SD_HPP_

#include "pcan_driver_base.hpp"
#include "util/logger.hpp"

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

namespace NaviFra {
#pragma pack(1)
union PCAN_3D_SD_WRITE  // 수신부
{
    struct {
        uint16_t un16_targetRPM;
        uint8_t un8_none;
        int16_t n16_target_angle;
        uint8_t un8_interlock;
        uint8_t un8_direction;
    };
    uint8_t byte_space[7];
};

union PCAN_3D_SD_TRACTION_READ {
    struct {
        int16_t n16_motor_rpm;  // -32768 ~ 32767
        uint16_t un16_current;  // 0 ~ 50000
        uint16_t un16_voltage;  // 0 ~ 1000
        uint8_t un8_none;  // N/A
        uint8_t un8_error;  // error message
    };
    uint8_t byte_space[8];
};

union PCAN_3D_SD_STEER_READ  // 수신부
{
    struct {
        int16_t n16_steer_angle;
        uint16_t un16_none;
        uint8_t un8_switch_status;
        int16_t n16_feedback_angle;
        uint8_t un8_error;
    };
    uint8_t byte_space[8];
};

union PCAN_3D_SD_ABSSTEER_READ  // 수신부
{
    struct {
        int16_t n16_encoder_value;
    };
    uint8_t byte_space[8];
};
#pragma pack(8)

class Pcan_driver_curtis_3d_sd : public Pcan_driver {
public:
    Pcan_driver_curtis_3d_sd(){};
    ~Pcan_driver_curtis_3d_sd(){};

    void Initialize();
    void ReInitializeCheck();
    void ReInitialize(int n_node_id);
    void Write(const Wheel_Cmd_t& st_cmd);
    void PcanCallback(const boost::any& any_type_var);
    void EncoderZero(string& str_data);

    int n_FL_traction_feedback_rpm_;

    // 특정 모터 데이터를 가져오기 위한 getter
    motor_msgs::MotorData getMotorData(int motor_id);
    void SetMotorData(int motor_id, const motor_msgs::MotorData& data);

    // 모터 업데이트 및 처리 로직
    void UpdateMotorData(int motor_id, const motor_msgs::MotorData& update_data, int tpdo_type);

private:
    std::chrono::system_clock::time_point traction_time_;
    std::chrono::system_clock::time_point steer_time_;
    std::chrono::system_clock::time_point tp_absencoder_;

    std::chrono::system_clock::time_point tp_reinit_ = std::chrono::system_clock::now();
    std::map<int16_t, motor_msgs::MotorData> motor_data_;  // 모터 데이터를 관리하는 벡터
    std::mutex data_mutex_;  // 멀티스레드 안전을 위한 mutex
};

};  // namespace NaviFra
#endif
