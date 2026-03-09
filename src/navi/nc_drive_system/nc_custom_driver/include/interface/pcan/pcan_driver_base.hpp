#ifndef NAVIFRA_CUSTOM_PCAN_DRIVER_HPP_
#define NAVIFRA_CUSTOM_PCAN_DRIVER_HPP_

#include "core/util/logger.hpp"
#include "interface/communicator.hpp"
#include "core_msgs/MotorData.h"

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

#ifdef NAVIFRA_PEAK_CAN_USB_FOUND
    #include "PCANBasic.h"
namespace NaviFra {
union NODE_STATE {
    struct {
        uint8_t un8_state;
    };
    uint8_t byte_space[1];
};

struct PcanResponseManager {
    std::map<int, std::promise<TPCANMsg>> promises;  // 각 CAN ID에 대한 promise를 저장
    std::map<int, std::future<TPCANMsg>> futures;  // 각 CAN ID에 대한 future를 저장

    // 특정 CAN ID에 대한 promise와 future를 생성하는 함수
    void createPromise(int can_id)
    {
        std::promise<TPCANMsg> promise;
        promises[can_id] = std::move(promise);  // promise 저장
        futures[can_id] = promises[can_id].get_future();  // future 생성 및 저장
    }

    // 특정 CAN ID에 해당하는 promise를 완료하는 함수
    void setPromiseValue(int can_id, const TPCANMsg& frame)
    {
        if (promises.find(can_id) != promises.end()) {
            promises[can_id].set_value(frame);  // promise를 완료
        }
    }

    // 타임아웃을 지원하는 응답 대기 함수
    TPCANMsg waitForResponse(int can_id, int timeout_ms = 1000)
    {
        createPromise(can_id);  // 요청이 들어오면 promise 생성

        if (futures.find(can_id) != futures.end()) {
            auto future_status = futures[can_id].wait_for(std::chrono::milliseconds(timeout_ms));  // 타임아웃 설정
            if (future_status == std::future_status::ready) {
                TPCANMsg response = futures[can_id].get();  // 응답이 준비되면 가져옴

                // future가 완료되었으므로, promise와 future를 맵에서 제거
                promises.erase(can_id);
                futures.erase(can_id);

                return response;  // 응답 반환
            }
            else {
                throw std::runtime_error("Timeout occurred while waiting for CAN response");
            }
        }
        else {
            throw std::runtime_error("해당 CAN ID에 대한 future를 찾을 수 없음");
        }
    }
};

class Pcan_driver : public Communicator {
public:
    Pcan_driver(){LOG_INFO("pcan_driver CAN USE")};
    ~Pcan_driver(){};

    void InterfaceOpen();
    virtual void Initialize(){};
    virtual void ReInitializeCheck(){};
    void uninit();
    void reset();

    virtual void Write(const NaviFra::Wheel_Cmd_t& st_cmd){};
    void Write(int n_can_id, uint8_t* pun8_msg_data, uint8_t un8_data_len);
    void Write(int id, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7, uint8_t b8);
    void Write(TPCANMsg msg);
    void Read_Loop();
    void WriteCan(const int& id, const vector<int>& data);

    TPCANStatus safe_CAN_Write(TPCANHandle Channel, TPCANMsg* MessageBuffer);
    TPCANStatus safe_CAN_Read(TPCANHandle Channel, TPCANMsg* MessageBuffer, TPCANTimestamp* TimestampBuffer);

    virtual MotorState handleMotorControl(int motorStatus);
    virtual void PcanCallback(const boost::any& any_type_var){};

    bool Notify(const std::string& str_cbf_name, const boost::any& any_type_var);
    bool RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func);
    TPCANStatus status_;

    std::chrono::steady_clock::time_point tp_dd_FL_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_dd_RR_pcan_read_check_time = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_qd_FL_traction_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_RR_traction_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_FL_steer_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_RR_steer_pcan_read_check_time = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_sd_FL_traction_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_sd_FL_steer_pcan_read_check_time = std::chrono::steady_clock::now();
};

};  // namespace NaviFra
#else  // peak can usb 모듈이 없는 경우
    #define PCAN_MESSAGE_STANDARD 0x00
namespace NaviFra {
typedef struct {
    uint8_t ID;
    uint8_t MSGTYPE;
    uint8_t LEN;
    uint8_t DATA[8];
} TPCANMsg;

union NODE_STATE {
    struct {
        uint8_t un8_state;
    };
    uint8_t byte_space[1];
};


struct PcanResponseManager {
    std::map<int, std::promise<TPCANMsg>> promises;  // 각 CAN ID에 대한 promise를 저장
    std::map<int, std::future<TPCANMsg>> futures;  // 각 CAN ID에 대한 future를 저장

    // 특정 CAN ID에 대한 promise와 future를 생성하는 함수
    void createPromise(int can_id)
    {
        std::promise<TPCANMsg> promise;
        promises[can_id] = std::move(promise);  // promise 저장
        futures[can_id] = promises[can_id].get_future();  // future 생성 및 저장
    }

    // 특정 CAN ID에 해당하는 promise를 완료하는 함수
    void setPromiseValue(int can_id, const TPCANMsg& frame)
    {
        if (promises.find(can_id) != promises.end()) {
            promises[can_id].set_value(frame);  // promise를 완료
        }
    }

    // 타임아웃을 지원하는 응답 대기 함수
    TPCANMsg waitForResponse(int can_id, int timeout_ms = 1000)
    {
        createPromise(can_id);  // 요청이 들어오면 promise 생성

        if (futures.find(can_id) != futures.end()) {
            auto future_status = futures[can_id].wait_for(std::chrono::milliseconds(timeout_ms));  // 타임아웃 설정
            if (future_status == std::future_status::ready) {
                TPCANMsg response = futures[can_id].get();  // 응답이 준비되면 가져옴

                // future가 완료되었으므로, promise와 future를 맵에서 제거
                promises.erase(can_id);
                futures.erase(can_id);

                return response;  // 응답 반환
            }
            else {
                throw std::runtime_error("Timeout occurred while waiting for CAN response");
            }
        }
        else {
            throw std::runtime_error("해당 CAN ID에 대한 future를 찾을 수 없음");
        }
    }
};

class Pcan_driver : public Communicator {
public:
    Pcan_driver(){LOG_ERROR("pcan_driver NOT USE BUILDED!!!!")};
    ~Pcan_driver(){};

    virtual void InterfaceOpen(){};
    virtual void Initialize(){};
    virtual void ReInitializeCheck(){};
    virtual void uninit(){};
    virtual void reset(){};

    virtual void Write(const NaviFra::Wheel_Cmd_t& st_cmd){};
    virtual void Write(int n_can_id, uint8_t* pun8_msg_data, uint8_t un8_data_len){};
    virtual void Write(int id, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7, uint8_t b8){};
    virtual void Write(TPCANMsg msg){};
    virtual void Read_Loop(){};

    virtual MotorState handleMotorControl(int motorStatus);
    virtual void PcanCallback(const boost::any& any_type_var){};

    virtual bool Notify(const std::string& str_cbf_name, const boost::any& any_type_var) { return 0; };
    virtual bool RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func) { return 0; };

    std::chrono::steady_clock::time_point tp_dd_FL_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_dd_RR_pcan_read_check_time = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_qd_FL_traction_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_RR_traction_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_FL_steer_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_RR_steer_pcan_read_check_time = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_sd_FL_traction_pcan_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_sd_FL_steer_pcan_read_check_time = std::chrono::steady_clock::now();
};
};  // namespace NaviFra
#endif
#endif
