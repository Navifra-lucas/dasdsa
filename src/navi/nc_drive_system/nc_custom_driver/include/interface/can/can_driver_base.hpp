#ifndef NAVIFRA_CUSTOM_CAN_DRIVER_HPP_
#define NAVIFRA_CUSTOM_CAN_DRIVER_HPP_

#include "interface/communicator.hpp"

#include <asm/types.h>
#include <boost/any.hpp>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>

using namespace std;

namespace NaviFra {

struct CanResponseManager {
    std::map<int, std::promise<can_frame>> promises;  // 각 CAN ID에 대한 promise를 저장
    std::map<int, std::future<can_frame>> futures;  // 각 CAN ID에 대한 future를 저장

    // 특정 CAN ID에 대한 promise와 future를 생성하는 함수
    void createPromise(int can_id)
    {
        std::promise<can_frame> promise;
        promises[can_id] = std::move(promise);  // promise 저장
        futures[can_id] = promises[can_id].get_future();  // future 생성 및 저장
    }

    // 특정 CAN ID에 해당하는 promise를 완료하는 함수
    void setPromiseValue(int can_id, const can_frame& frame)
    {
        if (promises.find(can_id) != promises.end()) {
            promises[can_id].set_value(frame);  // promise를 완료
        }
    }

    // 타임아웃을 지원하는 응답 대기 함수
    can_frame waitForResponse(int can_id, int timeout_ms = 1000)
    {
        createPromise(can_id);  // 요청이 들어오면 promise 생성

        if (futures.find(can_id) != futures.end()) {
            auto future_status = futures[can_id].wait_for(std::chrono::milliseconds(timeout_ms));  // 타임아웃 설정
            if (future_status == std::future_status::ready) {
                can_frame response = futures[can_id].get();  // 응답이 준비되면 가져옴

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
class Can_driver : public Communicator {
public:
    Can_driver(){};
    ~Can_driver(){};

    struct sockaddr_can can_addr;
    struct ifreq can_ifr;

    void InterfaceOpen();
    void Init();
    void uninit();
    void Write(int n_can_id, uint8_t* un8_send_data, uint8_t un8_data_length);
    void Write(can_frame msg);
    void Write(
        int n_can_id, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6, uint8_t byte7,
        uint8_t byte8);
    void Read_Loop();
    virtual MotorState handleMotorControl(int motorStatus);
    void WriteCan(const int& id, const vector<int>& data);
    string Can_error_check(unsigned int error_code);
    bool Notify(const std::string& str_cbf_name, const boost::any& any_type_var);
    bool RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func);

    ssize_t safe_CAN_Read(int n_can_fd, can_frame* Message);
    ssize_t safe_CAN_Write(int n_can_fd, can_frame* msg); 

    int n_nbytes_read_;
    int n_can_fd_;
    ssize_t n_bytes_write_;
};

};  // namespace NaviFra
#endif