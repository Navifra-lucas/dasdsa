
#ifndef NAVIFRA_SERIAL_DRIVER_HPP_
#define NAVIFRA_SERIAL_DRIVER_HPP_

#include "interface/communicator.hpp"

#include <asm/types.h>
#include <boost/any.hpp>
#include <float.h>
#include <math.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
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
#include <sstream>
#include <thread>

using namespace std;

namespace NaviFra {
struct SerialMsg {
    uint8_t* un8_serial_1_rec_buf;
    uint8_t un8_serial_1_buf_number;
};
class Serial_driver : public Communicator {
public:
    Serial_driver(){};
    ~Serial_driver(){};
    // serial::Serial ser;

    virtual void ReInitializeCheck(){};
    void InterfaceOpen();
    void uninit();
    void reset();
    void EncoderZero(string& str_data);
    void Write(const uint8_t* un8_data, size_t n_length);
    void Write(const string& str_data);
    void Read_Loop();

    virtual void SerialCallback(const boost::any& any_type_var){};

    bool Notify(const std::string& str_cbf_name, const boost::any& any_type_var);
    bool RegisteCallbackFunc(const std::string& str_cbf_name, const std::function<void(const boost::any&)>& pt_func);

    std::chrono::steady_clock::time_point tp_dd_FL_serial_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_dd_RR_serial_read_check_time = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_qd_FL_traction_serial_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_RR_traction_serial_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_FL_steer_serial_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_qd_RR_steer_serial_read_check_time = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point tp_sd_FL_traction_serial_read_check_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point tp_sd_FL_steer_serial_read_check_time = std::chrono::steady_clock::now();
};

};  // namespace NaviFra
#endif
