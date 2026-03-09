/**
 * @class service client
 * @brief lock response
 * @author henry (haechang kim)
 * contact : henry@navifra.com
 */
#pragma once

#include <chrono>
#include <string>
#include <condition_variable>
#include <mutex>

namespace ANSWER {

class ServiceClient {
public:
    ServiceClient(std::string str_name);
    ~ServiceClient();
    void Suspend();
    bool Request();
    bool WaitResponse();
    bool WaitResponseWithTimer(unsigned long ul_time_limit_msec);
    void ReceiveResponse();
private:
    std::string str_name_;
    bool b_request_ = false;
    bool b_wait_ = false;
    bool b_received_response_ = false;
    bool b_suspend_ = false;
    std::condition_variable cv_;
    std::mutex mtx_;
};

}  // namespace ANSWER