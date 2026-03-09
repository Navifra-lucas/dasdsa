/**
 * @class service client handler
 * @brief easy to use server client
 * @author henry (haechang kim)
 * contact : henry@navifra.com
 */
#pragma once

#include "common/service_client.h"

#include <map>

namespace ANSWER {

class ServiceClientHandler {
public:
    ServiceClientHandler() = default;
    ~ServiceClientHandler() = default;

    void Suspend(std::string s_key);
    bool Request(std::string s_key);
    bool WaitResponse(std::string s_key);
    bool WaitResponseWithTimer(std::string s_key, unsigned long ul_time_limit_msec);
    void ReceiveResponse(std::string s_key);
private:
    std::map< std::string, std::shared_ptr<ServiceClient> > map_;
};

}  // namespace ANSWER