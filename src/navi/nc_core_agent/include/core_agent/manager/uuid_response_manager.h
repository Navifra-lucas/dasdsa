#ifndef NAVIFRA_UUID_RESPONSE_MANAGER_H
#define NAVIFRA_UUID_RESPONSE_MANAGER_H

#include <Poco/SingletonHolder.h>
#include <core/util/logger.hpp>

#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <thread>

namespace NaviFra {

class UUIDResponseManager {
public:
    static UUIDResponseManager& instance()
    {
        static Poco::SingletonHolder<UUIDResponseManager> sh;
        return *sh.get();
    }

    void registerUUID(const std::string& uuid, int timeout)
    {
        {
            waitingUUIDs_.insert(uuid);
        }

        std::thread([this, uuid, timeout]() {
            std::this_thread::sleep_for(std::chrono::seconds(timeout));
            if (waitingUUIDs_.find(uuid) != waitingUUIDs_.end()) {
                NLOG(error) << "Timeout waiting for response for UUID: " << uuid;
                waitingUUIDs_.erase(uuid);
            }
        }).detach();
    }

    bool validateUUID(const std::string& uuid)
    {
        if (waitingUUIDs_.find(uuid) != waitingUUIDs_.end()) {
            waitingUUIDs_.erase(uuid);
            NLOG(info) << "Received response for UUID: " << uuid;
            return true;
        }
        return false;
    }

private:
    std::set<std::string> waitingUUIDs_;
};

}  // namespace NaviFra

#endif  // NAVIFRA_UUID_RESPONSE_MANAGER_H
