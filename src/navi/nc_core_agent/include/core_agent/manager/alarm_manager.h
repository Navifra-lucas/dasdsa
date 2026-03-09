#ifndef ALARM_MANAGER_H
#define ALARM_MANAGER_H

#include "core_agent/redis/redis_commnder.h"

#include <Poco/JSON/Object.h>
#include <Poco/SingletonHolder.h>
#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>

#include <ctime>
#include <mutex>
#include <string>
#include <unordered_map>
#include <queue>

namespace NaviFra {

struct AlarmInfo {
    std::string category;
    int alarm_id;
    std::string description;
};


class AlarmManager {
public:
    using Ptr = Poco::SharedPtr<AlarmManager>;

    static AlarmManager& instance()
    {
        static Poco::SingletonHolder<AlarmManager> sh;
        return *sh.get();
    }

    std::string setAlarm(uint32_t alarmId, const std::string& description);
    void clearAlarm(const std::string& uuid);
    void clearAllAlarms();
    Poco::JSON::Object::Ptr getAlarmStatus(const std::string& uuid) const;
    Poco::JSON::Object::Ptr getAllAlarms() const;

    void setCommander(const std::shared_ptr<RedisCommand>& commnader) { commnader_ = commnader; }
    void loadState();

    size_t size() const { return alarms.size(); }

private:
    std::unordered_map<std::string, Poco::JSON::Object::Ptr> alarms;
    std::shared_ptr<RedisCommand> commnader_;

    std::queue<AlarmInfo> alarm_queue_;
    std::mutex alarm_mutex_; // 멀티스레드 환경 대응

    mutable std::mutex mutex_;
};

}  // namespace NaviFra

#endif  // ALARM_MANAGER_H
