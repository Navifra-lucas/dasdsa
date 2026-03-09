#include "core_agent/core_agent.h"

#include <Poco/JSON/Parser.h>
#include <core_agent/core_agent.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/manager/alarm_manager.h>
#include <core_agent/message/message_broker.h>

namespace NaviFra {

std::string AlarmManager::setAlarm(uint32_t alarmId, const std::string& description)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    for (const auto& alarm : alarms) {
        if (!alarm.second->isNull("alarmId")) {
            try {
                uint32_t existing = alarm.second->getValue<uint32_t>("alarmId");
                if (existing == alarmId) {
                    NLOG(info) << "Alarm already exists: " << alarmId;
                    return "";
                }
            }
            catch (const Poco::Exception& e) {
                NLOG(error) << "Exception while checking alarmId: " << e.displayText();
            }
        }
    }

    Poco::UUID uuid = Poco::UUIDGenerator::defaultGenerator().create();
    std::string uuidStr = uuid.toString();

    std::time_t now = std::time(nullptr);
    std::string timestamp = std::ctime(&now);
    timestamp.pop_back();  // Remove newline character
    try {
        Poco::JSON::Object::Ptr alarm = new Poco::JSON::Object;
        alarm->set("uuid", uuidStr);
        alarm->set("timestamp", timestamp);
        alarm->set("alarmId", alarmId);
        alarm->set("description", description);
        alarm->set("status", "active");

        alarms[uuidStr] = alarm;
        LOG_ERROR("Alarm %u set: %s", alarmId, description.c_str());

        // Publish to Redis
        Poco::JSON::Object redisMessage, data;
        redisMessage.set("action", "setAlarm");
        redisMessage.set("uuid", uuidStr);
        data.set("alarmId", alarmId);
        data.set("description", description);
        redisMessage.set("data", data);
        redisMessage.set("timestamp", timestamp);

        robotInfo->setAlarmDescription(description);
        robotInfo->setAlarmId(alarmId);
        std::ostringstream oss;
        redisMessage.stringify(oss);

        if (commnader_) {
            auto robot = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
            MessageBroker::instance().publish("robot.status.alarm:" + robot->getID(), oss.str());
            commnader_->hset("alarm:" + robot->getID(), uuidStr, oss.str());
        }
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what() << '\n';
    }

    return uuidStr;
}

void AlarmManager::clearAlarm(const std::string& uuid)
{
    return;
    std::lock_guard<std::mutex> lock(mutex_);
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

    auto it = alarms.find(uuid);
    if (it != alarms.end()) {
        std::time_t now = std::time(nullptr);
        std::string timestamp = std::ctime(&now);
        timestamp.pop_back();  // Remove newline character

        it->second->set("status", "cleared");
        it->second->set("cleared_timestamp", timestamp);
        LOG_INFO("Alarm %s cleared.", uuid.c_str());

        auto robot = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
        // Publish to Redis
        Poco::JSON::Object redisMessage, data;
        redisMessage.set("action", "clearAlarm");
        redisMessage.set("uuid", uuid);
        redisMessage.set("data", data);
        redisMessage.set("timestamp", timestamp);
        robotInfo->setAlarmDescription("");
        robotInfo->setAlarmId(0000);

        std::string tmp = robotInfo->getAlarmDescription();
        LOG_INFO("Alarm description: %s", tmp.c_str());

        std::ostringstream oss;
        redisMessage.stringify(oss);
        MessageBroker::instance().publish("robot.status.alarm:" + robot->getID(), oss.str());

        // Remove from Redis
        commnader_->hdel("alarm:" + robot->getID(), uuid);
        // Remove from local storage
        alarms.erase(it);
    }
    else {
        LOG_WARNING("Alarm %s not found.", uuid.c_str());
    }
}

void AlarmManager::clearAllAlarms()
{
    return;
    LOG_INFO("clearAllAlarms called");
    while (!alarms.empty()) {
        auto it = alarms.begin();
        clearAlarm(it->first);
    }
}

Poco::JSON::Object::Ptr AlarmManager::getAlarmStatus(const std::string& uuid) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = alarms.find(uuid);
    if (it != alarms.end()) {
        return it->second;
    }
    else {
        auto result = new Poco::JSON::Object;
        result->set("error", "Alarm not found");
        return result;
    }
}

Poco::JSON::Object::Ptr AlarmManager::getAllAlarms() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = new Poco::JSON::Object;
    for (const auto& alarm : alarms) {
        result->set(alarm.first, alarm.second);
    }
    return result;
}

void AlarmManager::loadState()
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto robot = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
    for (const auto& key : commnader_->getKeys("alarm:" + robot->getID())) {
        std::string alarmJson = commnader_->hget("alarm:" + robot->getID(), key);
        if (!alarmJson.empty()) {
            Poco::JSON::Parser parser;
            Poco::Dynamic::Var result = parser.parse(alarmJson);
            Poco::JSON::Object::Ptr json = result.extract<Poco::JSON::Object::Ptr>();
            std::string uuid = json->getValue<std::string>("uuid");
            alarms[uuid] = json;
        }
    }
}
}  // namespace NaviFra
