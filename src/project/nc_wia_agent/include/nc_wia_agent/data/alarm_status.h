#ifndef ALARM_STATUS_H
#define ALARM_STATUS_H

#include "alarm_list.h"
#include "core_msgs/NaviAlarm.h"

#include <Poco/JSON/Array.h>

// struct Alarm {
//     core_msgs::NaviAlarm name;
//     Poco::Timestamp timestamp;
// };

// struct Warning {
//     core_msgs::NaviAlarm name;
//     Poco::Timestamp timestamp;
// };
namespace NaviFra {

using Alarms = std::unordered_map<WiaAlarm, Poco::Timestamp>;
using Warnings = std::unordered_map<WiaAlarm, Poco::Timestamp>;

class AlarmStatus {
public:
    const static std::string KEY;

    AlarmStatus() = default;
    ~AlarmStatus() = default;

    Poco::JSON::Object::Ptr toObject() const;

    Poco::BasicEvent<void> onAlarm_;
    Poco::BasicEvent<void> onWarning_;

    void setAlarm(uint64_t alarm_name);
    void setAlarm(const std::string& alarm_name);

    Alarms getAlarm();

    void clearAllAlarm();
    void clearSpecificAlarm(WiaAlarm alarm_name);

    void setWarning(uint64_t warning_name);
    Warnings getWarning();

    void clearAllWarning();

    void PublishAlarm(const std::string& rid);

    WiaAlarm SwitchWiaAlarm(uint64_t alarm_name);
    WiaAlarm SwitchWiaAlarm(const std::string& alarm_name);

    std::string NaviAlarm2String(core_msgs::NaviAlarm alarm_no);
    core_msgs::NaviAlarm String2NaviAlarm(const std::string& str);
    bool IsStringInVolatileNaviAlarm(const std::string& str);

    std::vector<uint64_t> getAlarmKeys();
    std::vector<uint64_t> getWarningKeys();

private:
    Alarms alarmlist_;
    Warnings warninglist_;
    mutable Poco::FastMutex mutexAlarmList_;
    mutable Poco::FastMutex mutexWarningList_;
};

}  // namespace NaviFra
#endif  // ALARM_STATUS_H