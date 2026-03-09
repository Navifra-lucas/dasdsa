#include "Poco/Thread.h"
#include "core_agent/manager/alarm_manager.h"

#include <core_agent/core/navicore.h>
#include <core_agent/core/navicore_noetic.h>

#include <iostream>

extern bool plc_alarm_clear_;

std::unordered_map<std::string, std::vector<std::string>> getAlarmClassifications()
{
    std::unordered_map<std::string, std::vector<std::string>> classifications;

    classifications["a"] = {"ERROR_PLC_NOT_CONNECTED",    "ERROR_PLC_IO_COMMUNICATION_FAILED",    "ERROR_FRONT_PIN_MOTOR_ALARM",
                            "ERROR_REAR_PIN_MOTOR_ALARM", "ERROR_FRONT_PIN_SENSOR_BOTH_DETECTED", "ERROR_REAR_PIN_SENSOR_BOTH_DETECTED",
                            "ERROR_PIN_MISMATCH"};

    classifications["b"] = {"ERROR_TEST4", "ERROR_TEST5", "ERROR_TEST6"};

    classifications["c"] = {"ERROR_TEST7", "ERROR_TEST8", "ERROR_TEST9"};

    return classifications;
}

std::string classifyAlarm(const std::string& description)
{
    auto classifications = getAlarmClassifications();

    // description을 소문자로 변환
    std::string lowerDescription = description;
    std::transform(lowerDescription.begin(), lowerDescription.end(), lowerDescription.begin(), ::tolower);

    // 각 분류의 키워드들과 매칭
    for (const auto& [category, keywords] : classifications) {
        for (const auto& keyword : keywords) {
            std::string lowerKeyword = keyword;
            std::transform(lowerKeyword.begin(), lowerKeyword.end(), lowerKeyword.begin(), ::tolower);

            if (lowerDescription.find(lowerKeyword) != std::string::npos) {
                return category;
            }
        }
    }

    return "unknown";
}

void logClassifiedAlarm(const std::string& category, int alarmId, const std::string& description)
{
    try {
        if (category == "a") {
            NLOG(info) << "a알람 [ID:" << alarmId << "]: " << description;
            Poco::Thread::sleep(200);
            NaviFra::alarmclear();
        }
        else if (category == "b") {
            NLOG(info) << "b알람 [ID:" << alarmId << "]: " << description;
            if (plc_alarm_clear_) {
                NLOG(info) << "PLC 알람 클리어 요청됨 - 처리 중";
                Poco::Thread::sleep(200);
                NaviFra::alarmclear();
                plc_alarm_clear_ = false;
            }
            else {
                NLOG(info) << "PLC 알람 클리어 대기 중...";
            }
        }
        else if (category == "c") {
            NLOG(info) << "c알람 [ID:" << alarmId << "]: " << description;
        }

        if (alarmId > 2000) {
            auto tmp = NaviFra::AlarmManager::instance().setAlarm(alarmId, description);
            NLOG(info) << "알람 관리자에 알람 설정: ID=" << alarmId << ", Description=" << description;
        }
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what() << '\n';
    }
}
