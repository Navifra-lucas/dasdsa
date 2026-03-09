#pragma once
#include "common/pch.h"

#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>

#include <string>
namespace ANSWER {
enum class STATUS
{
    NONE = 0,
    LOCALIZATION = 1,
    LOCALIZATION_TERMINATED = 2,
    SLAM = 3,
    SLAM_TERMINATED = 4,
    LOCALIZATION3D = 5,
    LOCALIZATION3D_TERMINATED = 6,
    SLAM3D = 7,
    SLAM3D_TERMINATED = 8,
    PERCEPTION = 9,
    ERROR = 10,
};
enum class ALARM
{
    NONE = 0,
    ANSWER_LIDAR_NOT_READY = 1 << 0,
    ANSWER_ODOM_NOT_READY = 1 << 1,
    LOCALIZATION_LARGE_CORRECTION = 1 << 2,
    LOCALIZATION_LOW_CONFIDENCE = 1 << 3,
    LOCALIZATION_NOT_INITIALIZED = 1 << 4,
    LOCALIZATION_MAP_NOT_READY = 1 << 5,
    SLAM_NOT_INITIALIZED = 1 << 6,
};
class AnswerStatus {
private:
    /* data */
    STATUS status_;
    ALARM alarm_;
    int current_alarm_;
    std::mutex status_mutex_;
    std::mutex alarm_mutex_;
    std::mutex lidar_time_mutex_;
    std::mutex odom_time_mutex_;
    std::mutex map_time_mutex_;
    std::chrono::system_clock::time_point get_lidar_time_;
    std::chrono::system_clock::time_point get_odom_time_;
    std::chrono::system_clock::time_point get_map_time_;
    float time_limit_;
    std::thread alarm_check_thread_;
    bool alarm_check_thread_flag_;

    void AlarmCheckThread();

public:
    AnswerStatus(/* args */);
    ~AnswerStatus();

    AnswerStatus(const AnswerStatus &) = delete;
    AnswerStatus &operator=(const AnswerStatus &) = delete;

    static AnswerStatus &GetInstance()
    {
        static AnswerStatus instance;
        return instance;
    }
    void SetStatus(const STATUS status);
    const STATUS GetStatus();
    const std::string GetStatusString();
    void SetAlarm(const ALARM alarm);
    const int GetAlarm();
    const bool GetAlarm(ANSWER::ALARM alarm);
    const std::string GetAlarmString();
    bool ClearAllAlarm();
    bool ClearAlarm(const ALARM alarm);
    bool UpdateLidarTime();
    bool UpdateOdomTime();
    bool UpdateMapTime();
    bool TerminateAlarmCheckThread();
    bool StartAlarmCheckThread();
};
}  // namespace ANSWER