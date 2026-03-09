#include "nc_wia_agent/data/alarm_status.h"

#include <core_agent/core/navicore.h>

using namespace NaviFra;

const std::string AlarmStatus::KEY = "AlarmStatus";

Poco::JSON::Object::Ptr AlarmStatus::toObject() const
{
    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;

    // Alarm list
    Poco::JSON::Array::Ptr alarmArr = new Poco::JSON::Array;
    {
        Poco::FastMutex::ScopedLock lock(mutexAlarmList_);
        for (const auto& pair : alarmlist_) {
            Poco::JSON::Object::Ptr alarmObj = new Poco::JSON::Object;
            alarmObj->set("name", pair.first);
            alarmObj->set("timestamp", Poco::DateTimeFormatter::format(pair.second, Poco::DateTimeFormat::ISO8601_FORMAT));

            alarmArr->add(alarmObj);
        }
    }
    // Warning list
    Poco::JSON::Array::Ptr warningArr = new Poco::JSON::Array;
    {
        Poco::FastMutex::ScopedLock lock(mutexWarningList_);
        for (const auto& pair : warninglist_) {
            Poco::JSON::Object::Ptr warningObj = new Poco::JSON::Object;
            warningObj->set("name", pair.first);
            warningObj->set("timestamp", Poco::DateTimeFormatter::format(pair.second, Poco::DateTimeFormat::ISO8601_FORMAT));
            warningArr->add(warningObj);
        }
    }

    obj->set("alarms", alarmArr);
    obj->set("warnings", warningArr);

    return obj;
}

void AlarmStatus::setAlarm(uint64_t alarm_name)
{
    auto alarm_enum = SwitchWiaAlarm(alarm_name);

    if (alarm_enum == WiaAlarm::ERROR_NULL) {
        NLOG(info) << "setAlarm failed: Unmapped alarm name from enum";
        return;
    }

    Poco::FastMutex::ScopedLock lock(mutexAlarmList_);
    auto [it, inserted] = alarmlist_.try_emplace(alarm_enum, Poco::Timestamp());

    if (!inserted) {
        it->second.update();
    }
    else {
        // NLOG(info) << "setAlarm " << alarm_name;
    }
}

void AlarmStatus::setAlarm(const std::string& alarm_name)
{
    auto alarm_enum = SwitchWiaAlarm(alarm_name);

    if (alarm_enum == WiaAlarm::ERROR_NULL) {
        NLOG(info) << "setAlarm failed: Unmapped alarm name from enum";
        return;
    }

    Poco::FastMutex::ScopedLock lock(mutexAlarmList_);
    auto [it, inserted] = alarmlist_.try_emplace(alarm_enum, Poco::Timestamp());

    if (!inserted) {
        it->second.update();
    }
    else {
        // NLOG(info) << "setAlarm " << alarm_name;
    }
}

void AlarmStatus::clearSpecificAlarm(WiaAlarm alarm_name)
{
    // WiaAlarm alarm_enum = SwitchWiaAlarm(alarm_name);

    Poco::FastMutex::ScopedLock lock(mutexAlarmList_);

    if (alarmlist_.find(alarm_name) != alarmlist_.end()) {
        alarmlist_.erase(alarm_name);
        NLOG(info) << "clear alarm " << alarm_name;
    }
}

void AlarmStatus::clearAllAlarm()
{
    NLOG(info) << "clear all alarm";
    Poco::FastMutex::ScopedLock lock(mutexAlarmList_);
    alarmlist_.clear();
}

void AlarmStatus::setWarning(uint64_t warning_name)
{
    Poco::FastMutex::ScopedLock lock(mutexWarningList_);
    auto warning_enum = SwitchWiaAlarm(warning_name);
    auto [it, inserted] = warninglist_.try_emplace(warning_enum, Poco::Timestamp());

    if (!inserted) {
        it->second.update();
    }
    else {
        // NLOG(info) << "setWarning " << warning_name;
    }
}

void AlarmStatus::clearAllWarning()
{
    NLOG(info) << "clear warning alarm";
    Poco::FastMutex::ScopedLock lock(mutexWarningList_);
    if (!warninglist_.empty()) {
        warninglist_.clear();
    }
}

Alarms AlarmStatus::getAlarm()
{
    Poco::FastMutex::ScopedLock lock(mutexAlarmList_);
    return alarmlist_;
}

std::vector<uint64_t> AlarmStatus::getAlarmKeys()
{
    Poco::FastMutex::ScopedLock lock(mutexAlarmList_);
    std::vector<uint64_t> keys;
    keys.reserve(alarmlist_.size());
    std::transform(alarmlist_.begin(), alarmlist_.end(), std::back_inserter(keys), [](const auto& pair) { return pair.first; });

    return keys;
}

Warnings AlarmStatus::getWarning()
{
    Poco::FastMutex::ScopedLock lock(mutexWarningList_);
    return warninglist_;
}

std::vector<uint64_t> AlarmStatus::getWarningKeys()
{
    Poco::FastMutex::ScopedLock lock(mutexWarningList_);
    std::vector<uint64_t> keys;
    keys.reserve(warninglist_.size());
    std::transform(warninglist_.begin(), warninglist_.end(), std::back_inserter(keys), [](const auto& pair) { return pair.first; });

    return keys;
}

// 우리거 정의되어 있는 알람들만
WiaAlarm AlarmStatus::SwitchWiaAlarm(uint64_t alarm_name)
{
    static const std::map<uint64_t, WiaAlarm> alarm_map = {
        {core_msgs::NaviAlarm::ERROR_NOT_ON_PATH, WiaAlarm::ERROR_NOT_ON_PATH},
        {core_msgs::NaviAlarm::ERROR_CONFIDENCE_LOW, WiaAlarm::ERROR_CONFIDENCE_LOW},
        {core_msgs::NaviAlarm::ERROR_MAP_NOT_LOADED, WiaAlarm::ERROR_MAP_NOT_LOADED},
        {core_msgs::NaviAlarm::ERROR_MOTOR_NOTCONNECTED, WiaAlarm::ERROR_MOTOR_NOTCONNECTED},
        {core_msgs::NaviAlarm::ERROR_FRONT_LIDAR_SIGNAL_TIMEOUT, WiaAlarm::ERROR_FRONT_LIDAR_SIGNAL_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_FRONT_LIDAR_CLEAN, WiaAlarm::ERROR_FRONT_LIDAR_CLEAN},
        {core_msgs::NaviAlarm::ERROR_REAR_LIDAR_SIGNAL_TIMEOUT, WiaAlarm::ERROR_REAR_LIDAR_SIGNAL_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_REAR_LIDAR_CLEAN, WiaAlarm::ERROR_REAR_LIDAR_CLEAN},
        {core_msgs::NaviAlarm::LOCALIZATION_LARGE_CORRECTION, WiaAlarm::LOCALIZATION_LARGE_CORRECTION},
        {core_msgs::NaviAlarm::ERROR_DOCKING_HEADING_CTR_EXCEEDED, WiaAlarm::ERROR_DOCKING_HEADING_CTR_EXCEEDED},
        {core_msgs::NaviAlarm::ERROR_FRONT_CAMERA_TIMEOUT, WiaAlarm::ERROR_FRONT_CAMERA_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_REAR_CAMERA_TIMEOUT, WiaAlarm::ERROR_REAR_CAMERA_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_DOCK_NOT_FIND, WiaAlarm::ERROR_DOCK_NOT_FIND},
        {core_msgs::NaviAlarm::ERROR_LICENSE_IS_NOT_CHECKED, WiaAlarm::ERROR_LICENSE_IS_NOT_CHECKED},
        {core_msgs::NaviAlarm::ERROR_PATH_NOT_CREATED, WiaAlarm::ERROR_PATH_NOT_CREATED},
        {core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_FEEDBACK_TIMEOUT, WiaAlarm::ERROR_FL_TRACTION_MOTOR_FEEDBACK_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_ENCODER_TIMEOUT, WiaAlarm::ERROR_FL_TRACTION_MOTOR_ENCODER_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_RPM_TIMEOUT, WiaAlarm::ERROR_FL_TRACTION_MOTOR_RPM_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_OVER_CURRENT, WiaAlarm::ERROR_FL_TRACTION_MOTOR_OVER_CURRENT},
        {core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_FAULT, WiaAlarm::ERROR_FL_TRACTION_MOTOR_FAULT},
        {core_msgs::NaviAlarm::ERROR_FL_TRACTION_MOTOR_SAFETY_STOP, WiaAlarm::ERROR_FL_TRACTION_MOTOR_SAFETY_STOP},
        {core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_FEEDBACK_TIMEOUT, WiaAlarm::ERROR_RR_TRACTION_MOTOR_FEEDBACK_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_ENCODER_TIMEOUT, WiaAlarm::ERROR_RR_TRACTION_MOTOR_ENCODER_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_RPM_TIMEOUT, WiaAlarm::ERROR_RR_TRACTION_MOTOR_RPM_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_OVER_CURRENT, WiaAlarm::ERROR_RR_TRACTION_MOTOR_OVER_CURRENT},
        {core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_FAULT, WiaAlarm::ERROR_RR_TRACTION_MOTOR_FAULT},
        {core_msgs::NaviAlarm::ERROR_RR_TRACTION_MOTOR_SAFETY_STOP, WiaAlarm::ERROR_RR_TRACTION_MOTOR_SAFETY_STOP},
        {core_msgs::NaviAlarm::ERROR_BRAKE_SIGNAL_TIMEOUT, WiaAlarm::ERROR_BRAKE_SIGNAL_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_IMU_FEEDBACK_TIMEOUT, WiaAlarm::ERROR_IMU_FEEDBACK_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_SLIP_PREDETECTED, WiaAlarm::ERROR_SLIP_PREDETECTED},
        {core_msgs::NaviAlarm::ERROR_ABNORMAL_IMU, WiaAlarm::ERROR_ABNORMAL_IMU},
        {core_msgs::NaviAlarm::ERROR_DOCKING_AR_NOT_FIND, WiaAlarm::ERROR_DOCKING_AR_NOT_FIND},
        {core_msgs::NaviAlarm::ERROR_CPU_OVERLOAD, WiaAlarm::ERROR_CPU_OVERLOAD},
        {core_msgs::NaviAlarm::ERROR_MEMORY_OVERLOAD, WiaAlarm::ERROR_MEMORY_OVERLOAD},
        {core_msgs::NaviAlarm::ERROR_DISK_OVERLOAD, WiaAlarm::ERROR_DISK_OVERLOAD},
        {core_msgs::NaviAlarm::ERROR_AR_DOCK_TIMEOUT, WiaAlarm::ERROR_AR_DOCK_TIMEOUT},
        {core_msgs::NaviAlarm::ERROR_PLC_NOT_CONNECTED, WiaAlarm::ERROR_PLC_NOT_CONNECTED},
        {core_msgs::NaviAlarm::ERROR_PLC_IO_COMMUNICATION_FAILED, WiaAlarm::ERROR_PLC_IO_COMMUNICATION_FAILED},
        {core_msgs::NaviAlarm::ERROR_FRONT_PIN_MOTOR_ALARM, WiaAlarm::ERROR_FRONT_PIN_MOTOR_ALARM},
        {core_msgs::NaviAlarm::ERROR_REAR_PIN_MOTOR_ALARM, WiaAlarm::ERROR_REAR_PIN_MOTOR_ALARM},
        {core_msgs::NaviAlarm::ERROR_FRONT_PIN_SENSOR_BOTH_DETECTED, WiaAlarm::ERROR_FRONT_PIN_SENSOR_BOTH_DETECTED},
        {core_msgs::NaviAlarm::ERROR_REAR_PIN_SENSOR_BOTH_DETECTED, WiaAlarm::ERROR_REAR_PIN_SENSOR_BOTH_DETECTED},
        {core_msgs::NaviAlarm::ERROR_PIN_MISMATCH, WiaAlarm::ERROR_PIN_MISMATCH},
        {core_msgs::NaviAlarm::ERROR_PIN_NOT_ENGAGED, WiaAlarm::ERROR_PIN_NOT_ENGAGED},
        {core_msgs::NaviAlarm::ERROR_BATTERY_SOC_HIGH, WiaAlarm::ERROR_BATTERY_SOC_HIGH},
        {core_msgs::NaviAlarm::ERROR_CHARGING_FAILED, WiaAlarm::ERROR_CHARGING_FAILED},
        {core_msgs::NaviAlarm::ERROR_FRONT_BUMPER_DETECTED, WiaAlarm::ERROR_FRONT_BUMPER_DETECTED},
        {core_msgs::NaviAlarm::ERROR_REAR_BUMPER_DETECTED, WiaAlarm::ERROR_REAR_BUMPER_DETECTED},
        {core_msgs::NaviAlarm::ERROR_BMS_PACK_OVER_VOLTAGE, WiaAlarm::ERROR_BMS_PACK_OVER_VOLTAGE},
        {core_msgs::NaviAlarm::ERROR_BMS_PACK_UNDER_VOLTAGE, WiaAlarm::ERROR_BMS_PACK_UNDER_VOLTAGE},
        {core_msgs::NaviAlarm::ERROR_BMS_CELL_OVER_VOLTAGE, WiaAlarm::ERROR_BMS_CELL_OVER_VOLTAGE},
        {core_msgs::NaviAlarm::ERROR_BMS_CELL_UNDER_VOLTAGE, WiaAlarm::ERROR_BMS_CELL_UNDER_VOLTAGE},
        {core_msgs::NaviAlarm::ERROR_BMS_SOC_OVER_LIMIT, WiaAlarm::ERROR_BMS_SOC_OVER_LIMIT},
        {core_msgs::NaviAlarm::ERROR_BMS_SOC_UNDER_LIMIT, WiaAlarm::ERROR_BMS_SOC_UNDER_LIMIT},
        {core_msgs::NaviAlarm::ERROR_BMS_OVER_TEMPERATURE, WiaAlarm::ERROR_BMS_OVER_TEMPERATURE},
        {core_msgs::NaviAlarm::ERROR_BMS_UNDER_TEMPERATURE, WiaAlarm::ERROR_BMS_UNDER_TEMPERATURE},
        {core_msgs::NaviAlarm::ERROR_BMS_TEMPERATURE_DEVIATION, WiaAlarm::ERROR_BMS_TEMPERATURE_DEVIATION},
        {core_msgs::NaviAlarm::ERROR_BMS_VOLTAGE_DEVIATION, WiaAlarm::ERROR_BMS_VOLTAGE_DEVIATION},
        {core_msgs::NaviAlarm::ERROR_BMS_OVER_CURRENT, WiaAlarm::ERROR_BMS_OVER_CURRENT},
        {core_msgs::NaviAlarm::ERROR_BMS_RELAY_FAILURE, WiaAlarm::ERROR_BMS_RELAY_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_FUSE_FAILURE, WiaAlarm::ERROR_BMS_FUSE_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_RELAY_FUSING_FAILURE, WiaAlarm::ERROR_BMS_RELAY_FUSING_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_EEPROM_FAILURE, WiaAlarm::ERROR_BMS_EEPROM_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_PRECHARGE_FAILURE, WiaAlarm::ERROR_BMS_PRECHARGE_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_AFIC_FAILURE, WiaAlarm::ERROR_BMS_AFIC_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_CT_MISMATCH, WiaAlarm::ERROR_BMS_CT_MISMATCH},
        {core_msgs::NaviAlarm::ERROR_BMS_PVS_MISMATCH, WiaAlarm::ERROR_BMS_PVS_MISMATCH},
        {core_msgs::NaviAlarm::ERROR_BMS_CT_FAILURE, WiaAlarm::ERROR_BMS_CT_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_PVS_FAILURE, WiaAlarm::ERROR_BMS_PVS_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_TEMPERATURE_SENSOR_FAIL, WiaAlarm::ERROR_BMS_TEMPERATURE_SENSOR_FAIL},
        {core_msgs::NaviAlarm::ERROR_BMS_CELL_VOLTAGE_SENSOR_FAIL, WiaAlarm::ERROR_BMS_CELL_VOLTAGE_SENSOR_FAIL},
        {core_msgs::NaviAlarm::ERROR_BMS_CT_ZEROSET_FAILURE, WiaAlarm::ERROR_BMS_CT_ZEROSET_FAILURE},
        {core_msgs::NaviAlarm::ERROR_BMS_PVS_ZEROSET_FAILURE, WiaAlarm::ERROR_BMS_PVS_ZEROSET_FAILURE},
        {core_msgs::NaviAlarm::ERROR_UNABLE_ACS_DOCKING_TASK, WiaAlarm::ERROR_UNABLE_ACS_DOCKING_TASK},
        {core_msgs::NaviAlarm::ERROR_UNABLE_ACS_PATH_SPEED, WiaAlarm::ERROR_UNABLE_ACS_PATH_SPEED},
    };

    auto it = alarm_map.find(alarm_name);
    if (it != alarm_map.end())
        return it->second;
    else
        return WiaAlarm::ERROR_NULL;
}

WiaAlarm AlarmStatus::SwitchWiaAlarm(const std::string& alarm_name)
{
    static const std::map<std::string, WiaAlarm> status_map = {
        {"paused_by_obs", WiaAlarm::ERROR_PAUSED_BY_OBSTACLE},
        {"paused_by_obs_camera", WiaAlarm::ERROR_PAUSE_BY_CAMERA_OBSTACLE},
        {"paused_by_obs_v2v", WiaAlarm::ERROR_PAUSE_BY_V2V_OBSTACLE},
        {"paused_by_obs_lidar", WiaAlarm::ERROR_PAUSE_BY_LIDAR_OBSTACLE},
        {"paused_by_dock_out_t_zone_obstacle", WiaAlarm::ERROR_PAUSE_BY_T_ZONE_OBSTACLE},
        {"paused_by_obs_timeout", WiaAlarm::ERROR_PAUSE_BY_OBSTACLE_TIMEOUT},
        {"ERROR_PATH_OUT", WiaAlarm::ERROR_PATH_OUT},
    };

    auto it = status_map.find(alarm_name);
    if (it != status_map.end())
        return it->second;
    else
        return WiaAlarm::ERROR_NULL;
}

void AlarmStatus::PublishAlarm(const std::string& rid)
{
    try {
        Poco::JSON::Array::Ptr data = new Poco::JSON::Array;

        Poco::FastMutex::ScopedLock lock(mutexAlarmList_);
        auto alarmlist = alarmlist_;

        for (auto& alarm : alarmlist) {
            NLOG(info) << "alarm add !!!" << alarm.first;
            data->add(static_cast<uint64_t>(alarm.first));
        }
        int msg_id = 1;

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "error");
        response->set("data", data);
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(rid + ".ACS", oss.str());
        NLOG(info) << "Published Alarm Message: " << oss.str();  // 테스트용 로그 출력
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Alarm Exception: " << ex.displayText();
    }
}