#ifndef ALARM_UTILS_H
#define ALARM_UTILS_H

#include <string>
#include <unordered_map>
#include <vector>

// 알람 분류 유틸리티 함수들
std::unordered_map<std::string, std::vector<std::string>> getAlarmClassifications();
std::string classifyAlarm(const std::string& description);
void logClassifiedAlarm(const std::string& category, int alarmId, const std::string& description);

#endif // ALARM_UTILS_H
