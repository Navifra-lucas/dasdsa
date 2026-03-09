/**
 * @class time checker
 * @brief measure computation time
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef TIME_CHECKER_H
#define TIME_CHECKER_H
#include "util/ansi_color.h"

#include "util/logger.hpp"

#include <limits.h>
#include <time.h>

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
using namespace std;
struct TimeRecord {
  int64_t total_time = 0;
  int64_t min_time = std::numeric_limits<int64_t>::max();
  int64_t max_time = 0;
  int64_t check_count = 0;
  std::chrono::time_point<std::chrono::steady_clock> start;
  std::chrono::time_point<std::chrono::steady_clock> end;
  bool b_record_start = false;
};

class TimeChecker {
private:
  /* data */
  std::chrono::time_point<std::chrono::steady_clock> start;
  std::chrono::time_point<std::chrono::steady_clock> end;
  bool b_use_limitation_;
  bool b_record_start_;
  std::vector<std::string> vec_record_target_name_;
  std::map<std::string, TimeRecord> map_time_record_;

  std::vector<std::chrono::microseconds> time_history_;
  std::map<std::string, unique_ptr<std::mutex>> map_mutex_time_data_;
  int64_t total_time_;
  int64_t max_time_;
  int64_t min_time_;

  int64_t GetMinTime(const string &target);
  int64_t GetMaxTime(const string &target);
  double GetAverageTime(const string &target);

public:
  // if you want to use limitation for warning print , set true
  TimeChecker(const bool b_use_limitation = false);
  ~TimeChecker();

  void MeasureStart();
  void MeasureEnd(const string message = std::string(),
                  const int limitation = INT_MAX);
  void MeasureEndMicro(const string message = std::string(),
                       const int limitation = INT_MAX);
  void RecordStart(const string &&target);
  void RecordEnd(const string &&target);
  void SaveFile();
  static string GetCurrentTime();
  float GetElapsedTime();
  float GetElapsedTimeMicro();
  friend std::ostream &operator<<(std::ostream &o, TimeChecker &fred);
};

#endif