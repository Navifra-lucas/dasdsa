#pragma once

// spdlog
#include "custom_file_sink.h"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/ringbuffer_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include <signal.h>
#include <sys/types.h>
#include <unistd.h>

#include <filesystem>
#include <iostream>

#include "sstream"

#define LOG_INFO(...)                                            \
    spdlog::log(                                                 \
        spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, \
        spdlog::level::info, __VA_ARGS__)
#define LOG_ERROR(...)                                           \
    spdlog::log(                                                 \
        spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, \
        spdlog::level::err, __VA_ARGS__)
#define LOG_WARN(...)                                            \
    spdlog::log(                                                 \
        spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, \
        spdlog::level::warn, __VA_ARGS__)
#define LOG_DEBUG(...)                                           \
    spdlog::log(                                                 \
        spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, \
        spdlog::level::debug, __VA_ARGS__)
#define LOG_CRITICAL(...)                                        \
    spdlog::log(                                                 \
        spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, \
        spdlog::level::critical, __VA_ARGS__)
#define LOG_TRACE(...)                                           \
    spdlog::log(                                                 \
        spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, \
        spdlog::level::trace, __VA_ARGS__)

namespace ANSWER {
class Logger {
public:
    Logger(){};
    Logger(
        const std::string &node_name, const std::string &log_level = "info",
        const std::string &base_path = "/navifra_solution/navicore/configs/",
        bool is_default_log = true);

    ~Logger();
    // static Logger& GetInstance()
    // {
    //     static Logger instance;
    //     return instance;
    // }

    // Logger(const Logger&) = delete;
    // Logger& operator=(const Logger&) = delete;

    bool SetupLogger();
    void SetLogLevel(const std::string &log_level);
    std::string GetLogLevel();
    std::string GetLogName();

    void RecordBag(const std::vector<std::string> &bag_command);
    void StopRecording(pid_t &pid);

    void PlayBag(const std::vector<std::string> &bag_command);
    void StopPlaying(pid_t &pid);

    bool EnableConsoleLog();
    bool DisableConsoleLog();
    std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> GetRigBufferSink(
        int buffer_size = 128);

    std::string log_level_;
    std::string node_name_;
    std::string base_path_;
    std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> ringbuffer_sink_;

    bool CreateLogger(const std::string &node_name);
    void TerminateLogger();

private:
    bool is_default_log_;
    pid_t recording_pid_;
    pid_t playing_pid_;
};
}  // namespace ANSWER