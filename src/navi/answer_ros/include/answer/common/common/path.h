#pragma once
#include "common/exe_arguments.h"
#include "logger/logger.h"

#include <string>
namespace ANSWER {
class Path {
public:
    std::string getCurrentWorkingDirectory();
    std::string join(const std::string &path1, const std::string &path2);
    // setters
    inline void SetAllPaths(
        const std::string &ros_path,
        const std::string &specific_path = std::string())
    {
        ros_install_path_ = ros_path;
        robot_name_ = ExeArguments::GetInstance().GetArgument(
            KEY::EXE_ARGUMENTS::ROBOT_NAME);
        robot_specific_path_ =
            join(ros_install_path_, "config/robot/" + robot_name_);
        config_path_ = join(ros_install_path_, "config");
        if (!specific_path.empty()) {
            default_config_path_ = join(config_path_, specific_path);
        }
        else {
            default_config_path_ = join(config_path_, "default");
        }
        LOG_INFO("Set ROS install path: {}", ros_install_path_.c_str());
        LOG_INFO("Set Robot name: {}", robot_name_.c_str());
        LOG_INFO(
            "Set Robot specific path automatically : {}",
            robot_specific_path_.c_str());
        LOG_INFO("Set Config path automatically : {}", config_path_.c_str());
    }
    inline void SetROSInstallPath(
        const std::string &path, const bool &set_config = true)
    {
        ros_install_path_ = path;
        LOG_INFO("Set ROS install path: {}", ros_install_path_.c_str());
        if (set_config) {
            config_path_ = join(ros_install_path_, "config");
            LOG_INFO(
                "Set Config path automatically : {}", config_path_.c_str());
        }
    }
    inline void SetConfigPath(const std::string &path)
    {
        config_path_ = path;
        LOG_INFO("Set Config path: {}", config_path_.c_str());
    }
    inline void SetRobotName(
        const std::string &robot_name, const bool &set_config = true)
    {
        robot_name_ = robot_name;
        LOG_INFO("Set Robot name: {}", robot_name_.c_str());
        if (set_config) {
            robot_specific_path_ =
                join(ros_install_path_, "config/robot/" + robot_name_);
            LOG_INFO(
                "Set Robot specific path automatically : {}",
                robot_specific_path_.c_str());
        }
    }

    // getters
    const std::string GetOffsetPath() { return offset_path_; }
    const std::string GetROSInstallPath() { return ros_install_path_; }
    const std::string GetRobotSpecificPath() { return robot_specific_path_; }
    const std::string GetConfigPath() { return config_path_; }
    const std::string GetDefaultConfigPath() { return default_config_path_; }

    // singleton pattern
    static Path &GetInstance()
    {
        static Path instance;
        return instance;
    }

    std::string ros_install_path_;
    std::string config_path_;
    std::string default_config_path_;
    std::string robot_name_;
    std::string robot_specific_path_;
    std::string offset_path_;

private:
    // Path() = default;
    // ~Path() = default;

    Path();  // Private constructor
    ~Path();  // Private constructor
    Path(const Path &) = delete;
    Path &operator=(const Path &) = delete;
};
}  // namespace ANSWER