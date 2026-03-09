#include "core_agent/core_agent.h"

#include <core_agent/data/robot_status.h>

using namespace NaviFra;

const std::string RobotStatus::KEY = "RobotStatus";

RobotStatus::RobotStatus()
{
    uuid_ = "";
    id_ = "0";
    token_ = "";
    is_enabled_ = false;
    job_is_active_ = false;
    is_slam_ = false;
}

RobotStatus::~RobotStatus()
{
}

std::string RobotStatus::getID()
{
    return id_;
}
void RobotStatus::setID(std::string id)
{
    id_ = id;
}

std::string RobotStatus::getUUID()
{
    return uuid_;
}
std::string RobotStatus::getToken()
{
    return token_;
}

void RobotStatus::setToken(std::string token)
{
    token_ = token;
}

bool RobotStatus::isEnable()
{
    return is_enabled_;
}

void RobotStatus::setEnable(bool enable)
{
    is_enabled_ = enable;
}

bool RobotStatus::isJobActive()
{
    return job_is_active_;
}

void RobotStatus::setActive(bool active)
{
    job_is_active_ = active;
}

bool RobotStatus::isSLAM()
{
    return is_slam_;
}

void RobotStatus::setSLAM(bool bOnOff)
{
    is_slam_ = bOnOff;
}

std::string RobotStatus::getVersion()
{
    static std::string version_ = "";
    if (!version_.empty()) {
        return version_;
    }
    std::string home_dir = getenv("HOME");
    std::string file_path = home_dir + "/navifra_solution/navicore/install/share/build_version.txt";

    std::string version;
    std::ifstream file(file_path);
    if (file.is_open()) {
        std::getline(file, version);
        file.close();
        std::cout << "Build version: " << version << std::endl;
    } else {
        std::cerr << "Failed to open file: " << file_path << std::endl;
    }
    version_ = version;
    return version;
}