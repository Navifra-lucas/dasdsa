#ifndef NAVIFRA_ROBOT_STATUS_H
#define NAVIFRA_ROBOT_STATUS_H

#include <memory>
#include <iostream>
#include <fstream>
#include <cstdlib>  // getenv

namespace NaviFra {
class RobotStatus {
public:
    RobotStatus();
    ~RobotStatus();

    using Ptr = std::shared_ptr<RobotStatus>;

    const static std::string KEY;

private:
    virtual std::string implName() final { return "RobotStatus"; }

public:
    std::string getID();
    void setID(std::string id);
    std::string getUUID();
    std::string getToken();
    void setToken(std::string token);
    bool isEnable();
    void setEnable(bool enable);
    bool isJobActive();
    void setActive(bool active);
    void setSLAM(bool bOnOff);
    bool isSLAM();
    std::string getVersion();

private:
    std::string uuid_;
    std::string id_;
    std::string token_;
    bool is_enabled_;
    bool job_is_active_;
    bool is_slam_;
};
}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_STATUS_H