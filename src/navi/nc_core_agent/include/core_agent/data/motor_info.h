#ifndef NAVIFRA_MOTOR_INFO_H
#define NAVIFRA_MOTOR_INFO_H

#include <Poco/JSON/Object.h>
#include <core_agent/data/types.h>
#include <core_msgs/MotorInfo.h>

#include <array>
#include <map>
#include <vector>

namespace NaviFra {
class CMotorInfo {
public:
    CMotorInfo();
    CMotorInfo(core_msgs::MotorInfo::ConstPtr msg);
    ~CMotorInfo();

public:
    const auto getMotors() { return motors_; }
    const auto getTimeStamps() { return timeStamps_; }

private:
    std::map<MOTOR, std::array<float, MOTOR_INFO_LENGTH>> motors_;
    std::map<MOTOR, Poco::Timestamp> timeStamps_;
};

class MotorInfoStore {
public:
    MotorInfoStore();
    ~MotorInfoStore(){};

    const static std::string KEY;

public:
    void append(core_msgs::MotorInfo::ConstPtr msg);
    std::string toString(std::string id);
    Poco::JSON::Object::Ptr toObject();

    void clear();
    size_t size() { return motor_infos_.size(); }

private:
    Poco::JSON::Object aggregateMotorInfo(MOTOR motor);

private:
    std::vector<CMotorInfo> motor_infos_;
    std::array<std::string, MOTOR_LENGTH> motor_name_;
};

}  // namespace NaviFra

#endif