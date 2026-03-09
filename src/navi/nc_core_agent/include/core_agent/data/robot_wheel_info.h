#ifndef NAVIFRA_ROBOT_WHEEL_INFO_H
#define NAVIFRA_ROBOT_WHEEL_INFO_H

#include <Poco/JSON/Object.h>
#include <core_agent/data/robot_pose.h>
#include <core_msgs/MotorInfo.h>

namespace NaviFra {
class RobotWheel {
public:
    RobotWheel(std::string id, Position position, float diameter);
    ~RobotWheel();

public:
    void updateDegree(float deg);

    Poco::JSON::Object toObject();
    std::string name() const { return id_; }
    float getDiameter() const { return diameter_; }
    float getDegree() const { return degree_; }
    Position getPosition() const { return position_; }

private:
    std::string id_;
    Position position_;
    float diameter_;
    float degree_;
};

class RobotWheelInfoStore {
public:
    RobotWheelInfoStore();
    RobotWheelInfoStore(const RobotWheelInfoStore& store);
    ~RobotWheelInfoStore();

    using Ptr = std::shared_ptr<RobotWheelInfoStore>;

    const static std::string KEY;

public:
    void updateWheelInfo(core_msgs::MotorInfo::ConstPtr msg);
    Poco::JSON::Object toObject();

    std::map<int, RobotWheel> getWheels() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return wheels_;
    }

private:
    void initDD(int mode = 1);
    void initQD(int mode = 1);
    void initSD(int mode = 1);
    void initQDOCT(int mode = 1);
    void updateDD(core_msgs::MotorInfo::ConstPtr msg);
    void updateQD(core_msgs::MotorInfo::ConstPtr msg);
    void updateSD(core_msgs::MotorInfo::ConstPtr msg);
    void updateQDOCT(core_msgs::MotorInfo::ConstPtr msg);

    struct WheelParams {
        double FL_x = 0.001, FR_x = 0.001, RL_x = 0.001, RR_x = 0.001;
        double FL_y = 0.001, FR_y = 0.001, RL_y = 0.001, RR_y = 0.001;

        bool operator!=(const WheelParams& other) const
        {
            return FL_x != other.FL_x || FR_x != other.FR_x || RL_x != other.RL_x || RR_x != other.RR_x || FL_y != other.FL_y ||
                FR_y != other.FR_y || RL_y != other.RL_y || RR_y != other.RR_y;
        }
    };

private:
    std::map<int, RobotWheel> wheels_;
    ROBOT_TYPE type_;
    WheelParams wheel_param_;
    bool b_init_ = true;
    mutable std::mutex mutex_;
};
}  // namespace NaviFra

#endif  // NC_ROBOT_INFO_H