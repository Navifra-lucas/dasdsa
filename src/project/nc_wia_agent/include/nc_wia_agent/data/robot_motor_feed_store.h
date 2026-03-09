#ifndef NAVIFRA_ROBOT_MOTOR_FEED_STORE_H
#define NAVIFRA_ROBOT_MOTOR_FEED_STORE_H

#include <motor_msgs/MotorDataInfo.h>

#include <mutex>

namespace NaviFra {

class RobotMotorFeedStore {
public:
    static constexpr const char* KEY = "RobotMotorFeedStore";

    void setDrivingMotors(const motor_msgs::MotorDataInfo& motors)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        driving_motors_ = motors;
    }

    motor_msgs::MotorDataInfo getDrivingMotors() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return driving_motors_;  // 복사본 리턴 (thread-safe)
    }

private:
    mutable std::mutex mutex_;
    motor_msgs::MotorDataInfo driving_motors_;
};

}  // namespace NaviFra

#endif  // NAVIFRA_ROBOT_MOTOR_FEED_STORE_H
