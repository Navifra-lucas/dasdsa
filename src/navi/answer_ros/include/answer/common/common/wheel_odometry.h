#pragma once

#include "common/answer_status.h"
#include "common/pose2d.h"
#include "common/types.h"
#include "logger/logger.h"

#include <mutex>

namespace ANSWER {
namespace SLAM2D {

class WheelOdometry {
private:
    /* data */
    Pose2D current_wheel_odometry_;
    Pose2D reference_odom_frame_;
    Pose2D delta_odom_;
    Pose2D initial_reference_frame_;
    std::mutex odom_mutex_;
    bool b_first_;

public:
    WheelOdometry(/* args */)
    {
        current_wheel_odometry_ = Pose2D();
        reference_odom_frame_ = Pose2D();
        initial_reference_frame_ = Pose2D();
        delta_odom_ = Pose2D();
        b_first_ = true;
    }
    ~WheelOdometry() {}

    const Pose2D GetDeltaOdom(const Pose2D &odom);
    const Pose2D &GetOdomPose();
    void SetOdomPose(const Pose2D &odom);
    void SetInitialPose(const Pose2D &initial_pose);

    WheelOdometry(const WheelOdometry &) = delete;
    WheelOdometry &operator=(const WheelOdometry &) = delete;
    void Initialize();
    bool IsInitialized() const { return !b_first_; }

    void SetReferenceFrame(const Pose2D &initial_reference_frame);

    static WheelOdometry &GetInstance()
    {
        static WheelOdometry ins;
        return ins;
    }
};

}  // namespace SLAM2D
}  // namespace ANSWER