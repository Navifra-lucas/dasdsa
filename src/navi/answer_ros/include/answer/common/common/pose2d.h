/**
 * @class pose 2d
 * @brief 3dof pose structure containing compound and inverse operator
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#pragma once

#include "common/ceres_util/normalize_angle.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

using namespace Eigen;

namespace ANSWER {
class Pose2D {
private:
    /* data */
    Vector3f pose;  // x,y,yaw(rad)
    std::chrono::system_clock::time_point timestamp_sec =
        std::chrono::system_clock::now();

public:
    Pose2D(/* args */);
    Pose2D(const float x, const float y, const float yaw);
    Pose2D(Eigen::Vector3f rhs);
    Pose2D(Eigen::Vector2f rhs);
    ~Pose2D();

    Vector3f GetPose() const { return pose; }
    const float x() const { return this->pose.x(); }
    const float y() const { return this->pose.y(); }
    const float yaw() const { return this->pose.z(); }
    void SetPose(Vector3f in) { this->pose = in; }
    void SetPose(float &x, float &y, float &yaw)
    {
        this->pose.x() = x;
        this->pose.y() = y;
        this->pose.z() = ceres::utils::NormalizeAngle(yaw);
    }
    void SetTimeStamp(std::chrono::system_clock::time_point stamp)
    {
        this->timestamp_sec = stamp;
    }

    const std::chrono::system_clock::time_point GetTimeStamp() const
    {
        return timestamp_sec;
    }

    // inverse operator
    const Pose2D inv();
    // compound operator
    friend Pose2D operator*(
        const Pose2D &a_to_b_pose, const Pose2D &b_to_c_pose);
    friend Pose2D operator+(const Pose2D &first, const Pose2D &second);
    friend Pose2D operator-(const Pose2D &first, const Pose2D &second);
    friend bool operator==(const Pose2D &first, const Pose2D &second);
    // Pose2D& operator=(const Pose2D& ref);
    friend bool operator!=(const Pose2D &first, const Pose2D &second);
    friend std::ostream &operator<<(std::ostream &o, Pose2D const &fred);
};

}  // namespace ANSWER
