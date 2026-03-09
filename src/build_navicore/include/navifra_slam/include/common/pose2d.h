/**
 * @class pose 2d
 * @brief 3dof pose structure containing compound and inverse operator
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef POSE2D_HPP
#define POSE2D_HPP

#include "common/ceres_util/normalize_angle.h"

#include <math.h>

#include <chrono>
#include <iostream>

#include "Eigen/Core"
using namespace Eigen;

namespace NaviFra {
namespace SLAM2D {
class Pose2D {
private:
    /* data */
    Vector3f pose;  // x,y,yaw(rad)
    std::chrono::steady_clock::time_point timestamp_sec = std::chrono::steady_clock::now();

public:
    Pose2D(/* args */);
    Pose2D(Vector3f rhs);
    Pose2D(Vector2f rhs);
    ~Pose2D();

    Vector3f GetPose() const { return pose; }
    void SetPose(Vector3f in) { this->pose = in; }
    void SetPose(float& x, float& y, float& yaw)
    {
        this->pose.x() = x;
        this->pose.y() = y;
        this->pose.z() = ceres::utils::NormalizeAngle(yaw);
    }
    void SetTimeStamp(std::chrono::steady_clock::time_point stamp) { this->timestamp_sec = stamp; }

    const std::chrono::steady_clock::time_point& GetTimeStamp() { return timestamp_sec; }

    // inverse operator
    Pose2D inv();
    // compound operator
    friend Pose2D operator*(const Pose2D& a_to_b_pose, const Pose2D& b_to_c_pose);
    friend Pose2D operator+(const Pose2D& first, const Pose2D& second);
    friend Pose2D operator-(const Pose2D& first, const Pose2D& second);
    friend bool operator==(const Pose2D& first, const Pose2D& second);
    friend bool operator!=(const Pose2D& first, const Pose2D& second);
    friend std::ostream& operator<<(std::ostream& o, Pose2D const& fred);
};

class Pose2DWithCov : public Pose2D {
private:
    /* data */
    // Vector3f pose; // x,y,yaw(rad)
    Eigen::Matrix3f covariance;

public:
    Pose2DWithCov(/* args */)
    {
        covariance = Eigen::Matrix3f::Zero();
        covariance.diagonal() << 0.0001, 0.0001, 0.0001;
    }
    Pose2DWithCov(Vector3f rhs)
        : Pose2D(rhs)
    {
        covariance = Eigen::Matrix3f::Zero();
        covariance.diagonal() << 0.0001, 0.0001, 0.0001;
    }
    ~Pose2DWithCov() {}
    void SetCovariance(const Eigen::Matrix3f cov) { this->covariance = cov; }
    const Matrix3f& GetCovariance() const { return this->covariance; }
    // inverse operator
    Pose2DWithCov inv();
    // compound operator
    friend Pose2DWithCov operator*(const Pose2DWithCov& a_to_b_pose, const Pose2DWithCov& b_to_c_pose);
    friend Pose2DWithCov operator+(const Pose2DWithCov& first, const Pose2DWithCov& second);
    friend Pose2DWithCov operator-(const Pose2DWithCov& first, const Pose2DWithCov& second);
    friend bool operator==(const Pose2DWithCov& first, const Pose2DWithCov& second);
    friend bool operator!=(const Pose2DWithCov& first, const Pose2DWithCov& second);
    friend std::ostream& operator<<(std::ostream& o, Pose2DWithCov const& fred);
};
}  // namespace SLAM2D
}  // namespace NaviFra

#endif