#pragma once
#include "common/pch.h"

namespace ANSWER {
namespace MATH {

static inline float rad2degf(const int x)
{
    return x * 180.0 / M_PI;
}

static inline double rad2degd(const int x)
{
    return x * 180.0 / M_PI;
}
static inline float rad2degf(const float x)
{
    return x * 180.0 / M_PI;
}
static inline double rad2degd(const double x)
{
    return x * 180.0 / M_PI;
}
static inline float deg2radf(const float x)
{
    return x * M_PI / 180.0;
}
static inline double deg2radd(const double x)
{
    return x * M_PI / 180.0;
}
static inline Eigen::Vector3d Quaternion2RPY(const Eigen::Quaterniond &q)
{
    Eigen::Vector3d rpy;
    rpy.x() = std::atan2(
        2 * (q.w() * q.x() + q.y() * q.z()),
        1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    rpy.y() = std::asin(2 * (q.w() * q.y() - q.z() * q.x()));
    rpy.z() = std::atan2(
        2 * (q.w() * q.z() + q.x() * q.y()),
        1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    return rpy;
}
static inline Eigen::Vector3f Quaternion2RPY(const Eigen::Quaternionf &q)
{
    Eigen::Vector3f rpy;
    rpy.x() = std::atan2(
        2 * (q.w() * q.x() + q.y() * q.z()),
        1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    rpy.y() = std::asin(2 * (q.w() * q.y() - q.z() * q.x()));
    rpy.z() = std::atan2(
        2 * (q.w() * q.z() + q.x() * q.y()),
        1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    return rpy;
}
static inline Eigen::Quaternionf RPYToQuaternion(
    float roll, float pitch, float yaw)
{
    // Convert roll, pitch, and yaw from degrees to radians
    // roll *= M_PI / 180.0;
    // pitch *= M_PI / 180.0;
    // yaw *= M_PI / 180.0;

    // Create quaternion from roll, pitch, yaw using Eigen
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;

    return q;
}

static inline Eigen::Quaterniond RPYToQuaternion(
    double roll, double pitch, double yaw)
{
    // Convert roll, pitch, and yaw from degrees to radians
    // roll *= M_PI / 180.0;
    // pitch *= M_PI / 180.0;
    // yaw *= M_PI / 180.0;

    // Create quaternion from roll, pitch, yaw using Eigen
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q;
}
static inline Eigen::Matrix3d vec3ToSkewSymmetric(const Eigen::Vector3d &vec3)
{
    Eigen::Matrix3d skewSymmetric;
    skewSymmetric << 0.0, -vec3.z(), vec3.y(), vec3.z(), 0.0, -vec3.x(),
        -vec3.y(), vec3.x(), 0.0;
    return skewSymmetric;
}
static inline Eigen::Matrix3d rotationVectorToRotationMatrix(
    const Eigen::Vector3d &vec3)
{
    double phi = vec3.norm();
    Eigen::Vector3d u = vec3.normalized();
    Eigen::Matrix3d uSkew = vec3ToSkewSymmetric(u);
    Eigen::Matrix3d rotationMatrix =
        Eigen::Matrix3d::Identity() * std::cos(phi);
    rotationMatrix += uSkew * std::sin(phi);
    rotationMatrix += u * u.transpose() * (1.0 - std::cos(phi));
    return rotationMatrix;
}
static inline Eigen::Quaterniond rotationVectorToQuaternion(
    const Eigen::Vector3d &vec3)
{
    double phi_2 = vec3.norm() / 2.0;
    Eigen::Vector3d u = vec3.normalized() * std::sin(phi_2);
    Eigen::Quaterniond quaternion(std::cos(phi_2), u(0), u(1), u(2));
    return quaternion;
}
static inline float CutFloat(const float f)
{
    return std::round(f * 1000.0) / 1000.0;
}

static inline double CutDouble(const double f)
{
    return std::round(f * 1000.0) / 1000.0;
}
static inline auto NormalizeVectord(const std::vector<double> &input)
{
    const auto [min_it, max_it] =
        std::minmax_element(input.cbegin(), input.cend());
    const double min_timestamp = *min_it;
    const double max_timestamp = *max_it;

    std::vector<double> normalized(input.size());
    std::transform(
        input.cbegin(), input.cend(), normalized.begin(),
        [&](const auto &value) {
            return (value - min_timestamp) / (max_timestamp - min_timestamp);
        });
    return normalized;
}
static inline Eigen::Isometry3d ConvertVector6dToIsometry3d(
    const Eigen::VectorXd &vec6d)
{
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.pretranslate(Eigen::Vector3d(vec6d(0), vec6d(1), vec6d(2)));
    Eigen::AngleAxisd rollAngle(vec6d(3), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(vec6d(4), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(vec6d(5), Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    isometry.rotate(q);
    return isometry;
}
}  // namespace MATH
}  // namespace ANSWER
