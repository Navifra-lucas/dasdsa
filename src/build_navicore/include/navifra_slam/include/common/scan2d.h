/**
 * @class scan 2d
 * @brief 2d lidar structure
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef SCAN_2D_HPP
#define SCAN_2D_HPP

#include "common/pose2d.h"
// #include "common/slam_utils.h"
#include "glog/logging.h"
#include "util/logger.hpp"

#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
using namespace Eigen;
using namespace std;
using Point2D = Eigen::Vector2f;
using PointCloud2D = vector<Point2D>;

namespace NaviFra {
namespace SLAM2D {
struct Extrinsic {
    // xyz
    Vector3f position = Vector3f::Zero();
    // rpy
    Vector3f orientation = Vector3f::Zero();
};
class Scan2D {
private:
    /* data */
public:
    Scan2D(/* args */);
    ~Scan2D();
    // pointcloud
    Eigen::Vector3f origin;
    // Pose2D origin;
    // vector<Pose2D> point_cloud;
    PointCloud2D point_cloud;
    // raw measurements
    vector<float> ranges;
    vector<float> intensities;
    uint32_t secs;
    uint32_t nsecs;
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    Extrinsic extrinsic;

    void ConvertRangeDataToPointCloud();
    bool ConvertROSMessageToCustomStructure(
        const uint32_t sec, const uint32_t nsec, const float& angle_increment, const float& angle_min, const float& angle_max,
        const float& range_min, const float& range_max, const std::vector<float>& ranges, const std::vector<float>& intensities);

    void SetExtrinsicParameter(
        const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw);
};
}  // namespace SLAM2D
}  // namespace NaviFra

#endif