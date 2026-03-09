/**
 * @class scan 2d
 * @brief 2d lidar structure
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef SCAN_2D_HPP
#define SCAN_2D_HPP

// #include "common/pch.h"
#include "common/pose2d.h"
#include "common/types.h"
#include "glog/logging.h"

using namespace Eigen;
using namespace std;

namespace ANSWER {
struct Extrinsic {
    // xyz
    Vector3f translation = Vector3f::Zero();
    // rpy
    Vector3f orientation = Vector3f::Zero();
    Matrix3f rotation_matrix = Matrix3f::Identity();
};
class Scan2D {
private:
    /* data */
public:
    Scan2D(/* args */);
    ~Scan2D();
    template <typename T>
    Scan2D(const std::vector<Eigen::Matrix<T, 3, 1>> &scan3d)
        : secs(0.)
        , nsecs(0.)
        , angle_min(0.)
        , angle_max(0.)
        , angle_increment(0.)
        , range_min(0.)
        , range_max(0.)
    {
        point_cloud.clear();
        ranges.clear();
        intensities.clear();
        is_extrinsic_set = false;
        point_cloud.reserve(scan3d.size());
        ranges.reserve(scan3d.size());
        for (const auto &point : scan3d) {
            float range =
                std::sqrt(point.x() * point.x() + point.y() * point.y());
            ranges.emplace_back(range);
            float x = static_cast<float>(point.x());
            float y = static_cast<float>(point.y());
            point_cloud.emplace_back(Point2D(x, y));
        }
    }
    // pointcloud
    Eigen::Vector3f origin;
    // Pose2D origin;
    // vector<Pose2D> point_cloud;
    PointCloud2D point_cloud;
    PointCloud2D reflectors;
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
    bool is_extrinsic_set;

    void ConvertRangeDataToPointCloud();
    bool ConvertROSMessageToCustomStructure(
        const uint32_t sec, const uint32_t nsec, const float &angle_increment,
        const float &angle_min, const float &angle_max, const float &range_min,
        const float &range_max, const std::vector<float> &ranges,
        const std::vector<float> &intensities);

    void SetExtrinsicParameter(
        const double &x, const double &y, const double &z, const double &roll,
        const double &pitch, const double &yaw);
    Extrinsic GetExtrinsic() const { return extrinsic; }
    inline void clear()
    {
        point_cloud.clear();
        reflectors.clear();
        ranges.clear();
        intensities.clear();
        secs = 0;
        nsecs = 0;
        angle_min = 0.0f;
        angle_max = 0.0f;
        angle_increment = 0.0f;
        range_min = 0.0f;
        range_max = 0.0f;
        extrinsic = Extrinsic();
    }
    // bool NormalizeIntensities();
};
}  // namespace ANSWER

#endif