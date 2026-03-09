#pragma once

#include "common/3d/icp_result_3d.h"
#include "common/3d/point_cloud_utils.h"
#include "common/3d/scan3d.h"
#include "common/configurator.h"
#include "common/pch.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/tbb.h>

namespace ANSWER {
namespace MATCHER {
class icp3d {
private:
    void UpdateParameters();

    double CalculateConfidence(
        const int &scan_size, const std::vector<bool> &vec_effective,
        const int &K);

    std::tuple<
        nanoflann::KNNResultSet<double>, std::vector<size_t>,
        std::vector<double>>
    FindCorrespondence(
        const KDTree3d *kdtree, const Eigen::Vector3d &query, const int &K);
    double ComputePoint2PlaneError3D(
        const Eigen::Vector3d &target_point, const Eigen::Vector3d &query_point,
        const Eigen::Vector3d &target_normal);
    Eigen::Matrix<double, 6, 1> ComputePoint2PlaneJacobian3D(
        const Eigen::Vector3d &query_point,
        const Eigen::Vector3d &target_normal);

    std::string method_;
    int max_iterations_;
    double max_correspondence_distance_;
    double min_correspondence_distance_;

    std::optional<double> coarse_matching_distance_;

    double min_matching_ratio_;
    int matching_neighbors_;
    std::string kernel_;
    double d_confidence_;

public:
    icp3d();
    ~icp3d();
    ANSWER::SLAM3D::ICPResult3D AlignPointToPlane(
        const Scan3D &source, const Scan3D &target, const KDTree3d *kdtree,
        const Pose3D &initial_guess);

    void SetGlobalConfidence(const double &confidence)
    {
        d_confidence_ = confidence;
    }

    void SetMinCorrespondenceDistance(const double &min_distance)
    {
        min_correspondence_distance_ = min_distance;
    }

    void SetCoarseMatchingDistance(const double &coarse_distance)
    {
        coarse_matching_distance_ = coarse_distance;
    }
};
}  // namespace MATCHER
}  // namespace ANSWER