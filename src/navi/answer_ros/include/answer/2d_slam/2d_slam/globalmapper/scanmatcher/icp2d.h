#pragma once

#include "2d_slam/localmapper/localmap/localmap2d.h"
#include "common/icp_result.h"
#include "common/kdtree_wrapper.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/time_checker.h"
#include "sophus/se2.h"
#include "sophus/so2.h"

#include <opencv2/opencv.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/tbb.h>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <memory>
#include <random>
#include <unordered_map>

using namespace Sophus;
using ITERATION = int;
using KDTREE_K = int;
using MATCHING_DIST = float;

namespace ANSWER {
namespace SLAM2D {

class icp2d {
private:
    /* data */

    Eigen::MatrixXf model_;

    int iteration_;
    int knn_search_num_;
    float correspondence_dist_;
    float max_rmse_;
    float min_matching_ratio_;

    int index = 0;
    int ref_index = 0;
    bool use_weight_;

    void ValidateMatchResult(ICPResult& result);
    void BuildGlobalKDTree(const PointCloud2D& target);
    // void BuildKDTree(const PointCloud2D& target, Nabo::NNSearchF* nabo_kdtree);
    PointCloud2D ComputeNormalsSimple(const PointCloud2D& points, const int& step = 1);
    PointCloud2D ComputeNormals(const PointCloud2D& points, KDTreeWrapper::KDTree2D* kdtree);
    PointCloud2D ComputeNormals(Localmap2D* target);

    std::unique_ptr<tbb::task_arena> arena_;
    std::unique_ptr<tbb::global_control> tbb_global_control_;

public:
    icp2d(/* args */);

    ~icp2d();

    ICPResult AlignGaussPoint2PlaneSLAM(Localmap2D* source, Localmap2D* target, const SE2f& initial_guess, const bool isTracking = false);

    bool SetParameters(
        const int& iteration, const float& correspondence_dist, const int& knn_search_num, const float& min_matching_ratio,
        const float& max_rmse, const bool use_weight = false);

    bool SavePCDFile(const PointCloud2D& pointcloud, const float& confidence = 100, const bool is_reference = false);
    bool SavePCDFile(const Eigen::MatrixXf& pointcloud, const float& confidence = 100, const bool is_reference = false);
    std::tuple<ITERATION, KDTREE_K, MATCHING_DIST> GetAdaptiveParameters(const SE2f& initial_guess, const bool isTracking = false);
    bool CheckConvergence(const Eigen::Vector3f& dx);
    bool CheckWrongMatch(const float& confidence, const int& iter, const int& iterations);

    std::pair<MATCHING_DIST, KDTREE_K> UpdateParameters(
        const int& iteration, const float& initial_match_dist, const int& initial_k, const float& match_dist_weight, const float& k_weight);
};

}  // namespace SLAM2D
}  // namespace ANSWER