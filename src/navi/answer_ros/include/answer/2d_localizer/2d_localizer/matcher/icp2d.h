#pragma once

#include "2d_localizer/common/localizer_parameter_container.h"
#include "2d_localizer/matcher/icp2d.h"
#include "2d_localizer/matcher/kdtree_wrapper.h"
#include "common/icp_result.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/time_checker.h"
#include "nanoflann/nanoflann.h"

#include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
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
#include <tuple>
#include <unordered_map>

using ITERATION_LIMIT = int;
using KDTREE_K = int;
using MATCHING_DIST = float;

namespace ANSWER {

using namespace LOCALIZATION2D;
class icp2d {
private:
    /* --------------------------------- data ----------------------------------
     */

    /**
     * @param kdtree_            nanoflann 기반 트리.
     * @param map_cloud_         지도 (포인트 데이터) -> 멤버변수에서
     * 제외해도문제없는가
     * @param last_confidence_   마지막 컨피던스
     *
     * @param model_             지도 (포인트 데이터) - Eigen
     * @param vec_normals_       지도 노말벡터
     */
    std::unique_ptr<KDTree2f> kdtree_;
    PointCloudf map_cloud_;
    float last_confidence_;

    Eigen::MatrixXf model_;
    localizer_parameter_container *localizer_param_;
    localizer_parameter_container *init_pose_localizer_param_;
    PointCloud2D vec_normals_;

    bool reflector_detected_;

    /* --------------------------------- Methods
     * ---------------------------------- */
    PointCloud2D ComputeNormals(
        const PointCloud2D &points, const int &step = 1);
    std::tuple<ITERATION_LIMIT, KDTREE_K, MATCHING_DIST>
    GetInitialAdaptiveParameters(
        const Pose2D &o_predict_robot_pos, const bool is_tracking = true);
    std::pair<KDTREE_K, MATCHING_DIST> UpdateAdaptiveParameters(
        const int &iteration, localizer_parameter_container *param_container);
    std::pair<KDTREE_K, MATCHING_DIST> UpdateAdaptiveParametersInitPose(
        const int &iteration, localizer_parameter_container *param_container);
    void InspectConfidence(
        const float &global_robot_angle,
        const std::vector<float> &vec_matched_angle,
        std::tuple<float, float, float, float> &section_matching_ratio);

public:
    icp2d(/* args */);

    ~icp2d();

    LocalizeResult AlignPoint2PlaneInitPose(
        const PointCloud2D &vec_current_data, const Pose2D &o_predict_robot_pos,
        localizer_parameter_container *param_container);
    LocalizeResult AlignPoint2PlaneLocalize(
        const PointCloud2D &vec_current_data, const Pose2D &o_predict_robot_pos,
        const Pose2D &pose_diff, localizer_parameter_container *param_container,
        std::tuple<float, float, float, float> &section_matching_ratio,
        const bool reflector_detected = false);
    bool BuildKDTree(const PointCloud2D &vec_db_pts);
    bool SetParameters(
        localizer_parameter_container *param_container,
        const bool b_initial_pose);
    std::tuple<
        nanoflann::KNNResultSet<float>, std::vector<size_t>, std::vector<float>>
    FindCorrespondence(
        const KDTree2f *kdtree, const Eigen::Vector2f &query, const int &K);
    float ComputePoint2PlaneError(
        const Eigen::Vector2f &target, const Eigen::Vector2f &query,
        const Point2D &normal);
    Eigen::Vector3f ComputePoint2PlaneJacobian(
        const Point2D &lidar_ptr, const Pose2D &estimated,
        const Point2D &normal);
    float CalculateConfidence(
        const int &scan_size, const std::vector<bool> &vec_effective,
        const int &K);
    bool CheckConvergence(
        const Eigen::Vector3f &delta, const int &current_iteration,
        const int &minimum_iteration, const float &current_computation,
        const float &time_limit = 1000);
    const PointCloud2D &GetNormalVectors() { return vec_normals_; }
};
}  // namespace ANSWER