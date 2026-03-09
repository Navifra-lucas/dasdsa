#pragma once
#include "common/callbacks.h"
#include "common/pch.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "logger/logger.h"

namespace ANSWER {
class ReflectorMatcher {
public:
    ReflectorMatcher();
    ~ReflectorMatcher();

    // Matches detected reflectors with global reflectors and estimates pose
    Pose2D MatchAndEstimatePose(
        const PointCloud2D &global_reflectors,
        const Eigen::VectorXd &detected_reflectors,
        const Pose2D &reference_frame);

    bool MatchAndEstimatePose(
        const Eigen::VectorXd &detected_reflectors,
        const Pose2D reference_frame, Pose2D &estimated_pose,
        Scan2D &matched_reflectors);

    void SetReflectorMap(const PointCloud2D &reflector_map);
    void SetParameters(
        const int minimum_matched_num, const int pattern_match_vote_thresh,
        const float pattern_distance_m_tolerance,
        const float pattern_angle_rad_tolerance, const float matching_dist_max,
        const float f_modified_z_score);

private:
    PointCloud2D reflector_map_;  // Global reflectors map
    int minimum_matched_num_;
    int pattern_match_vote_thresh_;
    float pattern_distance_m_tolerance_;
    float pattern_angle_rad_tolerance_;
    float matching_dist_max_;
    float f_modified_z_score_;

    std::vector<std::tuple<float, float, int, int>> reflector_patterns_;
    // Helper function to find the best match for a detected reflector
    int FindBestMatch(
        const int index, const Pose2D &detected_reflector,
        const std::vector<Pose2D> &global_reflectors,
        std::map<int, std::pair<int, float>> &matched_map);

    // Helper function to estimate pose based on matched reflectors

    const Pose2D CalculateTransformation(
        const std::vector<std::pair<Pose2D, Pose2D>> &matched_pairs);

    void PatternizeDetectedReflectors(
        const std::vector<Pose2D> &detected_reflectors,
        std::vector<std::tuple<float, float, int, int>> &patterns);
    void MatchPatterns(
        const std::vector<std::tuple<float, float, int, int>> &patterns,
        const std::vector<std::tuple<float, float, int, int>> &global_patterns,
        std::vector<int> &valid_index);
    std::vector<std::pair<Pose2D, Pose2D>> OutlierRemoval(
        std::vector<std::pair<Pose2D, Pose2D>> matched_paris);
};
}  // namespace ANSWER