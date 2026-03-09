/**
 * @class mapper
 * @brief SLAM back-end(graph optimization & loop detection thread) using ceres solver & icp.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef MAPPER_H
#define MAPPER_H
#include "ceres/local_parameterization.h"
#include "ceres/problem.h"
#include "common/ceres_util/angle_manifold.h"
#include "common/ceres_util/pose_graph_2d_error_term.h"
#include "common/data_base.h"
#include "common/pose2d.h"
#include "globalmapper/scanmatcher/icp_matcher.h"
#include "localmapper/localmap/localmap2d.h"
#include "localmapper/localmap/probability_values.h"

#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

#include "Eigen/Core"

using namespace std;
using namespace Eigen;
using namespace ceres::utils;

#define INDEX_DIFF_CONDITION (3)
#define DIST_DIFF_CONDITION (10)
namespace NaviFra {
namespace SLAM2D {
enum SLAM_MODE
{
    NOTHING = -1,
    LOCALIZATION = 0,
    MAPPING,
    UPDATE,
};

class Mapper {
private:
    int n_slam_mode_ = SLAM_MODE::NOTHING;
    Pose2D current_pose_;
    deque<Edge2D> loop_constraint_;

    std::mutex global_map_mutex_;
    std::mutex localize_mutex_;
    std::mutex localization_result_mutex_;
    std::mutex mutex_get_map_size_;
    std::mutex mutex_get_map_origin_;
    std::mutex mutex_get_map_bound_;

    bool b_do_loop_closing_ = false;
    bool b_do_optimize_ = false;
    bool b_optimize_complete_ = false;
    bool b_localized_ = false;
    bool b_find_loop_ = false;
    bool b_localization_ = false;
    bool b_initial_pose_ = false;
    std::thread loopclosing_thread_;
    std::thread optimization_thread_;
    std::thread localization_thread_;
    int n_last_node_index_ = 0;
    int n_initial_node_index_ = 0;
    int n_current_node_index_ = 0;
    std::queue<shared_ptr<Localmap2D>> loop_candidate_;
    std::queue<PointCloud2D> localization_candidate_;
    // meter scale
    Vector2f map_origin_;
    Vector2f map_size_;
    Vector2f mutable_map_size_;

    Vector2f prev_map_origin_;
    Vector2f prev_map_size_;
    // for localmap update
    std::vector<uint16_t> hit_table_;
    std::vector<uint16_t> miss_table_;

    std::vector<uint16_t> high_hit_table_;
    std::vector<uint16_t> high_miss_table_;
    int m_localmap_size = 0;
    float f_map_resolution_;
    float f_hit_probability_;
    float f_miss_probability_;

    float f_high_hit_probability_;
    float f_high_miss_probability_;

    AlignedBox2f map_bound_;

    Pose2D loop_correction_;
    ICPResult icp_result_;

    const slam_parameter_container* slam_param_;
    bool b_global_map_size_updated_ = false;
    std::vector<int8_t> global_map_;

    std::condition_variable do_optimization_condition_;
    std::mutex mutex_do_optimization_;

    unique_ptr<ICPMatcher> pm_;
    unique_ptr<ICPMatcher> pm_localization_;
    void MergePointCloud(Localmap2D* localmap1, Localmap2D* localmap2, const Pose2D& localmap1_to_localmap2);
    void AddConstraint(ceres::Problem* problem, deque<shared_ptr<Node2D>> poses, deque<shared_ptr<Edge2D>> constraint);
    void AddLoopConstraint(ceres::Problem* problem, deque<shared_ptr<Node2D>> poses, deque<Edge2D> LoopConstraint);
    bool SolveOptimizationProblem(ceres::Problem* problem);
    void SetOptimizedResultToLocalmap(deque<shared_ptr<Node2D>> nodes);

public:
    Mapper();
    Mapper(const Mapper& ref);
    ~Mapper();
    static Mapper* GetInstance()
    {
        static Mapper s;
        return &s;
    }

    int GetSLAMMode() const { return n_slam_mode_; }
    void SetSLAMMode(int mode)
    {
        NLOG(info) << "slam mode : " << n_slam_mode_;
        n_slam_mode_ = mode;
    }
    // equal to localmap index and node index
    void AddNode(const Pose2D& current_pose, shared_ptr<Localmap2D> localmap);
    void AddLoopCandidate();
    void AddLocalizationCandidate(PointCloud2D localization_candidate, const bool b_initial_pose = false);
    void SetMapResolution(const float resolution_);
    const float& GetMapResolution() const { return f_map_resolution_; }
    //
    const int& LocalmapSize() const { return m_localmap_size; }

    void DoLocalization();
    void DoOptimization();
    void DetectLoop();
    void DrawGlobalMap();

    void UpdateGlobalMapSize();
    void GenerateLocalmapProbTable(const float hit_probability, const float miss_probability);

    void GenerateHighLocalmapProbTable(const float hit_probability, const float miss_probability);
    const std::vector<int8_t>& GetGlobalMap() const { return global_map_; }
    const Vector2f& GetMapSizeM()
    {
        std::lock_guard<std::mutex> lock(mutex_get_map_size_);
        return map_size_;
    }
    void SetMapSizeM(const Vector2f map_size)
    {
        std::lock_guard<std::mutex> lock(mutex_get_map_size_);
        map_size_ = map_size;
    }
    Vector2f& GetMutableMapSizeM()
    {
        std::lock_guard<std::mutex> lock(mutex_get_map_size_);
        return mutable_map_size_;
    }
    const Vector2f& GetMapOriginM()
    {
        std::lock_guard<std::mutex> lock(mutex_get_map_origin_);
        return map_origin_;
    }
    void SetMapOriginM(const Vector2f map_origin)
    {
        std::lock_guard<std::mutex> lock(mutex_get_map_origin_);
        map_origin_ = map_origin;
    }
    Vector2f& GetMutableMapOriginM()
    {
        std::lock_guard<std::mutex> lock(mutex_get_map_origin_);
        return map_origin_;
    }
    AlignedBox2f& GetMutableMapBound()
    {
        std::lock_guard<std::mutex> lock(mutex_get_map_bound_);
        return map_bound_;
    }
    void SetParam(const slam_parameter_container* slam_param)
    {
        slam_param_ = slam_param;
        pm_->SetParam(slam_param_, n_slam_mode_ == SLAM_MODE::LOCALIZATION);
        pm_localization_->SetParam(slam_param_, true);
    }
    void SetICPReference(const PointCloud2D reference_ptr)
    {
        CHECK(pm_localization_ != nullptr);
        pm_localization_->SetReferenceData(reference_ptr);
    }
    const bool& IsOptimized()
    {
        std::lock_guard<std::mutex> lock_guard(localize_mutex_);
        return b_optimize_complete_;
    }
    const bool& IsLocalized()
    {
        std::lock_guard<std::mutex> lock_guard(localize_mutex_);
        return b_localized_;
    }

    void InitializeOptimizeInfo()
    {
        std::lock_guard<std::mutex> lock_guard(localize_mutex_);
        b_optimize_complete_ = false;
        b_localized_ = false;
        loop_correction_ = Pose2D();
    }
    void InitializeMapper(const int mode = 1);
    const Pose2D& GetLoopCorrection()
    {
        std::lock_guard<std::mutex> lock_guard(localize_mutex_);
        return loop_correction_;
    }
    const ICPResult& GetICPResult()
    {
        std::lock_guard<std::mutex> lock_guard(localization_result_mutex_);
        return icp_result_;
    }
    bool CheckMapSize(const Vector2f& point);

    const std::vector<uint16_t>* GetHitTable() { return &hit_table_; }
    const std::vector<uint16_t>* GetMissTable() { return &miss_table_; }

    const std::vector<uint16_t>* GetHighHitTable() { return &high_hit_table_; }
    const std::vector<uint16_t>* GetHighMissTable() { return &high_miss_table_; }
};
}  // namespace SLAM2D
}  // namespace NaviFra

#endif