#ifndef HIGH_RESOLUTION_MAP_BUILDER_2D_H
#define HIGH_RESOLUTION_MAP_BUILDER_2D_H

#include "2d_slam/common/slam_parameter_container.h"
#include "2d_slam/globalmapper/mapper.h"
// #include "2d_slam/globalmapper/scanmatcher/icp_matcher.h"
#include "2d_slam/localmapper/localmap/localmap2d.h"
#include "common/pose2d.h"

#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

using namespace std;
namespace ANSWER {
namespace SLAM2D {
class HighResolutionMapBuilder2D {
private:
    /* data */
    const slam_parameter_container* slam_param_;
    bool b_build_high_resolution_map_ = false;
    bool b_localization_pcd_builder_ = false;
    bool b_set_first_localmap_ = false;
    bool b_corrected_ = false;
    thread high_resolution_builder_thread_;
    thread localization_point_cloud_builder_thread_;
    shared_ptr<Localmap2D> high_resolution_localmap_;
    // vector<pair<int,Scan2D>> all_scan_;
    // deque<pair<Pose2D,Scan2D>> pose_scan_deque_;
    deque<tuple<Pose2D, Scan2D, bool>> pose_scan_deque_;
    bool b_localmap_last_data_ = false;
    mutex set_origin_mutex_;

    string map_path_;
    string map_name_;

    float high_resolution_ = 0.01;
    int n_scan_accumulation_num_ = 0;
    Pose2D initial_pose_;
    Pose2D correction_pose_;

    // ICPMatcher icp_matcher_;

public:
    HighResolutionMapBuilder2D(/* args */);
    ~HighResolutionMapBuilder2D();

    void Initialize(const Pose2D initial_pose = Pose2D());

    void SetParam(const slam_parameter_container* slam_param);
    void BuildHighResolutionMap();
    void BuildLocalizationPCD();
    void PushData(const Pose2D current_pose, Scan2D scan, bool b_localmap_last_data = false)
    {
        pose_scan_deque_.emplace_back(make_tuple(current_pose, scan, b_localmap_last_data));
        // b_localmap_last_data_ = b_localmap_last_data;
    }
    void SaveMap();
    void WriteFile(const string path = string());
    shared_ptr<Localmap2D> GetCurrentLocalmap()
    {
        CHECK(high_resolution_localmap_ != nullptr);
        return high_resolution_localmap_;
    }
    void SetCorrectionInfo(const Pose2D correction_pose, const bool is_corrected)
    {
        if (set_origin_mutex_.try_lock()) {
            correction_pose_ = correction_pose;
            set_origin_mutex_.unlock();
            b_corrected_ = is_corrected;
        }
    }
    void SetInitPose(const Pose2D &pose);
    
    static HighResolutionMapBuilder2D* GetInstance()
    {
        static HighResolutionMapBuilder2D s;
        return &s;
    }
};

}  // namespace SLAM2D
}  // namespace ANSWER
#endif