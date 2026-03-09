/**
 * @class lidar odometry 2d
 * @brief SLAM front-end(Generate lidar odometry) using ceres scan matcher.
 * @author logan (donghak lee)
 * contact : donghak.lee@navifra.com
 */
#ifndef __LIDAR_ODOMETRY_2D_H__
#define __LIDAR_ODOMETRY_2D_H__
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/slam_parameter_container.h"
#include "globalmapper/mapper.h"
#include "localmapper/localmap/grid2d.h"
#include "localmapper/scanmatcher/ceres_scan_matcher.h"
#include "math.h"
#include "time_checker/time_checker.h"

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "Eigen/Core"

using namespace std;

namespace NaviFra {
namespace SLAM2D {
enum SIMILAR_CONDITION
{
    NOT = 0,
    TRANSLATION = 1,
    ROTATION = 2,
};
class LiDAROdometry2D {
private:
    /* data */
    Pose2D odom_to_robot_;
    Pose2D map_to_robot_;
    Pose2D acc_delta_odom_;
    Pose2D lastnode_to_current_node_;
    bool b_set_first_localmap_ = false;
    bool b_set_initial_pose_ = false;
    bool b_use_point_cloud_topic_ = false;
    std::unique_ptr<slam_parameter_container> slam_param_;
    shared_ptr<Localmap2D> current_localmap_;
    // Scan2D TransformPointCloudToTargetFrame(const Pose2D &localmap_to_robot,
    //                                         const Scan2D &robot_to_point);
    int n_last_localmap_index_ = 0;
    int n_scan_accumulation_condition_ = 0;
    float f_scan_accumulation_amount_dist_ = 0;
    float f_scan_accumulation_amount_rot_ = 0;

    float f_scan_accumulation_condition_dist_ = 0;
    float f_scan_accumulation_condition_rot_ = 0;
    float f_robot_accumulation_amount_dist_ = 0;
    float f_robot_accumulation_amount_rot_ = 0;
    vector<Scan2D> localmap_all_scan_;

    unique_ptr<CeresScanMatcher> scan_matcher_;
    std::mutex localmap_update_mutex_;
    TimeChecker time_recorder_;

public:
    LiDAROdometry2D();
    ~LiDAROdometry2D();
    const Pose2D GetOdomPose() const { return odom_to_robot_; }

    bool InitializeSLAM(const int slam_mode, const bool b_use_point_cloud_topic);
    const Pose2D DoSLAM(const Pose2D delta_odom, Scan2D& ranges, std::chrono::time_point<std::chrono::steady_clock>& update_time);

    const Pose2D GetLiDAROdometry(const Pose2D delta_odom, Scan2D& ranges);
    bool ReadParam();
    SIMILAR_CONDITION IsSimilar(const Pose2D delta_odom);
    void SetInitialPose(const Pose2D init_map_to_robot);
    void SetFindInitPose(const bool b_find_init_pose)
    {
        if (localmap_update_mutex_.try_lock()) {
            b_set_initial_pose_ = b_find_init_pose;
            localmap_update_mutex_.unlock();
        }
    }
};

}  // namespace SLAM2D
}  // namespace NaviFra

#endif