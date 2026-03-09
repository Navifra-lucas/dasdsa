#pragma once
#include "2d_slam/common/slam_parameter_container.h"
#include "2d_slam/globalmapper/globalmap/global_map_builder_2d.h"
#include "2d_slam/globalmapper/globalmap/high_resolution_map_builder_2d.h"
#include "2d_slam/localmapper/lidar_odometry_2d.h"
#include "2d_slam/map_generator.h"
#include "common/configurator.h"
#include "common/pch.h"
#include "common/pose2d.h"
#include "common/scan2d.h"

namespace ANSWER {
namespace SLAM2D {
class SLAM {
public:
    SLAM();
    ~SLAM();
    void Initialize();

    bool DoSLAM(const Scan2D &scan, const Pose2D &odom);
    bool DoSLAMThread(const Scan2D &scan, const Pose2D &odom);

    bool b_slam_initialized_;

    void StartSLAM(const int &slam_mode);
    void TerminateSLAM();
    const Pose2D GetRobotPose();

private:
    slam_parameter_container slam_params_;
    void UpdateParameters();
    void InitializeInstance(const int &slam_mode);
    void CreateMapGenerator();
    void run();

    void FindPose(const Pose2D &initpose, const bool &b_trigger);

    std::thread do_slam_thread_;
    std::unique_ptr<LiDAROdometry2D> lidar_odometry_;
    std::unique_ptr<MapGenerator> map_generator_;
    std::chrono::time_point<std::chrono::system_clock> update_time_;
    Pose2D last_odom_pose_;
    Pose2D visualize_last_odom_pose_;
    Pose2D map_to_robot_;
    std::mutex mtx_map_to_robot_;
    std::mutex mtx_do_slam_;
    bool b_doing_slam_;
    void StartDrawMap(const bool &b_save_map);
    bool CheckStatus(const ANSWER::STATUS &status);
    bool CheckInitialized();
};
}  // namespace SLAM2D
}  // namespace ANSWER