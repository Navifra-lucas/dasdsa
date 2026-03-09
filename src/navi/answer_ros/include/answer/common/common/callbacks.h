#pragma once

#include "Poco/JSON/Object.h"
#include "common/3d/scan3d.h"
#include "common/answer_utils.h"
#include "common/icp_result.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/types.h"
#include "common/wheel_odometry.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>
namespace ANSWER {
struct LogCallback {
    static CallbackSlot<void(const string &log)> log_callback;
};
struct VisualizerCallback {
    static CallbackSlot<void(
        const Scan2D &scan, const Pose2D &reference_frame,
        const string &frame_name, const string &color_code, float scale)>
        scan_pose_callback;

    static CallbackSlot<void(
        const PointCloud2D &scan, const string &frame_name,
        const string &color_code, const float &scale)>
        point_cloud_2d_callback;
    static CallbackSlot<void(const Pose2D &odom)> odom_callback;
    static CallbackSlot<void(
        const Pose2D &pose, const string &name, const string &color)>
        pose2d_drawable_callback;
    static CallbackSlot<void(const bool &is_reflector_localization)>
        is_reflector_localization_callback;

    static CallbackSlot<void(
        const Scan3D &point_cloud, const Eigen::Isometry3d &reference,
        const string &frame_name, const string &color_code, const float &scale)>
        point_cloud_3d_callback;

    static CallbackSlot<void(
        const Scan3D &point_cloud, const Eigen::Isometry3d &reference,
        const string &frame_name, const Eigen::Vector3f &color_code)>
        point_cloud_3d_with_color_code_callback;

    static CallbackSlot<void(
        const Scan3D &point_cloud, const Eigen::Isometry3d &reference,
        const string &frame_name, const Eigen::Vector3f &color_code)>
        cube_3d_with_color_code_callback;
    static CallbackSlot<void(
        const PointCloud2D &pointcloud, const PointCloud2D &normalvector2d)>
        normal_vector_2d_callback;

    static CallbackSlot<void(const std::vector<Pose2D> &pose_vector)>
        pose_vector_drawable_callback;
};

struct SLAMCallback {
    static CallbackSlot<void(const Scan2D &scan, const Pose2D &odom)>
        slam_callback;
    static CallbackSlot<void(const bool &map_save_trigger)> map_save_callback;
};
struct StartSLAM {
    static CallbackSlot<void(const int &slam_mode)> start_slam_callback;
};
struct TerminateSLAM {
    static CallbackSlot<void()> terminate_slam_callback;
};

struct ServiceCallback {
    static CallbackSlot<void()> request_global_map_callback;
    static CallbackSlot<void()> update_parameters_callback;
    static CallbackSlot<void(
        const bool &is_random_sampling, const int &sampling_num,
        const float &sample_max_dist, const float &sample_max_angle,
        const float &sample_dist_resolution,
        const float &sample_angle_resolution)>
        resampling_based_init_matching_callback;
};

struct SLAMResultCallback {
    // static CallbackSlot<void(const Scan2D& scan)> scan_callback;
    static CallbackSlot<void(const Pose2D &pose)> pose_callback;
    // static CallbackSlot<void(const std::vector<int8_t>& map)> map_callback;
    static CallbackSlot<void(const Poco::JSON::Object::Ptr map, bool b_publish)>
        map_callback;
    static CallbackSlot<void(
        const std::vector<int16_t> &global_map,
        std::optional<Vector2f> map_size, std::optional<Vector2f> map_origin)>
        global_map_callback;
    static CallbackSlot<void(const Poco::JSON::Object::Ptr pose_graph)>
        pose_graph_callback;
    static CallbackSlot<void(const int &mapping_progress)> progress_callback;
    static CallbackSlot<void(const std::vector<unsigned char> &map)>
        localmap_callback;
};

struct StartLocalization {
    static CallbackSlot<void()> start_localization_callback;
};
struct TerminateLocalization {
    static CallbackSlot<void()> terminate_localization_callback;
};
struct LocalizeCallback {
    static CallbackSlot<void(const Scan2D &scan)> localization_callback;
    static CallbackSlot<void(const Pose2D &odom)> odom_callback;
    static CallbackSlot<void(const Pose2D &initial_pose, bool b_correct)>
        initial_pose_with_trigger_callback;
    static CallbackSlot<void(const Pose2D &init_pose)> init_pose_callback;
    static CallbackSlot<void(const bool &b_trigger)> trigger_find_pose_callback;
    static CallbackSlot<void(const Scan3D &point_cloud)>
        localization_3d_callback;
    static CallbackSlot<void()> RegisterReflectorCallback;
    static CallbackSlot<void()> SwitchReflectorModeCallback;
};

struct LoadMap {
    static CallbackSlot<void(Poco::JSON::Object::Ptr map)>
        load_json_map_callback;
    static CallbackSlot<void(const PointCloud2D &vec_icp_db)>
        load_pcd_map_callback;
    static CallbackSlot<void(const PointCloud3D &map3d)>
        load_pcd_map3d_callback;
};
struct LocalizeResultCallback {
    static CallbackSlot<void(const LOCALIZATION2D::LocalizeResult result)>
        result_callback;
    static CallbackSlot<void(const Pose2D &map_to_robot)> pose_callback;
    static CallbackSlot<void(
        const Pose2D map_to_robot, LOCALIZATION2D::LocalizeResult result)>
        synced_result_callback;
    static CallbackSlot<void(const Poco::JSON::Object::Ptr log)> log_callback;
    static CallbackSlot<void(const Pose3D &map_to_robot)> pose_3d_callback;
    static CallbackSlot<void(const Poco::JSON::Object::Ptr reflectors)>
        reflectors_callback;
};
struct PerceptionResultCallback {
    static CallbackSlot<void(
        const Pose3D &result, const Pose3D &robot_pose,
        const long int &time_stamp)>
        pallet_pose_callback;
    static CallbackSlot<void(
        const Pose3D &result, const Pose3D &robot_pose, const int &time_stamp)>
        wingbody_pose_callback;
};

struct NavigatorCallback {
    static CallbackSlot<void(const Pose2D &goal)> goal_callback;
    static CallbackSlot<void(const std::deque<Pose2D> &goal_list)>
        goal_list_callback;
    static CallbackSlot<void(void)> stop_callback;
    // static CallbackSlot<void(const bool &is_navigation)> start_callback;
};

struct LoggerCallback {
    static CallbackSlot<void(const std::string &log_level)> log_level_callback;
    static CallbackSlot<void(void)> record_bag_callback;
    static CallbackSlot<void(const std::string bag_file)> play_bag_callback;
    static CallbackSlot<void(const Pose2D &pose)> save_pose_callback;
};

struct BrainUICallback {
    static CallbackSlot<void(
        const std::vector<Scan2D> &scans, const Pose2D &map_to_robot)>
        scan_visualizer_callback;
};

}  // namespace ANSWER