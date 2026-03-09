#pragma once
#include "common/answer_utils.h"
#include "common/callbacks.h"
#include "common/keys.h"
#include "common/math.h"
#include "common/pose2d.h"
//#include "common/pose3d.h"
//#include "common/3d/pose3d.h"
#include "common/3d/scan3d.h"
#include "common/scan2d.h"
#include "logger/logger.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "sophus/se3.h"
#include "sophus/so3.h"

#include <glk/pointcloud_buffer_pcl.hpp>
#include <guik/hovered_drawings.hpp>
#include <guik/initpose.hpp>
#include <guik/navigator_control.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>

// #include <ImGuizmo.h>
#include <Poco/File.h>
#include <Poco/FileStream.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <glk/gridmap.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/thin_lines.hpp>
#include <guik/spdlog_sink.hpp>
#include <guik/viewer/async_light_viewer.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <imgui.h>
#include <opencv2/imgproc.hpp>
#include <portable-file-dialogs.h>
#include <sys/sysinfo.h>

#include <chrono>
#include <ctime>
#include <execution>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace ANSWER {
struct Float3Hash {
    size_t operator()(const std::tuple<float, float, float> &t) const
    {
        auto [x, y, theta] = t;
        return std::hash<float>{}(x) ^ std::hash<float>{}(y) << 1 ^
            std::hash<float>{}(theta) << 2;
    }
};

struct PoseGraphData {
    std::vector<float> xs, ys, zs;
    std::vector<uint32_t> from_idx, to_idx;
    std::unordered_map<std::string, uint32_t> id_to_index;
};

class Visualizer {
public:
    Visualizer(Logger *logger);
    ~Visualizer();
    void Run();
    void StartUI();

    std::thread visualizer_thread_;
    bool terminate_;

private:
    std::mutex invoke_queue_mutex_;
    std::mutex odom_mutex_;
    std::mutex slam_mutex_;
    std::mutex map_mutex_;
    std::mutex reflectors_mutex_;
    std::mutex drawing_id_mutex_;

    std::vector<std::function<void()>> invoke_queue_;
    void GetScanWithFrame(
        const Scan2D &scan, const Pose2D &reference_frame,
        const string &frame_name, const string &color_code, float scale);
    void GetPose2DDrawable(
        const Pose2D &pose, const string &name, const string &color);
    void GetOdom(const Pose2D &pose);
    void GetSLAMPose(const Pose2D &pose);
    void GetSLAMMap(const Poco::JSON::Object::Ptr map, bool b_publish);
    void GetNormalVector2D(
        const PointCloud2D &pointcloud, const PointCloud2D &normalvector2d);
    void GetPoseVectorDrawable(const std::vector<Pose2D> &pose_vector);
    void GetLocalMap(const std::vector<unsigned char> &map);
    void GetPoseGraph(const Poco::JSON::Object::Ptr pose_graph);
    void ButtonForLogLevel(guik::LightViewer *viewer);
    void PrintAnswerStatus(guik::LightViewer *viewer);

    void CheckCheckBox(guik::LightViewer *viewer);
    void GetPointCloud2D(
        const PointCloud2D &scan, const string &frame_name,
        const string &color_code, float scale);

    void GetPointCloud3D(
        const Scan3D &point_cloud, const Eigen::Isometry3d &reference,
        const string &frame_name, const string &color_code, const float &scale);

    void GetPointCloud3DWithColorCode(
        const Scan3D &point_cloud, const Eigen::Isometry3d &reference,
        const string &frame_name, const Eigen::Vector3f &color_code);

    void GetCube3DWithColorCode(
        const Scan3D &point_cloud, const Eigen::Isometry3d &reference,
        const string &frame_name, const Eigen::Vector3f &color_code);

    void Invoke(const std::function<void()> &task);
    void SetDrawingPoseVector(
        const Eigen::Vector3f &latest,
        std::vector<Eigen::Vector3f> &drawing_pose_vector,
        const size_t &drawing_count, const float &distance_condition);

    void GetMappingProgress(const int &progress);

    void RegisterUiCallbacks(guik::LightViewer *viewer);
    void AddLog(const char *format, ...);
    void MakeNavigationUI(guik::LightViewer *viewer);
    void SetInitPoseUI(guik::LightViewer *viewer);
    void EditParam(guik::LightViewer *viewer);

    void GetLocalizerLog(const Poco::JSON::Object::Ptr log);
    void UpdateDrawablePoseGraph(guik::LightViewer *viewer);
    void VisualizeNavifraCI(guik::LightViewer *viewer);
    void VisualizeAnswerName(guik::LightViewer *viewer);
    bool LoadNavifraCI();
    bool LoadAnswerName();

    Pose2D current_odom_pose_2d_;
    std::vector<Pose2D> odom_poses_;
    Pose2D current_slam_pose_2d_;
    std::vector<Pose2D> slam_poses_;
    PointCloud2D reflectors_;
    std::vector<Eigen::Vector3f> robot_pose_trajectory_;
    std::vector<Eigen::Vector3f> odom_trajectory_;
    std::map<std::string, std::vector<Eigen::Vector3f>> drawable_trajectory_;
    PoseGraphData graph_data_;

    size_t odom_drwaing_count_;
    size_t slam_drawing_count_;
    float drawing_dist_condition_;
    float prior_height_cut_;
    int mapping_progress_;
    std::chrono::steady_clock::time_point init_sampling_time_;

    std::vector<std::string> logs_;
    bool autoScroll_;
    bool follow_pose_;
    bool edit_param_;
    bool draw_localmap_;
    bool navigation_mode_;
    bool initpose_mode_;
    bool draw_slam_graph_;
    bool b_slam_graph_update_ = false;
    bool b_run_update_mode_ = false;
    bool is_reflector_localization_;
    bool load_3d_map_;
    bool record_bag_;
    bool play_bag_;
    bool pushed_init_pose_;
    bool show_lidar_;
    bool show_init_frame_;
    bool show_odom_;
    bool use_topdown_camera_;
    // perception

    std::unique_ptr<guik::NavigatorControl> navigator_control_;
    std::unique_ptr<guik::InitPose> initpose_;
    std::deque<Pose2D> goal_list_;
    std::vector<uint64_t> drawing_ids_;
    std::unordered_map<std::tuple<float, float, float>, int, Float3Hash>
        goal_check_map_;

    pcl::PointCloud<pcl::PointXYZ> cloud_original_;
    // localization log
    std::deque<double> confidence_dq_;
    std::deque<double> computation_dq_;
    std::deque<double> section_ratio_up_dq_;
    std::deque<double> section_ratio_down_dq_;
    std::deque<double> section_ratio_left_dq_;
    std::deque<double> section_ratio_total_dq_;
    std::deque<double> section_ratio_right_dq_;
    std::deque<double> rmse_dq_;
    Logger *logger_;
    cv::Mat navifra_ci_image_;
    cv::Mat answer_image_;
    // std::shared_ptr<guik::HoveredDrawings> hovered;
};
}  // namespace ANSWER
