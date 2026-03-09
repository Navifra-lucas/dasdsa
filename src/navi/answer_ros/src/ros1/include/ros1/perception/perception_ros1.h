#pragma once
#include "3d_perception/pallet_detector.h"
#include "3d_perception/pallet_icp_matcher.h"
#include "3d_perception/wingbody_detector.h"
#include "common/answer_utils.h"
#include "common/callbacks.h"
#include "common/keys.h"
#include "common/pch.h"
#include "common/pose2d.h"
//#include "common/pose3d.h"
//#include "common/3d/pose3d.h"
#include "common/3d/scan3d.h"
#include "common/msg_converter.h"
#include "common/scan2d.h"
#include "common/wheel_odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "logger/logger.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#ifdef BUILD_WITH_VISUALIZER
    #include "visualizer/visualizer.h"
#endif

namespace ANSWER {
namespace PERCEPTION {
class PerceptionRos1 {
public:
    PerceptionRos1(const std::string &node_name);
    ~PerceptionRos1();
    void TerminateNode();

private:
    void GenerateSubscriptions();
    void GeneratePublishers();
    void GenerateServices();
    void Initialize();
    void UpdateParameters();

    // void GetLidar3D(const sensor_msgs::PointCloud2::ConstPtr msg);
    std::shared_ptr<Logger> logger_;
    std::unique_ptr<Visualizer> visualizer_;
    std::unique_ptr<PalletDetector> pallet_detector_;
    std::unique_ptr<PalletICPMatcher> pallet_icp_matcher_;
    std::unique_ptr<WingbodyDetector> wingbody_detector_;
    std::mutex mtx_scan3d_;
    std::mutex mtx_robot_pose_;
    ros::NodeHandle nh_;
    Scan3D latest_scan_;
    Eigen::Vector3d initial_guess_;
    int pallet_type_;
    bool b_cmd_start_;
    bool b_init_odom_;
    bool b_local_;

    bool b_publish_global_pose_;
    bool b_wingbody_cmd_start_;

    Pose2D o_prev_odom_;
    Pose2D o_current_robot_pose_;
    Pose3D odom_add_guess_;

    // Eigen::Vector3d wingbody_initial_guess_;
    Pose3D wingbody_initial_guess_;
    Pose3D T_first_to_center_;

    bool b_init_wingbody_odom_;
    Pose2D o_wingbody_prev_odom_;
    Pose2D o_wingbody_current_robot_pose_;
    Pose3D wingbody_odom_add_guess_;

    std::atomic<bool> detecting_{false};
    std::thread detecting_pallet_thread_;

    float f_detect_pallet_end_condition_;
    float f_yaw_offset_;

    std::vector<double> vec_pallet_width_;
    std::vector<double> vec_pallet_height_;
    std::vector<double> vec_pallet_x_offset_;
    std::vector<double> vec_pallet_y_offset_;
    std::vector<double> vec_pallet_z_offset_;
    std::vector<double> vec_pallet_deg_offset_;
    /** 팔레트별 결과 pose 보정 (템플릿 offset과 별개, /perception/pose 발행 직전 적용) [m], [deg] */
    std::vector<double> vec_output_pose_offset_x_;
    std::vector<double> vec_output_pose_offset_y_;
    std::vector<double> vec_output_pose_offset_z_;
    std::vector<double> vec_output_pose_offset_deg_;
    std::vector<Pose3D> vec_lack_positions_;

    std::map<KEY::SUBSCRIPTION::SENSOR, std::shared_ptr<ros::Subscriber>>
        sensor_subscriptions;
    std::map<KEY::SUBSCRIPTION::COMMAND, std::shared_ptr<ros::Subscriber>>
        command_subscriptions;
    std::map<KEY::SUBSCRIPTION::SENSOR, Eigen::Matrix<double, 6, 1>>
        extrinsics_;

    std::map<KEY::PUBLICATION::PERCEPTION, std::shared_ptr<ros::Publisher>>
        publishers_;

    // 디버그: RViz에서 매칭 위치 확인용 (latched)
    ros::Publisher pub_debug_template_initial_;
    ros::Publisher pub_debug_template_aligned_;
    ros::Publisher pub_debug_sensor_raw_;       // 센서 원본 (매 프레임)
    ros::Publisher pub_debug_sensor_matched_;   // 후처리 끝난 센서 = ICP 매칭에 사용된 포인트클라우드
    std::string debug_frame_id_{"base_link"};
};
}  // namespace PERCEPTION
}  // namespace ANSWER