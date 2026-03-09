#pragma once

#include "2d_localizer/localizer.h"
#include "2d_slam/slam.h"
#include "3d_localizer/localizer_3d.h"
#ifdef ANSWER_NAVICORE
    #include "answer/common_service.h"
using SRV = answer::common_service;
#else
    #include "answer_msgs/CommonService.h"
using SRV = answer_msgs::CommonService;
#endif
#include "common/answer_utils.h"
#include "common/callbacks.h"
#include "common/msg_converter.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "imu_pose_estimator/eskf.h"
#include "logger/logger.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"

#include <tf/transform_broadcaster.h>
#ifdef BUILD_WITH_VISUALIZER
    #include "visualizer/visualizer.h"
#endif

namespace ANSWER {
class AnswerRos1 : public AnswerInterface {
private:
    /* data */
    void Initialize() override;
    void GenerateSubscriptions() override;
    void GenerateServices() override;
    void GeneratePublishers() override;
    void RegisterCallbacks() override;
    std::map<KEY::SUBSCRIPTION::SENSOR, std::shared_ptr<ros::Subscriber>>
        sensor_subscriptions;
    std::map<KEY::SUBSCRIPTION::COMMAND, std::shared_ptr<ros::Subscriber>>
        command_subscriptions;

    std::map<KEY::PUBLICATION::SLAM, std::shared_ptr<ros::Publisher>>
        publishers;
    void GetRegisterReflector(const std_msgs::Bool::ConstPtr msg);
    void GetLidar2D(const sensor_msgs::PointCloud::ConstPtr msg);
    void GetLidar3D(const sensor_msgs::PointCloud2::ConstPtr msg);
    void GetImu(const sensor_msgs::Imu::ConstPtr msg);
    void GetLaserScan2D(const sensor_msgs::LaserScan::ConstPtr msg);
    void GetOdometry2D(const nav_msgs::Odometry::ConstPtr msg);

    void GetInitialPose(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void GetExternalPose(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);

    void GetCorrectionTrigger(const std_msgs::String::ConstPtr msg);
    void GetClearAlarm(const std_msgs::Bool::ConstPtr msg);

    bool ControlServiceCallback(SRV::Request &req, SRV::Response &res);
    bool LogLevelServiceCallback(SRV::Request &req, SRV::Response &res);
    bool UpdateParamServiceCallback(SRV::Request &req, SRV::Response &res);
    bool SamplingMatchServiceCallback(SRV::Request &req, SRV::Response &res);
    void GetMapSave(const std_msgs::Int16::ConstPtr msg);

    Pose2D last_odom_pose_;
    std::unique_ptr<SLAM2D::SLAM> slam_;
    std::unique_ptr<Localizer> localizer_;
    std::unique_ptr<Localizer3D> localizer_3d_;
    std::shared_ptr<Logger> logger_;
    ros::NodeHandle nh_;
    ros::ServiceServer answer_service_;
    std::map<KEY::SERVICES::CONTROL, ros::ServiceServer> control_srv_servers_;

    ros::Time lidar_stamp_;
    ros::Time last_update_stamp_;
    std::vector<Eigen::VectorXf> extrinsics_;
    std::map<KEY::SUBSCRIPTION::SENSOR, Eigen::VectorXf> extrinsics_map_;

    bool b_imu_first_update_;
    std::shared_ptr<tf::TransformBroadcaster> br_;

#ifdef BUILD_WITH_VISUALIZER
    std::unique_ptr<Visualizer> visualizer_;
#endif

public:
    // ANSWER(/* args */) = default;
    AnswerRos1(const std::string &node_name);
    ~AnswerRos1();

    void TerminateNode() override;
};

}  // namespace ANSWER