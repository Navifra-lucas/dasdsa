#pragma once
#include "2d_localizer/version.hpp"
#include "3d_slam/slam3d.h"
#include "common/pch.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

namespace ANSWER {

class Answer3DRos1 : public ANSWER::AnswerInterface {
private:
    // variables
    std::shared_ptr<Logger> logger_;
    std::unique_ptr<Visualizer> visualizer_;
    ros::NodeHandle nh_;

    std::map<KEY::SUBSCRIPTION::SENSOR, Eigen::Matrix<double, 6, 1>>
        extrinsics_;
    std::map<KEY::SUBSCRIPTION::SENSOR, std::shared_ptr<ros::Subscriber>>
        sensor_subscriptions;
    std::map<KEY::SUBSCRIPTION::COMMAND, std::shared_ptr<ros::Subscriber>>
        command_subscriptions;

    std::map<KEY::PUBLICATION::SLAM, std::shared_ptr<ros::Publisher>>
        publishers;

    std::unique_ptr<SLAM3D::SLAM> slam_3d_;

    // functions
    void GetLidar3D(const sensor_msgs::PointCloud2::ConstPtr msg);
    void GetImu(const sensor_msgs::Imu::ConstPtr msg);

    void Initialize() override;
    void GenerateSubscriptions() override;
    void GenerateServices() override;
    void GeneratePublishers() override;
    void RegisterCallbacks() override;

public:
    Answer3DRos1(const std::string &node_name);
    ~Answer3DRos1();

    void TerminateNode() override;
};

}  // namespace ANSWER