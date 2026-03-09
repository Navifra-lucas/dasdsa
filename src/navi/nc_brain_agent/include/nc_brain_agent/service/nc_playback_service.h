#ifndef NC_PLAY_BACK_SERVICE_H
#define NC_PLAY_BACK_SERVICE_H

#include "pos/pos.hpp"

#include <Poco/Runnable.h>
#include <core_agent/data/robot_collision_info.h>
#include <core_agent/data/robot_pose.h>
#include <core_agent/data/robot_wheel_info.h>
#include <core_msgs/MotorInfo.h>
#include <core_msgs/NavicoreStatus.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>

namespace NaviFra {
class NcPlaybackService
    : public Poco::Runnable
    , public boost::noncopyable
    , public boost::serialization::singleton<NcPlaybackService> {
public:
    NcPlaybackService();
    ~NcPlaybackService();

    static NcPlaybackService& get() { return boost::serialization::singleton<NcPlaybackService>::get_mutable_instance(); }

    void initialize(ros::NodeHandle& nh);

private:
    void onFrontCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onRearCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onLeftCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onRightCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onV2VCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onCameraCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onObsCloud(const sensor_msgs::PointCloud::ConstPtr msg);
    void onRobotPos(const geometry_msgs::PoseWithCovarianceStamped msg);
    void onNavifrainfo(const core_msgs::NavicoreStatus msg);
    void onMotorInfo(const core_msgs::MotorInfo::ConstPtr msg);
    void onGlobalPath(const nav_msgs::Path path);
    void onLocalPath(const nav_msgs::Path path);
    void onTime(const std_msgs::String time);
    void onPredictCollision(const geometry_msgs::PolygonStamped msg);
    void onObsPos(const geometry_msgs::PolygonStamped msg);
    void onCollision(const geometry_msgs::PolygonStamped polygon);
    void onCustomMessgae(const std_msgs::String::ConstPtr msg);

    sensor_msgs::PointCloud ToGlobal(sensor_msgs::PointCloud& vec_cloud, NaviFra::Pos current_pose);

    void sendPointCloud();
    virtual void run() override;

public:
    void start();
    void stop();
    bool isRunning();

private:
    RobotInfo::Ptr robotInfo_;
    RobotWheelInfoStore::Ptr robotWheelInfoStore_;
    RobotCollisionInfo::Ptr robotCollisionInfo_;
    RobotPose::Ptr robotPose_;

    bool isRunning_;
    std::string frontCloud_;
    std::string rearCloud_;
    std::string leftCloud_;
    std::string rightCloud_;
    std::string v2vCloud_;
    std::string cameraCloud_;
    std::string obsCloud_;

    std::string playTime_;
    std::string predict_polygon_;
    std::string obs_polygon_;
    std::string etc_message_;

    std::vector<ros::Subscriber> rosSubscriber_;
    Poco::Thread worker_;
    Poco::FastMutex mutex_;
    NaviFra::Pos current_pose_;
};

inline bool NcPlaybackService::isRunning()
{
    return isRunning_;
}

inline void NcPlaybackService::start()
{
    if (isRunning() != true) {
        isRunning_ = true;
        worker_.start(*this);
    }
}

inline void NcPlaybackService::stop()
{
    try {
        if (isRunning()) {
            isRunning_ = false;
            worker_.tryJoin(1000);
        }
    }
    catch (Poco::TimeoutException& ex) {
        NLOG(error) << "Worker Stop timeout";
    }
}

}  // namespace NaviFra
#endif