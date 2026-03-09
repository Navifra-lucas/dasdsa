#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetLaserScan2D(const sensor_msgs::LaserScan::ConstPtr msg)
{
    Scan2D scan = ConvertROS1LaserScanToScan2D<
        sensor_msgs::LaserScan::ConstPtr, ANSWER::Scan2D>(msg);

    if (extrinsics_map_.find(KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR) ==
        extrinsics_map_.end()) {
        LOG_TRACE("Lidar extrinsic not found");
        return;
    }
    else {
        scan.SetExtrinsicParameter(
            extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR](0),
            extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR](1),
            extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR](2),
            extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR](3),
            extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR](4),
            extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR](5));
        scan.ConvertRangeDataToPointCloud();
    }

    // set lidar stamp to now
    lidar_stamp_ = msg->header.stamp;

    last_odom_pose_ = SLAM2D::WheelOdometry::GetInstance().GetOdomPose();
    AnswerStatus::GetInstance().UpdateLidarTime();
    if (AnswerStatus::GetInstance().GetStatus() ==
        ANSWER::STATUS::LOCALIZATION) {
        LocalizeCallback::localization_callback(scan);
    }
    else if (AnswerStatus::GetInstance().GetStatus() == ANSWER::STATUS::SLAM) {
        SLAMCallback::slam_callback(
            scan, SLAM2D::WheelOdometry::GetInstance().GetOdomPose());
    }
}
}  // namespace ANSWER