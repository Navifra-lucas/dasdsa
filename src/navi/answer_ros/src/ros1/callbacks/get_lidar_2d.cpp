#include "ros1/answer_ros1.h"
namespace ANSWER {
void AnswerRos1::GetLidar2D(const sensor_msgs::PointCloud::ConstPtr msg)
{
    Scan2D scan =
        ConvertROSToScan2D<sensor_msgs::PointCloud::ConstPtr, Scan2D>(msg);
    // if (extrinsics_.empty() == false) {
    //    scan.SetExtrinsicParameter(
    //        extrinsics_[0](0), extrinsics_[0](1), extrinsics_[0](2),
    //        extrinsics_[0](3), extrinsics_[0](4), extrinsics_[0](5));
    //    scan.ConvertRangeDataToPointCloud();
    //}

    // LOG_INFO("here(lidar)");
    lidar_stamp_ = msg->header.stamp;
#if 0
    if (Configurator::GetInstance()
            .GetParamValue("ros", "base", "use_imu")
            .convert<bool>() == true) {
        auto total_nanoseconds = std::chrono::seconds(msg->header.stamp.sec) +
            std::chrono::nanoseconds(msg->header.stamp.nsec);
        ESKF::GetInstance().Predict(total_nanoseconds);
        auto imupose = ESKF::GetInstance().GetState2D();
        last_odom_pose_ = Pose2D(
            imupose.translation().x(), imupose.translation().y(),
            imupose.so2().log());

        LocalizeCallback::odom_callback(last_odom_pose_);
        VisualizerCallback::pose2d_drawable_callback(
            last_odom_pose_, "imu", "green");
    }
    else
#endif
    {
        last_odom_pose_ = SLAM2D::WheelOdometry::GetInstance().GetOdomPose();
    }
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