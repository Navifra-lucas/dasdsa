#include "ros1/answer_ros1.h"

namespace ANSWER {

void AnswerRos1::GetImu(const sensor_msgs::Imu::ConstPtr msg)
{
#if 0
    AnswerStatus::GetInstance().UpdateOdomTime();
    // if (lidar_stamp_ > msg->header.stamp) {
    //     LOG_TRACE(
    //         "late imu stamp detected, imu stamp: {}, lidar stamp: {}",
    //         msg->header.stamp.toSec(), lidar_stamp_.toSec());
    //     return;
    // }
    // if (last_update_stamp_ > msg->header.stamp) {
    //     LOG_TRACE(
    //         "late imu stamp detected, imu stamp: {}, last update stamp: {}",
    //         msg->header.stamp.toSec(), last_update_stamp_.toSec());
    //     last_update_stamp_ = msg->header.stamp;
    //     return;
    // }

    Eigen::Quaterniond q(
        msg->orientation.w, msg->orientation.x, msg->orientation.y,
        msg->orientation.z);
    Eigen::Vector3d euler = MATH::Quaternion2RPY(q);
    // LOG_TRACE(
    //     "imu orientation: {}, {}, {}", euler.x() * 180. / M_PI,
    //     euler.y() * 180. / M_PI, euler.z() * 180. / M_PI);
    euler(2) = 0;  // set yaw to 0
    auto q2 = MATH::RPYToQuaternion(euler(0), euler(1), euler(2));
    Sophus::SO3d so3(q2);
    Eigen::Vector3d trans_base_to_imu(
        extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::IMU](0),
        extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::IMU](1),
        extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::IMU](2));
    Eigen::AngleAxisd rollAngle(
        extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::IMU](3),
        Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(
        extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::IMU](4),
        Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(
        extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::IMU](5),
        Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rot_base_to_imu =
        (rollAngle * pitchAngle * yawAngle).toRotationMatrix();

    // LOG_DEBUG(
    //     "robot to imu trans {}, \nrot \n{}", trans_base_to_imu.transpose(),
    //     rot_base_to_imu);

    Eigen::Vector3d acc(
        msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z);

    #if 0
    Eigen::Vector3d gyro(
        msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z);
    #else

    Eigen::Vector3d gyro(
        msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z);
    // gyro(0) = 0;
    // gyro(1) = 0;
    gyro = gyro * (M_PI / 180.);

    acc = rot_base_to_imu * acc;
    gyro = rot_base_to_imu * gyro;
    #endif

    if (b_imu_first_update_ == false) {
        // euler(2) = 0;
        // auto q2 = MATH::RPYToQuaternion(euler(0), euler(1), euler(2));
        // Sophus::SO3d so3(q2);
        // LOG_DEBUG(
        //     "roll: {}, pitch: {}, yaw: {}", euler.x() * 180. / M_PI,
        //     euler.y() * 180. / M_PI, euler.z() * 180. / M_PI);
        // ESKF::GetInstance().SetInitialRotation(so3);
        // Eigen::Vector3d bias_acc = acc + Eigen::Vector3d(0, 0, -9.81);
        // ESKF::GetInstance().SetInitialBias(acc, gyro);
        last_update_stamp_ = msg->header.stamp;
        b_imu_first_update_ = true;
    }

    auto total_nanoseconds = std::chrono::seconds(msg->header.stamp.sec) +
        std::chrono::nanoseconds(msg->header.stamp.nsec);
    ESKF::GetInstance().PushImuData(acc, gyro, total_nanoseconds);
#endif
}
}  // namespace ANSWER