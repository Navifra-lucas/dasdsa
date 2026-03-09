#pragma once
#include "common/3d/scan3d.h"
#include "common/math.h"
#include "common/pose2d.h"
#include "common/scan2d.h"
#include "common/types.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ANSWER {

template <typename ros_point_cloud_ptr, typename Scan2D>
static inline Scan2D ConvertROSToScan2D(const ros_point_cloud_ptr &msg)
{
    Scan2D scan;

    for (auto &point : msg->points) {
        scan.point_cloud.emplace_back(Point2D(point.x, point.y));
    }

    return scan;
}

template <typename ros_odomtery_ptr, typename Pose2D>
static inline Pose2D ConvertROSToOdom2D(const ros_odomtery_ptr &msg)
{
    Pose2D pose;

    auto rpy = MATH::Quaternion2RPY(Eigen::Quaternionf(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

    pose.SetPose(Eigen::Vector3f(
        msg->pose.pose.position.x, msg->pose.pose.position.y, rpy.z()));

    return pose;
}

template <typename ros_pose_with_covariance_stamped_ptr, typename Pose2D>
static inline Pose2D ConvertROSToPose2D(
    const ros_pose_with_covariance_stamped_ptr &msg)
{
    Pose2D pose;

    auto rpy = MATH::Quaternion2RPY(Eigen::Quaternionf(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

    pose.SetPose(Eigen::Vector3f(
        msg->pose.pose.position.x, msg->pose.pose.position.y, rpy.z()));

    return pose;
}

template <typename ros_pose_with_covariance_stamped, typename Pose2D>
static inline ros_pose_with_covariance_stamped ConvertPose2DToROS(
    const Pose2D &pose)
{
    ros_pose_with_covariance_stamped pose_msg;

    auto q = MATH::RPYToQuaternion(0, 0, pose.GetPose().z());
    pose_msg.pose.pose.position.x = pose.GetPose().x();
    pose_msg.pose.pose.position.y = pose.GetPose().y();
    pose_msg.pose.pose.position.z = 0;
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    return pose_msg;
}
template <typename ros_laser_scan_ptr, typename Scan2D>
static inline Scan2D ConvertROS1LaserScanToScan2D(ros_laser_scan_ptr msg)
{
    Scan2D scan;
    scan.origin = Eigen::Vector3f(0, 0, 0);
    scan.point_cloud.reserve(msg->ranges.size());
    scan.ranges.reserve(msg->ranges.size());
    scan.intensities.reserve(msg->ranges.size());

    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.angle_increment = msg->angle_increment;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;
    scan.secs = msg->header.stamp.sec;
    scan.nsecs = msg->header.stamp.nsec;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;
        float range = msg->ranges[i];
        if (range < msg->range_min || range > msg->range_max) {
            range = 0;
        }
        float x = range * cosf(angle);
        float y = range * sinf(angle);
        scan.point_cloud.emplace_back(Eigen::Vector2f(x, y));
        scan.intensities.emplace_back(msg->intensities[i]);
        scan.ranges.emplace_back(range);
    }

    return scan;
}
template <typename ros_laser_scan_ptr, typename Scan2D>
static inline Scan2D ConvertROS2LaserScanToScan2D(ros_laser_scan_ptr msg)
{
    Scan2D scan;
    scan.origin = Eigen::Vector3f(0, 0, 0);
    scan.point_cloud.reserve(msg->ranges.size());
    scan.ranges.reserve(msg->ranges.size());
    scan.intensities.reserve(msg->ranges.size());

    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.angle_increment = msg->angle_increment;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;
    scan.secs = msg->header.stamp.sec;
    scan.nsecs = msg->header.stamp.nanosec;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;
        float range = msg->ranges[i];
        if (range < msg->range_min || range > msg->range_max) {
            range = 0;
        }
        float x = range * cosf(angle);
        float y = range * sinf(angle);
        scan.point_cloud.emplace_back(Eigen::Vector2f(x, y));
        scan.intensities.emplace_back(msg->intensities[i]);
        scan.ranges.emplace_back(range);
    }

    return scan;
}
template <
    typename ros_point_cloud_2_ptr, typename ros_point_cloud_2_iterator_double,
    typename ros_point_cloud_2_iterator_float,
    typename ros_point_cloud_2_iterator_uint, typename ros_point_field,
    typename Scan3D>
static inline Scan3D ConvertROSToScan3D(const ros_point_cloud_2_ptr &msg)
{
    Scan3D scan;
    scan.SetTimestamp(msg->header.stamp.toSec());
    ros_point_cloud_2_iterator_double msg_x(*msg, "x");
    ros_point_cloud_2_iterator_double msg_y(*msg, "y");
    ros_point_cloud_2_iterator_double msg_z(*msg, "z");

    for (size_t i = 0; i < msg->height * msg->width;
         ++i, ++msg_x, ++msg_y, ++msg_z) {
        Eigen::Vector3d point_in_lidar_frame(*msg_x, *msg_y, *msg_z);
        // Skip invalid points
        if (std::isfinite(point_in_lidar_frame(0)) &&
            std::isfinite(point_in_lidar_frame(1)) &&
            std::isfinite(point_in_lidar_frame(2)) &&
            std::isnan(point_in_lidar_frame(0)) == false &&
            std::isnan(point_in_lidar_frame(1)) == false &&
            std::isnan(point_in_lidar_frame(2)) == false) {
            scan.MutablePointCloud().emplace_back(point_in_lidar_frame);
            scan.MutablePointCloudHomogeneous().emplace_back(Eigen::Vector4d(
                point_in_lidar_frame(0), point_in_lidar_frame(1),
                point_in_lidar_frame(2), 1.0));
        }
    }

    ros_point_field timestamp_field;
    for (const auto &field : msg->fields) {
        if ((field.name == "t" || field.name == "timestamp" ||
             field.name == "time")) {
            timestamp_field = field;
            // std::cout << "Found timestamp field: " <<
            // field.name << std::endl;
        }
    }
    auto extract_timestamp = [&msg]<typename T>(T iterator) {
        const size_t n_points = msg->height * msg->width;
        std::vector<double> timestamps;
        timestamps.reserve(n_points);
        for (size_t i = 0; i < n_points; ++i, ++iterator) {
            timestamps.emplace_back(static_cast<double>(*iterator));
        }
        return MATH::NormalizeVectord(timestamps);
    };
    if (timestamp_field.datatype == ros_point_field::FLOAT32) {
        scan.SetTimestamps(extract_timestamp(
            ros_point_cloud_2_iterator_float(*msg, timestamp_field.name)));
    }
    else if (timestamp_field.datatype == ros_point_field::FLOAT64) {
        scan.SetTimestamps(extract_timestamp(
            ros_point_cloud_2_iterator_double(*msg, timestamp_field.name)));
    }
    else if (timestamp_field.datatype == ros_point_field::UINT32) {
        scan.SetTimestamps(extract_timestamp(
            ros_point_cloud_2_iterator_uint(*msg, timestamp_field.name)));
    }
    // for (auto &stamp : scan.GetTimestamps()) {
    //     std::cout << stamp << " ";
    // }

    return scan;
}

template <
    typename ros_point_cloud_2_ptr, typename ros_point_cloud_2_iterator_double,
    typename ros_point_cloud_2_iterator_float,
    typename ros_point_cloud_2_iterator_uint, typename ros_point_field,
    typename Scan3D>
static inline Scan3D ConvertROS2ToScan3D(const ros_point_cloud_2_ptr &msg)
{
    Scan3D scan;

    ros_point_field timestamp_field;
    int data_type = 0;
    for (const auto &field : msg->fields) {
        if ((field.name == "t" || field.name == "timestamp" ||
             field.name == "time")) {
            timestamp_field = field;
            // std::cout << "Found timestamp field: " <<
            // field.name << std::endl;
        }
        if (field.name == "x") {
            data_type = field.datatype;
        }
    }
    // LOG_INFO("Point cloud data type: {}", data_type);
    auto extract_timestamp = [&msg]<typename T>(T iterator) {
        const size_t n_points = msg->height * msg->width;
        std::vector<double> timestamps;
        timestamps.reserve(n_points);
        for (size_t i = 0; i < n_points; ++i, ++iterator) {
            timestamps.emplace_back(static_cast<double>(*iterator));
        }
        return MATH::NormalizeVectord(timestamps);
    };
    if (timestamp_field.datatype == ros_point_field::FLOAT32) {
        scan.SetTimestamps(extract_timestamp(
            ros_point_cloud_2_iterator_float(*msg, timestamp_field.name)));
    }
    else if (timestamp_field.datatype == ros_point_field::FLOAT64) {
        scan.SetTimestamps(extract_timestamp(
            ros_point_cloud_2_iterator_double(*msg, timestamp_field.name)));
    }
    else if (timestamp_field.datatype == ros_point_field::UINT32) {
        scan.SetTimestamps(extract_timestamp(
            ros_point_cloud_2_iterator_uint(*msg, timestamp_field.name)));
    }
    scan.SetTimestamp(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

    if (data_type == ros_point_field::FLOAT32) {
        ros_point_cloud_2_iterator_float msg_x(*msg, "x");
        ros_point_cloud_2_iterator_float msg_y(*msg, "y");
        ros_point_cloud_2_iterator_float msg_z(*msg, "z");

        for (size_t i = 0; i < msg->height * msg->width;
             ++i, ++msg_x, ++msg_y, ++msg_z) {
            Eigen::Vector3d point_in_lidar_frame(
                static_cast<double>(*msg_x), static_cast<double>(*msg_y),
                static_cast<double>(*msg_z));
            // Skip invalid points
            if (std::isfinite(point_in_lidar_frame(0)) &&
                std::isfinite(point_in_lidar_frame(1)) &&
                std::isfinite(point_in_lidar_frame(2)) &&
                std::isnan(point_in_lidar_frame(0)) == false &&
                std::isnan(point_in_lidar_frame(1)) == false &&
                std::isnan(point_in_lidar_frame(2)) == false) {
                scan.MutablePointCloud().emplace_back(point_in_lidar_frame);
                scan.MutablePointCloudHomogeneous().emplace_back(
                    Eigen::Vector4d(
                        point_in_lidar_frame(0), point_in_lidar_frame(1),
                        point_in_lidar_frame(2), 1.0));
            }
        }
    }
    else if (data_type == ros_point_field::FLOAT64) {
        ros_point_cloud_2_iterator_double msg_x(*msg, "x");
        ros_point_cloud_2_iterator_double msg_y(*msg, "y");
        ros_point_cloud_2_iterator_double msg_z(*msg, "z");

        for (size_t i = 0; i < msg->height * msg->width;
             ++i, ++msg_x, ++msg_y, ++msg_z) {
            Eigen::Vector3d point_in_lidar_frame(*msg_x, *msg_y, *msg_z);
            // Skip invalid points
            if (std::isfinite(point_in_lidar_frame(0)) &&
                std::isfinite(point_in_lidar_frame(1)) &&
                std::isfinite(point_in_lidar_frame(2)) &&
                std::isnan(point_in_lidar_frame(0)) == false &&
                std::isnan(point_in_lidar_frame(1)) == false &&
                std::isnan(point_in_lidar_frame(2)) == false) {
                scan.MutablePointCloud().emplace_back(point_in_lidar_frame);

                scan.MutablePointCloudHomogeneous().emplace_back(
                    Eigen::Vector4d(
                        point_in_lidar_frame(0), point_in_lidar_frame(1),
                        point_in_lidar_frame(2), 1.0));
            }
        }
    }
    return scan;
}
template <typename ros_imu_ptr, typename IMUData>
static inline IMUData ConvertROSToIMUData(
    const ros_imu_ptr &msg, double time_offset = 0.0)
{
    IMUData imu_data;
    double timestamp = msg->header.stamp.toSec();
    imu_data.timestamp = timestamp + time_offset;
    imu_data.angular_velocity << msg->angular_velocity.x,
        msg->angular_velocity.y, msg->angular_velocity.z;

    imu_data.linear_acceleration << msg->linear_acceleration.x,
        msg->linear_acceleration.y, msg->linear_acceleration.z;
    return imu_data;
}

template <typename ros_imu_ptr, typename IMUData>
static inline IMUData ConvertROS2ToIMUData(
    const ros_imu_ptr &msg, double time_offset = 0.0)
{
    IMUData imu_data;
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    imu_data.timestamp = timestamp + time_offset;
    imu_data.angular_velocity << msg->angular_velocity.x,
        msg->angular_velocity.y, msg->angular_velocity.z;

    imu_data.linear_acceleration << msg->linear_acceleration.x,
        msg->linear_acceleration.y, msg->linear_acceleration.z;
    return imu_data;
}
}  // namespace ANSWER