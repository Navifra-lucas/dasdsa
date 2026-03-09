#pragma once
#include "common/3d/scan3d.h"

#include <sophus/se2.h>
#include <sophus/se3.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <deque>
#include <mutex>
#include <vector>

#define STRINGFY(T) (#T)
namespace ANSWER {
using Pose3D = Sophus::SE3d;
using Point3D = Eigen::Vector3d;
using PointCloud3D = std::vector<Point3D>;
using ClusterIndices = std::vector<int>;
using Point2D = Eigen::Vector2f;
using PointCloud2D = std::vector<Point2D, Eigen::aligned_allocator<Point2D>>;
struct IMUData {
    double timestamp;
    double dt;
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};
struct IMUNoise {
    Eigen::Vector3d angle_randomwalk = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_randomwalk = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyrbias_std = Eigen::Vector3d::Zero();
    // gyro bias standard error
    Eigen::Vector3d accbias_std = Eigen::Vector3d::Zero();
    // accelerometer bias standard error
    double correlation_time;
};
struct IMUError {
    Eigen::Vector3d gyrbias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accbias = Eigen::Vector3d::Zero();
};

struct BodyState {
    Pose3D pose = Pose3D();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    IMUError imu_error;
};
struct IMUDataQueue {
private:
    std::deque<IMUData> imu_data_deque;

public:
    std::mutex mtx_;
    std::deque<IMUData> GetDataAndClear()
    {
        std::deque<IMUData> temp;
        std::lock_guard<std::mutex> lock(mtx_);
        temp.swap(imu_data_deque);
        return temp;
    }
    void Clear()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_data_deque.clear();
    }
    void PushBack(const IMUData &data)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_data_deque.push_back(data);
    }
    bool Empty()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return imu_data_deque.empty();
    }
    std::deque<IMUData> &GetData() { return imu_data_deque; }
};

struct Scan3DQueue {
private:
    std::deque<Scan3D> scan3d_deque;

public:
    std::mutex mtx_;
    std::deque<Scan3D> GetDataAndClear()
    {
        std::deque<Scan3D> temp;
        std::lock_guard<std::mutex> lock(mtx_);
        temp.swap(scan3d_deque);
        return temp;
    }
    void Clear()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        scan3d_deque.clear();
    }
    void PushBack(const Scan3D &data)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        scan3d_deque.push_back(data);
    }
    bool Empty()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return scan3d_deque.empty();
    }

    std::deque<Scan3D> &GetData() { return scan3d_deque; }
};

}  // namespace ANSWER