#pragma once

#include "common/callbacks.h"
#include "common/math.h"
#include "common/pch.h"
#include "common/pose2d.h"
namespace ANSWER {
class EKF {
private:
    /* data */
    Eigen::Matrix<double, 6, 1>
        state_;  // State vector: [x, y, yaw, vx, vy, omega(angular velocity)]
    Eigen::Matrix<double, 6, 6> P_;  // Covariance matrix
    std::chrono::nanoseconds last_stamp_;
    Eigen::Matrix<double, 3, 3> measurement_noise_;  // Measurement noise matrix
    Eigen::Matrix<double, 6, 6> process_noise_;  // Process noise matrix

    std::mutex state_update_mutex_;

public:
    EKF(/* args */);
    ~EKF();

    static EKF &GetInstance()
    {
        static EKF ins;
        return ins;
    }
    void SetInitPose(const Pose2D &pose);
    void Initialize(const std::string &config_path);
    void PushImuData(
        const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
        const std::chrono::nanoseconds &stamp);
    void Predict(
        const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
        const std::chrono::nanoseconds &stamp);
    // void Update(const Pose2D &measurement);
    void Update2D(const Pose2D &measurement);
    const Pose2D GetState2D();
    // bool InitializeIMUIntrinsic(const std::deque<imu_data_t>
    // &imu_data_queue);
};

}  // namespace ANSWER