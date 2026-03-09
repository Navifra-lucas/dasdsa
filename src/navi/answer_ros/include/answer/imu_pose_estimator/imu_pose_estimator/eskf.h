#pragma once
#include "common/callbacks.h"
#include "common/math.h"
#include "common/pch.h"
#include "common/pose2d.h"
using imu_data_t =
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, std::chrono::nanoseconds>;
namespace ANSWER {
class ESKF {
private:
    bool is_initialized_;
    bool online_calibration_;
    void LoadParameters();
    void ResetErrorState();
    void InjectErrorToNominalState();

    std::string config_path_;
    Eigen::Vector3d acc_noise_;
    Eigen::Vector3d gyro_noise_;
    Eigen::Vector3d acc_bias_;
    Eigen::Vector3d gyro_bias_;
    double update_rate_;

    Eigen::Vector3d nominal_position_;
    Eigen::Vector3d nominal_velocity_;
    Eigen::Vector3d nominal_orientation_;
    Sophus::SO3d nominal_rot_mat_;
    Eigen::Quaterniond nominal_rot_;
    Eigen::Vector3d nominal_acc_bias_;
    Eigen::Vector3d nominal_gyro_bias_;
    Eigen::Vector3d nominal_gravity_;
    Eigen::Matrix<double, 18, 1> error_state_;

    Eigen::MatrixXd P_;
    Eigen::MatrixXd Fx_;
    Eigen::MatrixXd Fi_;
    Eigen::MatrixXd Hx_;
    Eigen::MatrixXd Qi_;
    Eigen::MatrixXd G_;
    Eigen::MatrixXd measurement_noise_;

    std::chrono::time_point<std::chrono::system_clock> prev_time_;
    std::chrono::time_point<std::chrono::system_clock> last_predict_time_;

    Sophus::SE3d reference_frame3d_;
    Sophus::SE2d reference_frame2d_;

    Sophus::SE2d prior_pose_;
    Sophus::SE2d imu_pose_;
    std::mutex state_mutex_;
    std::mutex data_mutex_;

    std::deque<imu_data_t> imu_data_queue_;
    void SetReferenceFrame(const Sophus::SE3d &reference_frame);
    void SetReferenceFrame(const Sophus::SE2d &reference_frame);

    void Predict(
        const Eigen::Vector3d &acc_last, const Eigen::Vector3d &gyro_last,
        const Eigen::Vector3d &acc_cur, const Eigen::Vector3d &gyro_cur,
        double dt, const bool is_median = false);

public:
    ESKF();
    ~ESKF();

    static ESKF &GetInstance()
    {
        static ESKF ins;
        return ins;
    }
    void SetInitPose(const Pose2D &pose);
    void Initialize(const std::string &config_path);
    void PushImuData(
        const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
        const std::chrono::nanoseconds &stamp);
    void Predict(const std::chrono::nanoseconds &stamp);
    void Update(const Sophus::SE3d &pose, const Eigen::MatrixXd &covariance);
    void Update2D(const Pose2D &pose);
    void SetInitialBias(
        const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyro_bias)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        nominal_acc_bias_ = acc_bias;
        nominal_gyro_bias_ = gyro_bias;
    }
    void SetInitialRotation(const Sophus::SO3d &rotation)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        nominal_rot_mat_ = rotation;
    }
    const Sophus::SE3d GetState3D();
    const Sophus::SE2d GetState2D();
    bool InitializeIMUIntrinsic(const std::deque<imu_data_t> &imu_data_queue);
};  // namespace ESKF
}  // namespace ANSWER