#ifndef NC_MOTOR_TUNING_TESTER_HPP
#define NC_MOTOR_TUNING_TESTER_HPP

#include <ros/ros.h>
#include "util/logger.hpp"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <jsoncpp/json/json.h>
#include <boost/thread.hpp>
#include <string>
#include <vector>
#include <fstream>

#include "core_msgs/MotorInfo.h"


class MotorTuningTester {
public:
    MotorTuningTester();
    ~MotorTuningTester();

    void StartTest();
    void checkLowSpeedDriving(const Json::Value& scenario);
    void checkMidSpeedDriving(const Json::Value& scenario);
    void checkCasterLock(const Json::Value& scenario);
    void checkSuddenStartStop(const Json::Value& scenario);
    void checkRightRotation(const Json::Value& scenario); // Added for right rotation test
    void checkLeftRotation(const Json::Value& scenario);  // Added for left rotation test

    void MotorCallback(const core_msgs::MotorInfo::ConstPtr& msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void AnswerCallback(const std_msgs::String::ConstPtr& msg);
    void cmdVelocity(float linear_x, float angular_z);
    void cmdQuadVelocity(float linearVel1, float steerAngle1, float linearVel2, float steerAngle2);
    bool loadScenarios(const std::string& filename, Json::Value& scenarios);
    void executeScenario(const Json::Value& scenario);
    void saveResult();
    std::string getCurrentTime();

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher quad_cmd_pub_;
    ros::Subscriber motor_info_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber start_sub_;

    std::string package_path_;
    core_msgs::MotorInfo robot_motor_info_;
    nav_msgs::Odometry robot_odom_;
    double odom_time_;

    std::string scenario_start_;

    int n_kinematics_; // 0: DD, 1: QD, 2: SD, 3: OD
    std::string s_kinematics_type_;
    float f_dd_wheelbase_width_;
    std::string robot_name_;
    int n_motor_scenario_count_;
    std::vector<std::string> vec_failed_scenario_name_;
    std::vector<int> result_data_list_; // 0: fail, 1: pass, 2: error
    std::vector<std::string> result_error_list_;

    bool user_confirmed_;
    std::string user_input_;
    int cnt_; // Counter for test iterations

    // Motor feedback tracking
    float fl_rpm_min_, fl_rpm_max_;
    float fr_rpm_min_, fr_rpm_max_;
    float rl_rpm_min_, rl_rpm_max_;
    float rr_rpm_min_, rr_rpm_max_;
    float prev_current_fl = 0.0;
    float prev_current_fr = 0.0;
    float prev_current_rl = 0.0;
    float prev_current_rr = 0.0;
    int rpm_violation_count_;
    int current_violation_count_;

    // Thresholds (configurable via ROS params)
    float rpm_diff_threshold_; // Max allowed RPM difference
    float current_threshold_;  // Max allowed current
    int max_violations_;       // Max allowed threshold violations
    float stabilization_tolerance_; // 5% tolerance for stabilization
    float time_threshold_;     // Time limit for caster lock test

    std::deque<float> fl_rpm_window_, fr_rpm_window_, rl_rpm_window_, rr_rpm_window_; // rpm_슬라이딩 윈도우
    std::deque<float> fl_current_window_, fr_current_window_, rl_current_window_, rr_current_window_; // current_슬라이딩 윈도우
    const size_t window_size_rpm_ = 50; // 5초 (10Hz * 5)
    const size_t window_size_current_ = 30; // 3초 (10Hz * 3)
    bool stabilized_fl = false, stabilized_fr = false, stabilized_rl = false, stabilized_rr = false;
};

#endif // NC_MOTOR_TUNING_TESTER_HPP