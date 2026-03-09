#ifndef NC_MOTOR_SEMICAL_TESTER_HPP
#define NC_MOTOR_SEMICAL_TESTER_HPP

#include "util/logger.hpp"

#include <Poco/Dynamic/Var.h>
#include <core/util/logger.hpp>
#include <core_msgs/CommonString.h>
#include <core_msgs/EtcInfo.h>
// #include <core_msgs/MotionInfo.h>
#include <core_msgs/MotorInfo.h>
#include <core_msgs/NavicoreStatus.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

class MotorTester {
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_;

    ros::Publisher param_pub_;
    ros::Publisher nav_cmd_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher quad_cmd_pub_;

    ros::Subscriber motor_info_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber front_scan_sub_;
    ros::Subscriber rear_scan_sub_;
    ros::Subscriber left_scan_sub_;
    ros::Subscriber right_scan_sub_;
    ros::Subscriber start_sub_;

    std::string package_path_;

private:
    void StartTest();
    
    // 콜백 함수
    void MotorCallback(const core_msgs::MotorInfo::ConstPtr& msg);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void FrontLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void RearLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void LeftLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void RightLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void AnswerCallback(const std_msgs::String::ConstPtr& msg);

    // 테스트 메서드
    void checkFLTractionMotor();
    void checkRRTractionMotor();
    void checkFLSteeringMotor();
    void checkRRSteeringMotor();
    void checkFrontLidar();
    void checkRearLidar();
    void checkLeftLidar();
    void checkRightLidar();

    // 유틸리티 함수
    void MotorErrorCheck();
    void cmdVelocity(float linear_x, float angular_z);
    void cmdQuadVelocity(float linearVel1, float steerAngle1, float linearVel2, float steerAngle2);
    void saveResult();
    void loadResult();

    // 멤버 변수
    core_msgs::MotorInfo robot_motor_info_;
    nav_msgs::Odometry robot_odom_;
    double odom_time_ = 0;
    int n_kinematics_ = 0;
    std::string s_kinematics_type_;
    std::string robot_name_ = "unknown";
    bool b_fl_wheel_real_dir_switch, b_rr_wheel_real_dir_switch = true;
    bool b_fl_wheel_ui_dir_switch, b_rr_wheel_ui_dir_switch = true;
    bool b_fl_wheel_enc_dir_switch, b_rr_wheel_enc_dir_switch = true;
    bool b_fl_steer_real_dir_switch, b_rr_steer_real_dir_switch = true;
    bool b_fl_steer_ui_dir_switch, b_rr_steer_ui_dir_switch = true;
    int input_lidars_ = 0;

    // 모터 테스트 변수
    int cnt_ = 0;
    float fl_encoder_ = 0, rr_encoder_ = 0;
    float fl_encoder_past_ = 0, rr_encoder_past_ = 0;
    float fl_angle_past_ = 0, rr_angle_past_ = 0;
    double fl_traction_target_time_ = 0, fl_traction_feedback_time_ = 0;
    double rr_traction_target_time_ = 0, rr_traction_feedback_time_ = 0;
    double fl_steer_target_time_ = 0, fl_steer_feedback_time_ = 0;
    double rr_steer_target_time_ = 0, rr_steer_feedback_time_ = 0;

    // 라이다 테스트 변수
    int front_laser_count_ = 0, rear_laser_count_ = 0, left_laser_count_ = 0, right_laser_count_ = 0;
    size_t front_laser_ranges_size_ = 0, rear_laser_ranges_size_ = 0;
    size_t left_laser_ranges_size_ = 0, right_laser_ranges_size_ = 0;
    bool lidar_data_check_ = false;

    // 결과 저장 변수
    std::vector<int> result_data_list_ = {0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<std::string> result_error_list_ = {"", "", "", "", "", "", "", ""};

    std::string getCurrentTime();
    int n_motor_scenario_count_ = 0;
    std::vector<std::string> vec_failed_scenario_name_;

    // 사용자 입력
    std::string scenario_start_;
    bool user_confirmed = false;

public:
    MotorTester();
    virtual ~MotorTester();
};
;

#endif
