#include "nc_motor_tuning_tester.hpp"
#include <ros/package.h>
#include <chrono>
#include <sstream>
#include <cmath>
#include <deque>
#include <numeric>
#include <std_msgs/String.h>

MotorTuningTester::MotorTuningTester() : nh_("~"), n_motor_scenario_count_(0), user_confirmed_(false), cnt_(0) {
    package_path_ = ros::package::getPath("nc_motor_tester") + "/launch";
    
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    quad_cmd_pub_ = nh_.advertise<std_msgs::String>("/quad_cmd", 1, true);

    motor_info_sub_ = nh_.subscribe("/motor_info", 10, &MotorTuningTester::MotorCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &MotorTuningTester::OdomCallback, this);
    start_sub_ = nh_.subscribe("/nc_motor_tester/start", 10, &MotorTuningTester::AnswerCallback, this);

    // Initialize parameters from ROS params
    ros::param::param<int>("/motion_base/n_kinematics_type", n_kinematics_, 0);
    // ros::param::param<float>("driver_dd/f_dd_FL_wheel_diameter_m", f_dd_wheelbase_width_, 0.17);

    // Initialize result lists
    result_data_list_.resize(6, 0); // Updated to 6 tests: low-speed, mid-speed, caster lock, sudden start/stop, right rotation, left rotation
    result_error_list_.resize(6, "NOT_RUN");

    // Set kinematics type
    if (n_kinematics_ == 0) s_kinematics_type_ = "DD";
    else if (n_kinematics_ == 1) s_kinematics_type_ = "QD";
    else if (n_kinematics_ == 2) s_kinematics_type_ = "SD";
    else if (n_kinematics_ == 3) s_kinematics_type_ = "OD";

    // Start test in a separate thread
    boost::thread th1 = boost::thread(boost::bind(&MotorTuningTester::StartTest, this));
}

MotorTuningTester::~MotorTuningTester() {}

void MotorTuningTester::checkLowSpeedDriving(const Json::Value& scenario) {
    NLOG(info) <<"Checking Low-Speed Driving (0.02 m/s)...: " << s_kinematics_type_;
    // 테스트 파라미터 설정
    float target_speed = 0.02; // 주행 속도: 0.02 m/s (변경 하면 안됨.)
    float target_distance = 0.5; // 주행 거리: 0.5 m (변경 가능. 다만 속도가 느리니 너무 길게 설정하지 말 것.)
    // 변수 초기화
    double start_time = 0.0;
    bool start_time_set = false;
    double stabilized_time = 0.0;
    bool stabilized = false;

    // Load parameters from JSON
    float rpm_std_dev_threshold = scenario["parameters"].isMember("rpm_std_dev_threshold") ? 
                                      scenario["parameters"]["rpm_std_dev_threshold"].asFloat() : 50.0;
    int max_violations = scenario["parameters"].isMember("max_violations") ? 
                         scenario["parameters"]["max_violations"].asInt() : 5;
    float stabilization_time_threshold = scenario["parameters"].isMember("stabilization_time_threshold") ? 
                                         scenario["parameters"]["stabilization_time_threshold"].asFloat() : 2.0;
    if (cnt_ == 0) {
        NLOG(info) << "저속 주행(0.02m/s) 테스트를 시작합니다...";
        NLOG(info) << "/nc_motor_tester/start 토픽을 통해 true 를 날리면 시작하겠습니다...";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed_ = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋
    }

    ros::Rate rate(10); // 10Hz
    
    double start_x = robot_odom_.pose.pose.position.x;
    double start_y = robot_odom_.pose.pose.position.y;
    double distance_traveled = 0.0;
    rpm_violation_count_ = 0;

    // 슬라이딩 윈도우 초기화
    fl_rpm_window_.clear();
    fr_rpm_window_.clear();
    rl_rpm_window_.clear();
    rr_rpm_window_.clear();
    
    if (user_confirmed_) {
        while (distance_traveled < target_distance && ros::ok()) {
            // 1. Command velocity
            if (n_kinematics_ == 0) {
                cmdVelocity(target_speed, 0.0);
            } else {
                cmdQuadVelocity(target_speed, 0.0, target_speed, 0.0);
            }

            if (!start_time_set) {
                start_time = ros::Time::now().toSec();
                start_time_set = true;
            }

            // 2. 거리 측정
            double current_x = robot_odom_.pose.pose.position.x;
            double current_y = robot_odom_.pose.pose.position.y;
            distance_traveled = std::hypot(current_x - start_x, current_y - start_y);
            
            // 3. 최초 안정화 체크
            if (!stabilized) {
                // 저속 주행이기 때문에 타겟 rpm 이 작아 피드백 rpm 이 타겟 rpm 의 95% 이상 되면 안정화라고 판단.
                bool fl_stable = robot_motor_info_.f_FL_traction_motor_feedback_rpm >= robot_motor_info_.f_FL_traction_motor_target_rpm * 0.95;
                bool fr_stable = robot_motor_info_.f_FR_traction_motor_feedback_rpm >= robot_motor_info_.f_FR_traction_motor_target_rpm * 0.95;
                bool rl_stable = robot_motor_info_.f_RL_traction_motor_feedback_rpm >= robot_motor_info_.f_RL_traction_motor_target_rpm * 0.95;
                bool rr_stable = robot_motor_info_.f_RR_traction_motor_feedback_rpm >= robot_motor_info_.f_RR_traction_motor_target_rpm * 0.95;
                
                if (n_kinematics_ == 0 || n_kinematics_ == 1) // DD or QD
                {
                    if (fl_stable && fr_stable)
                    {
                        stabilized = true;
                        stabilized_time = ros::Time::now().toSec();
                        double response_time = stabilized_time - start_time;
                        NLOG(info) << "[Low Speed] RPM Stabilized after: " << response_time << " sec";
                        // rpm 반응성 판단
                        if (response_time > stabilization_time_threshold) {
                            rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                            NLOG(info) << "반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                        }
                    }
                }
                else if (n_kinematics_ == 2) // SD
                {
                    if (fl_stable)
                    {
                        stabilized = true;
                        stabilized_time = ros::Time::now().toSec();
                        double response_time = stabilized_time - start_time;
                        NLOG(info) << "[Low Speed] RPM Stabilized after: " << response_time << " sec";
                        // rpm 반응성 판단
                        if (response_time > stabilization_time_threshold) {
                            rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                            NLOG(info) << "반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                        }
                    }
                }
                else if (n_kinematics_ == 3) // OD
                {
                    if (fl_stable && fr_stable && rl_stable && rr_stable)
                    {
                        stabilized = true;
                        stabilized_time = ros::Time::now().toSec();
                        double response_time = stabilized_time - start_time;
                        NLOG(info) << "[Low Speed] RPM Stabilized after: " << response_time << " sec";
                        // rpm 반응성 판단
                        if (response_time > stabilization_time_threshold) {
                            rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                            NLOG(info) << "반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                        }
                    }
                }

                fl_rpm_window_.push_back(robot_motor_info_.f_FL_traction_motor_feedback_rpm);
                fr_rpm_window_.push_back(robot_motor_info_.f_FR_traction_motor_feedback_rpm);
                rl_rpm_window_.push_back(robot_motor_info_.f_RL_traction_motor_feedback_rpm);
                rr_rpm_window_.push_back(robot_motor_info_.f_RR_traction_motor_feedback_rpm);

                if (fl_rpm_window_.size() > window_size_rpm_) fl_rpm_window_.pop_front();
                if (fr_rpm_window_.size() > window_size_rpm_) fr_rpm_window_.pop_front();
                if (rl_rpm_window_.size() > window_size_rpm_) rl_rpm_window_.pop_front();
                if (rr_rpm_window_.size() > window_size_rpm_) rr_rpm_window_.pop_front();
                // 표준편차 계산
                if (fl_rpm_window_.size() == window_size_rpm_) {
                    auto calc_std_dev = [](const std::deque<float>& data) {
                        float mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
                        float sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
                        return std::sqrt(sq_sum / data.size() - mean * mean);
                    };
                    float fl_std_dev = calc_std_dev(fl_rpm_window_);
                    float fr_std_dev = calc_std_dev(fr_rpm_window_);
                    float rl_std_dev = calc_std_dev(rl_rpm_window_);
                    float rr_std_dev = calc_std_dev(rr_rpm_window_);
                    // 위반 체크
                    if (fl_std_dev > rpm_std_dev_threshold || fr_std_dev > rpm_std_dev_threshold ||
                        rl_std_dev > rpm_std_dev_threshold || rr_std_dev > rpm_std_dev_threshold) {
                        rpm_violation_count_++;
                        NLOG(info) << "(low)rpm_평균치: "
                        << "FL: " << fl_std_dev << ", FR: " << fr_std_dev 
                        << ", RL: " <<rl_std_dev <<", RR: " << rr_std_dev 
                        << " / Threshold:  " << rpm_std_dev_threshold;
                    }
                    NLOG(info) << "(low)rpm_평균치: "
                        << "FL: " << fl_std_dev << ", FR: " << fr_std_dev 
                        << ", RL: " <<rl_std_dev <<", RR: " << rr_std_dev 
                        << " / Threshold:  " << rpm_std_dev_threshold;
                }
            }
            cnt_++;
            rate.sleep();
        }
    } else {
        ROS_INFO("User did not confirm. Skipping test.");
    }

    // Stop robot
    cmdVelocity(0.0, 0.0);
    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

    // Evaluate results
    if (user_confirmed_ && rpm_violation_count_ <= max_violations) {
        NLOG(info) << "저속 주행 테스트 통과";
        result_data_list_[0] = 1;
        result_error_list_[0] = "OK";
    } else {
        NLOG(info) << "저속 주행 테스트 실패";
        result_data_list_[0] = 2;
        result_error_list_[0] = rpm_violation_count_ > max_violations ? "RPM TUNING ERROR" : "USER CANCELLED";
        vec_failed_scenario_name_.emplace_back("low_speed_driving");
        NLOG(info) << "--- 모터 떨림이 감지되었습니다. 튜닝이 필요합니다. ---";
    }

    // Reset variables
    cnt_ = 0;
    user_confirmed_ = false;
    saveResult();
    NLOG(info) << "Low-speed driving test end: " << rpm_violation_count_ << " / " << max_violations;
}

void MotorTuningTester::checkMidSpeedDriving(const Json::Value& scenario) {
    ROS_INFO("Checking Mid-Speed Driving (0.5 m/s)...");
    // 테스트 파라미터 설정
    float target_speed = 0.5;
    float target_distance = 0.5;
    // 변수 초기화
    double start_time = 0.0;
    bool start_time_set = false;
    double stabilized_time = 0.0;
    bool stabilized = false;

    // Load parameters from JSON
    float rpm_std_dev_threshold = scenario["parameters"].isMember("rpm_std_dev_threshold") ? 
                                      scenario["parameters"]["rpm_std_dev_threshold"].asFloat() : 50.0;
    int max_violations = scenario["parameters"].isMember("max_violations") ? 
                         scenario["parameters"]["max_violations"].asInt() : 5;
    float stabilization_time_threshold = scenario["parameters"].isMember("stabilization_time_threshold") ? 
                                         scenario["parameters"]["stabilization_time_threshold"].asFloat() : 2.0;    
    if (cnt_ == 0) {
        NLOG(info) << "중속 주행(0.5m/s) 테스트를 시작합니다...";
        NLOG(info) << "/nc_motor_tester/start 토픽을 통해 true 를 날리면 시작하겠습니다...";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed_ = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋
    }

    ros::Rate rate(10); // 10Hz

    double start_x = robot_odom_.pose.pose.position.x;
    double start_y = robot_odom_.pose.pose.position.y;
    double distance_traveled = 0.0;
    rpm_violation_count_ = 0;

    // 슬라이딩 윈도우 초기화
    fl_rpm_window_.clear();
    fr_rpm_window_.clear();
    rl_rpm_window_.clear();
    rr_rpm_window_.clear();

    if (user_confirmed_) {
        while (distance_traveled < target_distance && ros::ok()) {
            // 1. Command velocity
            if (n_kinematics_ == 0) {
                cmdVelocity(target_speed, 0.0);
            } else {
                cmdQuadVelocity(target_speed, 0.0, target_speed, 0.0);
            }

            if (!start_time_set) {
                start_time = ros::Time::now().toSec();
                start_time_set = true;
            }

            // 2. 거리 측정
            double current_x = robot_odom_.pose.pose.position.x;
            double current_y = robot_odom_.pose.pose.position.y;
            distance_traveled = std::hypot(current_x - start_x, current_y - start_y);

            // 3. 최초 안정화 체크
            if (!stabilized) {
                
                bool fl_stable = robot_motor_info_.f_FL_traction_motor_feedback_rpm >= robot_motor_info_.f_FL_traction_motor_target_rpm * 0.98;
                bool fr_stable = robot_motor_info_.f_FR_traction_motor_feedback_rpm >= robot_motor_info_.f_FR_traction_motor_target_rpm * 0.98;
                bool rl_stable = robot_motor_info_.f_RL_traction_motor_feedback_rpm >= robot_motor_info_.f_RL_traction_motor_target_rpm * 0.98;
                bool rr_stable = robot_motor_info_.f_RR_traction_motor_feedback_rpm >= robot_motor_info_.f_RR_traction_motor_target_rpm * 0.98;
                
                if (n_kinematics_ == 0 || n_kinematics_ == 1) // DD or QD
                {
                    if (fl_stable && fr_stable)
                    {
                        stabilized = true;
                        stabilized_time = ros::Time::now().toSec();
                        double response_time = stabilized_time - start_time;
                        NLOG(info) << "[Mid Speed] RPM Stabilized after: " << response_time << " sec";
                        // rpm 반응성 판단
                        if (response_time > stabilization_time_threshold) {
                            rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                            NLOG(info) << "[Mid Speed] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                        }
                    }
                }
                else if (n_kinematics_ == 2) // SD
                {
                    if (fl_stable)
                    {
                        stabilized = true;
                        stabilized_time = ros::Time::now().toSec();
                        double response_time = stabilized_time - start_time;
                        NLOG(info) << "[Mid Speed] RPM Stabilized after: " << response_time << " sec";
                        // rpm 반응성 판단
                        if (response_time > stabilization_time_threshold) {
                            rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                            NLOG(info) << "[Mid Speed] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                        }
                    }
                }
                else if (n_kinematics_ == 3) // OD
                {
                    if (fl_stable && fr_stable && rl_stable && rr_stable)
                    {
                        stabilized = true;
                        stabilized_time = ros::Time::now().toSec();
                        double response_time = stabilized_time - start_time;
                        NLOG(info) << "[Mid Speed] RPM Stabilized after: " << response_time << " sec";
                        // rpm 반응성 판단
                        if (response_time > stabilization_time_threshold) {
                            rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                            NLOG(info) << "[Mid Speed] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                        }
                    }
                }

                fl_rpm_window_.push_back(robot_motor_info_.f_FL_traction_motor_feedback_rpm);
                fr_rpm_window_.push_back(robot_motor_info_.f_FR_traction_motor_feedback_rpm);
                rl_rpm_window_.push_back(robot_motor_info_.f_RL_traction_motor_feedback_rpm);
                rr_rpm_window_.push_back(robot_motor_info_.f_RR_traction_motor_feedback_rpm);

                if (fl_rpm_window_.size() > window_size_rpm_) fl_rpm_window_.pop_front();
                if (fr_rpm_window_.size() > window_size_rpm_) fr_rpm_window_.pop_front();
                if (rl_rpm_window_.size() > window_size_rpm_) rl_rpm_window_.pop_front();
                if (rr_rpm_window_.size() > window_size_rpm_) rr_rpm_window_.pop_front();
                // 표준편차 계산
                if (fl_rpm_window_.size() == window_size_rpm_) {
                    auto calc_std_dev = [](const std::deque<float>& data) {
                        float mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
                        float sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
                        return std::sqrt(sq_sum / data.size() - mean * mean);
                    };
                    float fl_std_dev = calc_std_dev(fl_rpm_window_);
                    float fr_std_dev = calc_std_dev(fr_rpm_window_);
                    float rl_std_dev = calc_std_dev(rl_rpm_window_);
                    float rr_std_dev = calc_std_dev(rr_rpm_window_);
                    // 위반 체크
                    if (fl_std_dev > rpm_std_dev_threshold || fr_std_dev > rpm_std_dev_threshold ||
                        rl_std_dev > rpm_std_dev_threshold || rr_std_dev > rpm_std_dev_threshold) {
                        rpm_violation_count_++;
                        NLOG(info) << "(mid)rpm_평균치: "
                        << "FL: " << fl_std_dev << ", FR: " << fr_std_dev 
                        << ", RL: " <<rl_std_dev <<", RR: " << rr_std_dev 
                        << " / Threshold:  " << rpm_std_dev_threshold;
                    }
                    NLOG(info) << "(mid)rpm_평균치: "
                        << "FL: " << fl_std_dev << ", FR: " << fr_std_dev 
                        << ", RL: " <<rl_std_dev <<", RR: " << rr_std_dev 
                        << " / Threshold:  " << rpm_std_dev_threshold;
                }
                
            }
            cnt_++;
            rate.sleep();
        }
    } else {
        ROS_INFO("User did not confirm. Skipping test.");
    }

    // Stop robot
    cmdVelocity(0.0, 0.0);
    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

    // Evaluate results
    if (user_confirmed_ && rpm_violation_count_ <= max_violations) {
        NLOG(info) << "중속 주행 테스트 통과";
        result_data_list_[1] = 1;
        result_error_list_[1] = "OK";
    } else {
        NLOG(info) << "중속 주행 테스트 실패";
        result_data_list_[1] = 2;
        result_error_list_[1] = rpm_violation_count_ > max_violations ? "RPM TUNING ERROR" : "USER CANCELLED";
        vec_failed_scenario_name_.emplace_back("mid_speed_driving");
        NLOG(info) << "--- 모터 떨림이 감지되었습니다. 튜닝이 필요합니다. ---";
    }

    // Reset variables
    cnt_ = 0;
    user_confirmed_ = false;
    saveResult();
    NLOG(info) << "Mid-speed driving test end: " << rpm_violation_count_ << " / " << max_violations;
}

void MotorTuningTester::checkCasterLock(const Json::Value& scenario) {
    ROS_INFO("Checking Caster Lock (0.5m straight, 90 deg rotation, 1m straight)...");
    // 테스트 파라미터 설정 
    // 테스트: 직선주행 -> 90도 회전 -> 직선주행
    float straight_speed = 0.5; // 주행 선속도
    float rotation_speed = 0.349066; // 주행 각속도
    float first_distance = 0.5; // 첫번째 직선 주행 거리
    float second_distance = 0.5; // 두번째 직선 주행 거리
    float target_angle = M_PI / 2; // quad_cmd 기준 90도 회전 
    // 변수 초기화
    double global_start_time = ros::Time::now().toSec();
    double rpm_check_start_time = global_start_time;
    double stabilized_time = 0.0;
    bool rpm_stable = false;
    bool stabilized = false;

    // Load parameters from JSON
    float current_std_dev_threshold = scenario["parameters"].isMember("current_std_dev_threshold") ? 
                                      scenario["parameters"]["current_std_dev_threshold"].asFloat() : 0.5;
    int max_violations = scenario["parameters"]["max_violations"].asInt();
    float stabilization_time_threshold = scenario["parameters"].isMember("stabilization_time_threshold") ? 
                                         scenario["parameters"]["stabilization_time_threshold"].asFloat() : 2.0;
    if (cnt_ == 0) {
        NLOG(info) << "캐스터락 현상으로 인한 과전류 테스트를 시작합니다...";
        NLOG(info) << "/nc_motor_tester/start 토픽을 통해 true 를 날리면 시작하겠습니다...";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed_ = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋
    }

    ros::Rate rate(10); // 10Hz

    double start_x = robot_odom_.pose.pose.position.x;
    double start_y = robot_odom_.pose.pose.position.y;
    double distance_traveled = 0.0;
    double current_angle = 0.0;
    current_violation_count_ = 0;
    enum { STRAIGHT1, ROTATE, STRAIGHT2, DONE } state = STRAIGHT1;

    // 슬라이딩 윈도우 초기화
    fl_current_window_.clear();
    fr_current_window_.clear();
    rl_current_window_.clear();
    rr_current_window_.clear();

    if (user_confirmed_) {
        while (state != DONE && ros::ok()) {
            if (state == STRAIGHT1) {
                // 1. Command velocity
                if (n_kinematics_ == 0) {
                    cmdVelocity(straight_speed, 0.0);
                } else {
                    cmdQuadVelocity(straight_speed, 0.0, straight_speed, 0.0);
                }
                if (!rpm_stable) {
                    rpm_check_start_time = ros::Time::now().toSec();
                    rpm_stable = true;
                }
                // 2. 거리 측정
                double current_x = robot_odom_.pose.pose.position.x;
                double current_y = robot_odom_.pose.pose.position.y;
                distance_traveled = std::hypot(current_x - start_x, current_y - start_y);
                
                // 3. 최초 안정화 체크
                if (!stabilized)
                {
                    bool fl_stable = robot_motor_info_.f_FL_traction_motor_feedback_rpm >= robot_motor_info_.f_FL_traction_motor_target_rpm * 0.98;
                    bool fr_stable = robot_motor_info_.f_FR_traction_motor_feedback_rpm >= robot_motor_info_.f_FR_traction_motor_target_rpm * 0.98;
                    bool rl_stable = robot_motor_info_.f_RL_traction_motor_feedback_rpm >= robot_motor_info_.f_RL_traction_motor_target_rpm * 0.98;
                    bool rr_stable = robot_motor_info_.f_RR_traction_motor_feedback_rpm >= robot_motor_info_.f_RR_traction_motor_target_rpm * 0.98;
                    
                    if (n_kinematics_ == 0 || n_kinematics_ == 1) // DD or QD
                    {
                        if (fl_stable && fr_stable)
                        {
                            stabilized = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - rpm_check_start_time;
                            NLOG(info) << "[CasterRock1] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[CasterRock1] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                    else if (n_kinematics_ == 2) // OD
                    {
                        if (fl_stable)
                        {
                            stabilized = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - rpm_check_start_time;
                            NLOG(info) << "[CasterRoc1k] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[CasterRock1] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                    else if (n_kinematics_ == 3) // OD
                    {
                        if (fl_stable && fr_stable && rl_stable && rr_stable)
                        {
                            stabilized = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - rpm_check_start_time;
                            NLOG(info) << "[CasterRock1] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[CasterRock1] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                }
                // NLOG(info) << "(caster1)이동거리_확인: " << distance_traveled << " / " << first_distance;
                if (distance_traveled >= first_distance) {
                    state = ROTATE;
                    current_angle = 0.0;
                }
            } 
            else if (state == ROTATE) {
                // Rotate 90 degrees
                if (n_kinematics_ == 0) {
                    cmdVelocity(0.0, rotation_speed);
                } else {
                    cmdQuadVelocity(0.0, target_angle, 0.0, target_angle);
                }
                current_angle += rotation_speed * 0.1; // Approx angle based on time
                if (current_angle >= target_angle) {
                    state = STRAIGHT2;
                    start_x = robot_odom_.pose.pose.position.x;
                    start_y = robot_odom_.pose.pose.position.y;
                    
                    distance_traveled = 0.0;
                    stabilized = false;
                    rpm_stable = false;
                }
            } else if (state == STRAIGHT2) {
                // 1. Command velocity
                if (n_kinematics_ == 0) {
                    cmdVelocity(straight_speed, 0.0);
                } else {
                    cmdQuadVelocity(straight_speed, 0.0, straight_speed, 0.0);
                }
                if (!rpm_stable) {
                    rpm_check_start_time = ros::Time::now().toSec();
                    rpm_stable = true;
                }
                // 2. 거리 측정
                double current_x = robot_odom_.pose.pose.position.x;
                double current_y = robot_odom_.pose.pose.position.y;
                distance_traveled = std::hypot(current_x - start_x, current_y - start_y);
                
                // 3. 최초 안정화 체크
                if (!stabilized)
                {
                    bool fl_stable = robot_motor_info_.f_FL_traction_motor_feedback_rpm >= robot_motor_info_.f_FL_traction_motor_target_rpm * 0.98;
                    bool fr_stable = robot_motor_info_.f_FR_traction_motor_feedback_rpm >= robot_motor_info_.f_FR_traction_motor_target_rpm * 0.98;
                    bool rl_stable = robot_motor_info_.f_RL_traction_motor_feedback_rpm >= robot_motor_info_.f_RL_traction_motor_target_rpm * 0.98;
                    bool rr_stable = robot_motor_info_.f_RR_traction_motor_feedback_rpm >= robot_motor_info_.f_RR_traction_motor_target_rpm * 0.98;
                    
                    if (n_kinematics_ == 0 || n_kinematics_ == 1) // DD or QD
                    {
                        if (fl_stable && fr_stable)
                        {
                            stabilized = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - rpm_check_start_time;
                            NLOG(info) << "[CasterRock2] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[CasterRock2] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                    else if (n_kinematics_ == 2) // SD
                    {
                        if (fl_stable)
                        {
                            stabilized = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - rpm_check_start_time;
                            NLOG(info) << "[CasterRock2] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[CasterRock2] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                    else if (n_kinematics_ == 3) // OD
                    {
                        if (fl_stable && fr_stable && rl_stable && rr_stable)
                        {
                            stabilized = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - rpm_check_start_time;
                            NLOG(info) << "[CasterRock2] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[CasterRock2] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                }
                // 전류 데이터 수집
                fl_current_window_.push_back(robot_motor_info_.f_FL_traction_motor_current);
                fr_current_window_.push_back(robot_motor_info_.f_FR_traction_motor_current);
                rl_current_window_.push_back(robot_motor_info_.f_RL_traction_motor_current);
                rr_current_window_.push_back(robot_motor_info_.f_RR_traction_motor_current);

                if (fl_current_window_.size() > window_size_current_) fl_current_window_.pop_front();
                if (fr_current_window_.size() > window_size_current_) fr_current_window_.pop_front();
                if (rl_current_window_.size() > window_size_current_) rl_current_window_.pop_front();
                if (rr_current_window_.size() > window_size_current_) rr_current_window_.pop_front();

                // 표준편차 계산
                if (fl_current_window_.size() == window_size_current_) {
                    auto calc_std_dev = [](const std::deque<float>& data) {
                        float mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
                        float sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
                        return std::sqrt(sq_sum / data.size() - mean * mean);
                    };
                    float fl_std_dev = calc_std_dev(fl_current_window_);
                    float fr_std_dev = calc_std_dev(fr_current_window_);
                    float rl_std_dev = calc_std_dev(rl_current_window_);
                    float rr_std_dev = calc_std_dev(rr_current_window_);

                    if (fl_std_dev > current_std_dev_threshold || fr_std_dev > current_std_dev_threshold ||
                        rl_std_dev > current_std_dev_threshold || rr_std_dev > current_std_dev_threshold) {
                        current_violation_count_++;
                        NLOG(info) << "(caster2) Current_평균치: "
                                   << "FL: " << fl_std_dev << ", FR: " << fr_std_dev
                                   << ", RL: " << rl_std_dev << ", RR: " << rr_std_dev
                                   << " / Threshold: " << current_std_dev_threshold;
                    }
                    NLOG(info) << "(caster2) Current_평균치: "
                                   << "FL: " << fl_std_dev << ", FR: " << fr_std_dev
                                   << ", RL: " << rl_std_dev << ", RR: " << rr_std_dev
                                   << " / Threshold: " << current_std_dev_threshold;
                }
                if (distance_traveled >= second_distance) {
                    // current_time = ros::Time::now().toSec();
                    state = DONE;
                }
            }
            cnt_++;
            rate.sleep();
        }
    } else {
        ROS_INFO("User did not confirm. Skipping test.");
    }

    // Stop robot
    cmdVelocity(0.0, 0.0);
    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

    // Evaluate results
    if (user_confirmed_ && current_violation_count_ <= max_violations) {
        NLOG(info) << "캐스터락 현상으로 인한 과전류 확인 테스트 통과";
        result_data_list_[2] = 1;
        result_error_list_[2] = "OK";
    } else {
        NLOG(info) << "캐스터락 현상으로 인한 과전류 확인 테스트 실패";
        result_data_list_[2] = 2;
        result_error_list_[2] = (current_violation_count_ > max_violations ? "CURRENT OVERLOAD" : "USER CANCELLED");
        vec_failed_scenario_name_.emplace_back("caster_lock");
    }

    // Reset variables
    cnt_ = 0;
    user_confirmed_ = false;
    saveResult();
    NLOG(info) << "Caster lock test end: " << current_violation_count_ << " / " << max_violations;
}

void MotorTuningTester::checkSuddenStartStop(const Json::Value& scenario) {
    ROS_INFO("Checking Sudden Start/Stop (1.8 m/s)...");
    // 테스트 파라미터 설정
    // 테스트: 최고속도 주행 -> cmd_vel(0,0)
    float target_speed = 1.8; // 주행 선속도
    float target_distance = 1.0; // 주행 거리(최고속도까지 오를만한 충분한 거리 필요.)
    // 변수 초기화
    double start_time = 0.0;
    bool start_time_set = false;
    double stabilized_time = 0.0;
    bool stabilized_start = false;
    bool stabilized_stop = false;

    // Load parameters from JSON
    float current_std_dev_threshold = scenario["parameters"].isMember("current_std_dev_threshold") ? 
                                      scenario["parameters"]["current_std_dev_threshold"].asFloat() : 3.0;
    int max_violations = scenario["parameters"]["max_violations"].asInt();
    float stabilization_time_threshold = scenario["parameters"].isMember("stabilization_time_threshold") ? 
                                         scenario["parameters"]["stabilization_time_threshold"].asFloat() : 2.0;    
    if (cnt_ == 0) {
        NLOG(info) << "급정지로 인한 과전류 테스트를 시작합니다...";
        NLOG(info) << "/nc_motor_tester/start 토픽을 통해 true 를 날리면 시작하겠습니다...";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed_ = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋
    }

    ros::Rate rate(10); // 10Hz

    double start_x = robot_odom_.pose.pose.position.x;
    double start_y = robot_odom_.pose.pose.position.y;
    double distance_traveled = 0.0;
    current_violation_count_ = 0;
    enum { START, STOP, DONE } state = START;

    // 슬라이딩 윈도우 초기화
    fl_current_window_.clear();
    fr_current_window_.clear();
    rl_current_window_.clear();
    rr_current_window_.clear();

    if (user_confirmed_) {
        while (state != DONE && ros::ok()) 
        {
            if (state == START) 
            {
                if (n_kinematics_ == 0) {
                    cmdVelocity(target_speed, 0.0);
                } else {
                    cmdQuadVelocity(target_speed, 0.0, target_speed, 0.0);
                }
                if (!start_time_set) {
                    start_time = ros::Time::now().toSec();
                    start_time_set = true;
                }
                // 2. 거리 측정
                double current_x = robot_odom_.pose.pose.position.x;
                double current_y = robot_odom_.pose.pose.position.y;
                distance_traveled = std::hypot(current_x - start_x, current_y - start_y);
                
                // 3. 최초 안정화 체크
                if (!stabilized_start) {
                    bool fl_stable = robot_motor_info_.f_FL_traction_motor_feedback_rpm >= robot_motor_info_.f_FL_traction_motor_target_rpm * 0.98;
                    bool fr_stable = robot_motor_info_.f_FR_traction_motor_feedback_rpm >= robot_motor_info_.f_FR_traction_motor_target_rpm * 0.98;
                    bool rl_stable = robot_motor_info_.f_RL_traction_motor_feedback_rpm >= robot_motor_info_.f_RL_traction_motor_target_rpm * 0.98;
                    bool rr_stable = robot_motor_info_.f_RR_traction_motor_feedback_rpm >= robot_motor_info_.f_RR_traction_motor_target_rpm * 0.98;

                    if (n_kinematics_ == 0 || n_kinematics_ == 1) // DD or QD
                    {
                        if (fl_stable && fr_stable)
                        {
                            stabilized_start = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - start_time;
                            NLOG(info) << "[SuddenStop] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[SuddenStop] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                    else if (n_kinematics_ == 2) // SD
                    {
                        if (fl_stable)
                        {
                            stabilized_start = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - start_time;
                            NLOG(info) << "[SuddenStop] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[SuddenStop] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                    else if (n_kinematics_ == 3) // OD
                    {
                        if (fl_stable && fr_stable && rl_stable && rr_stable)
                        {
                            stabilized_start = true;
                            stabilized_time = ros::Time::now().toSec();
                            double response_time = stabilized_time - start_time;
                            NLOG(info) << "[SuddenStop] RPM Stabilized after: " << response_time << " sec";
                            // rpm 반응성 판단
                            if (response_time > stabilization_time_threshold) {
                                rpm_violation_count_ = max_violations + 1; // 강제 실패 처리
                                NLOG(info) << "[SuddenStop] 반응성이 기준치보다 미달입니다. 튜닝이 필요합니다.";
                            }
                        }
                    }
                    
                } 
                // NLOG(info) << "이동거리_확인: " << distance_traveled << " / " << target_distance;
                if (distance_traveled >= target_distance) {
                    // current_time = ros::Time::now().toSec();
                    state = STOP;
                    stabilized_stop = false;
                    start_time_set = false;
                    // STOP 상태로 전환 시 윈도우 초기화
                    fl_current_window_.clear();
                    fr_current_window_.clear();
                    rl_current_window_.clear();
                    rr_current_window_.clear();
                }
        }
        else if (state == STOP) {
                    // Sudden stop
                    // 1. Command velocity
                    cmdVelocity(0.0, 0.0);
                    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

                    if (!start_time_set) {
                    start_time = ros::Time::now().toSec();
                    start_time_set = true;
                }

                // 전류 데이터 수집
                fl_current_window_.push_back(robot_motor_info_.f_FL_traction_motor_current);
                fr_current_window_.push_back(robot_motor_info_.f_FR_traction_motor_current);
                rl_current_window_.push_back(robot_motor_info_.f_RL_traction_motor_current);
                rr_current_window_.push_back(robot_motor_info_.f_RR_traction_motor_current);

                if (fl_current_window_.size() > window_size_current_ - 20) fl_current_window_.pop_front();
                if (fr_current_window_.size() > window_size_current_ - 20) fr_current_window_.pop_front();
                if (rl_current_window_.size() > window_size_current_ - 20) rl_current_window_.pop_front();
                if (rr_current_window_.size() > window_size_current_ - 20) rr_current_window_.pop_front();

                // 표준편차 계산
                if (fl_current_window_.size() == window_size_current_ - 20) {
                    auto calc_std_dev = [](const std::deque<float>& data) {
                        float mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
                        float sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
                        return std::sqrt(sq_sum / data.size() - mean * mean);
                    };
                    float fl_std_dev = calc_std_dev(fl_current_window_);
                    float fr_std_dev = calc_std_dev(fr_current_window_);
                    float rl_std_dev = calc_std_dev(rl_current_window_);
                    float rr_std_dev = calc_std_dev(rr_current_window_);

                    if (fl_std_dev > current_std_dev_threshold || fr_std_dev > current_std_dev_threshold ||
                        rl_std_dev > current_std_dev_threshold || rr_std_dev > current_std_dev_threshold) {
                        current_violation_count_++;
                        NLOG(info) << "(suddenstop) Current std dev violation: "
                                   << "FL: " << fl_std_dev << ", FR: " << fr_std_dev
                                   << ", RL: " << rl_std_dev << ", RR: " << rr_std_dev
                                   << " / Threshold: " << current_std_dev_threshold;
                    }
                    NLOG(info) << "(suddenstop) Current std dev violation: "
                                   << "FL: " << fl_std_dev << ", FR: " << fr_std_dev
                                   << ", RL: " << rl_std_dev << ", RR: " << rr_std_dev
                                   << " / Threshold: " << current_std_dev_threshold;
                }
                if (ros::Time::now().toSec() - start_time > 2.0) {
                        state = DONE;
                    }
                }
            cnt_++;
            rate.sleep();
        }   
    }
    else {
        ROS_INFO("User did not confirm. Skipping test.");
    }

    // Ensure robot is stopped
    cmdVelocity(0.0, 0.0);
    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

    // Evaluate results
    if (user_confirmed_ && current_violation_count_ <= max_violations) {
        NLOG(info) << "급정지로 인한 과전류 확인 테스트 통과";
        result_data_list_[3] = 1;
        result_error_list_[3] = "OK";
    } else {
        NLOG(info) << "급정지로 인한 과전류 확인 테스트 실패";
        result_data_list_[3] = 2;
        result_error_list_[3] = current_violation_count_ > max_violations ? "CURRENT OVERLOAD" : "USER CANCELLED";
        vec_failed_scenario_name_.emplace_back("sudden_start_stop");
    }

    // Reset variables
    cnt_ = 0;
    user_confirmed_ = false;
    saveResult();
    NLOG(info) << "Sudden start/stop test end: " << current_violation_count_ << " / " << max_violations;
}

void MotorTuningTester::checkRightRotation(const Json::Value& scenario) {
    ROS_INFO("Checking Right 90-Degree Rotation...");
    // 테스트 파라미터 설정
    float rotation_speed = -0.349066; // cmd_vel 토픽을 위한 각속도
    float target_angle = -M_PI / 2; // quad_cmd 기준 90도 회전
    // 변수 초기화
    double start_time = 0.0;
    bool start_time_set = false;
    double stabilized_time = 0.0;
    bool time_exceeded = false;
    rpm_violation_count_ = 0;

    // Load parameters from JSON
    float time_threshold = scenario["parameters"]["time_threshold"].asFloat();
    int max_violations = scenario["parameters"]["max_violations"].asInt();
    float stabilization_time_threshold = scenario["parameters"].isMember("stabilization_time_threshold") ? 
                                         scenario["parameters"]["stabilization_time_threshold"].asFloat() : 2.0;    
    if (cnt_ == 0) {
        NLOG(info) << "90도 우회전을 통해 목표각도까지 도달시간 테스트 시작합니다...";
        NLOG(info) << "/nc_motor_tester/start 토픽을 통해 true 를 날리면 시작하겠습니다...";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed_ = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋
    }

    ros::Rate rate(10); // 10Hz
    double current_angle = 0.0;
    double current_time;

    if (user_confirmed_) {
        while (current_angle > target_angle && ros::ok()) {
            current_time = ros::Time::now().toSec();
            // NLOG(info) << "(right-spin)이동각도_확인: " << current_angle << " / " << target_angle;
            // Command rotation (positive angular velocity for right rotation)
            if (n_kinematics_ == 0) {
                cmdVelocity(0.0, rotation_speed);
            } else {
                cmdQuadVelocity(0.0, target_angle, 0.0, target_angle);
            }
            if (!start_time_set) {
                start_time = ros::Time::now().toSec();
                start_time_set = true;
            }
            // Update angle (approximation based on time and speed)
            current_angle += rotation_speed * 0.1;

            if (current_angle >= target_angle) {
                 // Ensure we don't overshoot
                current_time = ros::Time::now().toSec();
            }
            
            if (current_time - start_time > time_threshold) {
                rpm_violation_count_ = max_violations + 1; // Force failure
                time_exceeded = true;
                NLOG(info) << "(right)eddie_time_check: " << current_time - start_time ;
                break;
            }
            cnt_++;
            rate.sleep();
        }
    } else {
        ROS_INFO("User did not confirm. Skipping test.");
    }

    // Stop robot
    cmdVelocity(0.0, 0.0);
    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

    // Evaluate results
    if (user_confirmed_ && !time_exceeded && rpm_violation_count_ <= max_violations) {
        NLOG(info) << "90도 우회전 목표각도 도달 테스트 통과";
        result_data_list_[4] = 1;
        result_error_list_[4] = "OK";
    } else {
        NLOG(info) << "90도 우회전 목표각도 도달 테스트 실패";
        result_data_list_[4] = 2;
        result_error_list_[4] = time_exceeded ? "TIME EXCEEDED" : "USER CANCELLED";
        vec_failed_scenario_name_.emplace_back("right_rotation");
        NLOG(info) << "--- 회전 시간이 초과되었습니다. 모터 튜닝이 필요합니다. ---";
    }
    NLOG(info) << "(right)eddie_time_check: " << current_time - start_time ;
    // Reset variables
    cnt_ = 0;
    user_confirmed_ = false;
    saveResult();
    NLOG(info) << "Right rotation test end: " << current_time - start_time << " / " << time_threshold;
}

void MotorTuningTester::checkLeftRotation(const Json::Value& scenario) {
    ROS_INFO("Checking Left 90-Degree Rotation...");
    // 테스트 파라미터 설정
    float rotation_speed = 0.349066; // Approx 20 deg/s for DD (negative for left rotation)
    float target_angle = M_PI / 2; // 90 degrees
    // 변수 초기화
    double start_time = 0.0;
    bool start_time_set = false;
    double stabilized_time = 0.0;
    bool time_exceeded = false;
    rpm_violation_count_ = 0;

    // Load parameters from JSON
    float time_threshold = scenario["parameters"]["time_threshold"].asFloat();
    int max_violations = scenario["parameters"]["max_violations"].asInt();
    float stabilization_time_threshold = scenario["parameters"].isMember("stabilization_time_threshold") ? 
                                         scenario["parameters"]["stabilization_time_threshold"].asFloat() : 2.0;
    if (cnt_ == 0) {
        NLOG(info) << "90도 좌회전을 통해 목표각도까지 도달시간 테스트 시작합니다...";
        NLOG(info) << "/nc_motor_tester/start 토픽을 통해 true 를 날리면 시작하겠습니다...";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed_ = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋
    }

    ros::Rate rate(10); // 10Hz
    double current_angle = 0.0;
    double current_time;

    if (user_confirmed_) {
        while (current_angle < target_angle && ros::ok()) {
            current_time = ros::Time::now().toSec();
            // NLOG(info) << "(left-spin)이동각도_확인: " << current_angle << " / " << target_angle;
            // Command rotation (negative angular velocity for left rotation)
            if (n_kinematics_ == 0) {
                cmdVelocity(0.0, rotation_speed);
            } else {
                cmdQuadVelocity(0.0, target_angle, 0.0, target_angle);
            }
            if (!start_time_set) {
                start_time = ros::Time::now().toSec();
                start_time_set = true;
            }

            // Update angle (approximation based on time and speed)
            current_angle += rotation_speed * 0.1;
            if (current_angle >= target_angle) {
                 // Ensure we don't overshoot
                current_time = ros::Time::now().toSec();
            }
            if (current_time - start_time > time_threshold) {
                rpm_violation_count_ = max_violations + 1; // Force failure
                time_exceeded = true;
                NLOG(info) << "(left)eddie_time_check: " << current_time - start_time ;
                break;
            }
            cnt_++;
            rate.sleep();
        }
    } else {
        ROS_INFO("User did not confirm. Skipping test.");
    }

    // Stop robot
    cmdVelocity(0.0, 0.0);
    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

    // Evaluate results
    if (user_confirmed_ && !time_exceeded && rpm_violation_count_ <= max_violations) {
        NLOG(info) << "90도 좌회전 목표각도 도달 테스트 통과";
        result_data_list_[5] = 1;
        result_error_list_[5] = "OK";
    } else {
        NLOG(info) << "90도 좌회전 목표각도 도달 테스트 실패";
        result_data_list_[5] = 2;
        result_error_list_[5] = time_exceeded ? "TIME EXCEEDED" : "USER CANCELLED";
        vec_failed_scenario_name_.emplace_back("left_rotation");
        NLOG(info) << "--- 회전 시간이 초과되었습니다. 모터 튜닝이 필요합니다. ---";
    }
    NLOG(info) << "(left)eddie_time_check: " << current_time - start_time ;
    // Reset variables
    cnt_ = 0;
    user_confirmed_ = false;
    saveResult();
    NLOG(info) << "Left rotation test end: " << current_time - start_time << " / " << time_threshold;
}

void MotorTuningTester::StartTest() {
    Json::Value scenarios;
    if (!loadScenarios(package_path_ + "/motor_tuning_scenarios.json", scenarios)) {
        ROS_ERROR("Failed to load scenarios!");
        return;
    }

    int n_scenario_start = 1;
    ros::param::param<int>("/nc_motor_tuning/scenario", n_scenario_start, 1);

    n_scenario_start--;
    for (int i = n_scenario_start; i < scenarios.size(); i++) {
        executeScenario(scenarios[i]);
    }

    ROS_INFO("All scenarios executed");
    ROS_INFO("Motor scenario count: %d", n_motor_scenario_count_);
    ROS_INFO("Failed scenario count: %lu", vec_failed_scenario_name_.size());
    for (const auto& name : vec_failed_scenario_name_) {
        ROS_INFO("Failed: %s", name.c_str());
    }
}

void MotorTuningTester::MotorCallback(const core_msgs::MotorInfo::ConstPtr& msg) {
    robot_motor_info_ = *msg;
}

void MotorTuningTester::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_time_ = ros::Time::now().toSec();
    robot_odom_ = *msg;
}

void MotorTuningTester::AnswerCallback(const std_msgs::String::ConstPtr& msg) {
    scenario_start_ = msg->data;
}

void MotorTuningTester::cmdVelocity(float linear_x, float angular_z) {
    geometry_msgs::Twist vel;
    vel.linear.x = linear_x;
    vel.angular.z = angular_z;
    cmd_vel_pub_.publish(vel);
}

void MotorTuningTester::cmdQuadVelocity(float linearVel1, float steerAngle1, float linearVel2, float steerAngle2) {
    std_msgs::String vel;
    vel.data = std::to_string(linearVel1) + "/" + std::to_string(steerAngle1) + "/" +
               std::to_string(linearVel2) + "/" + std::to_string(steerAngle2);
    quad_cmd_pub_.publish(vel);
}

bool MotorTuningTester::loadScenarios(const std::string& filename, Json::Value& scenarios) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open scenario file: %s", filename.c_str());
        return false;
    }
    Json::Reader reader;
    bool success = reader.parse(file, scenarios);
    file.close();
    return success;
}

void MotorTuningTester::executeScenario(const Json::Value& scenario) {
    ROS_INFO("Executing scenario: %s/%s", scenario["name"].asCString(), s_kinematics_type_.c_str());
    n_motor_scenario_count_++;

    // if (ros::Time::now().toSec() - odom_time_ > 2) {
    //     ROS_ERROR("Odometry timeout! Check connection.");
    //     return;
    // }

    if (scenario["low_speed_driving"].asBool()) {
        checkLowSpeedDriving(scenario);
    } else if (scenario["mid_speed_driving"].asBool()) {
        checkMidSpeedDriving(scenario);
    } else if (scenario["caster_lock"].asBool()) {
        checkCasterLock(scenario);
    } else if (scenario["sudden_start_stop"].asBool()) {
        checkSuddenStartStop(scenario);
    } else if (scenario["right_rotation"].asBool()) {
        checkRightRotation(scenario);
    } else if (scenario["left_rotation"].asBool()) {
        checkLeftRotation(scenario);
    }

    ROS_INFO("===============================================");
}

void MotorTuningTester::saveResult() {
    std::ofstream file(package_path_ + "/motor_tuning_result.txt");
    if (file.is_open()) {
        for (int i = 0; i < 6; i++) { // Updated to 6 tests
            file << result_data_list_[i] << (i < 5 ? "/" : "\n");
        }
        for (int i = 0; i < 6; i++) {
            file << result_error_list_[i] << (i < 5 ? "/" : "/");
        }
        file.close();
        ROS_INFO("Results saved to motor_tuning_result.txt");
    } else {
        ROS_ERROR("Failed to save results!");
    }
}

std::string MotorTuningTester::getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_tuning_tester_node");
    MotorTuningTester tester;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}