#include "nc_motor_semical_tester.hpp"

#include "core/util/logger.hpp"

MotorTester::MotorTester()
    : nh("~")
{
    NLOG(info) << "=== Program Started at ";

    package_path_ = ros::package::getPath("nc_motor_tester") + "/launch";
    // param_pub_ = nh_.advertise<std_msgs::String>("navifra/param_update", 1, true);
    nav_cmd_pub_ = nh_.advertise<std_msgs::String>("/navifra/cmd", 1, true);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    quad_cmd_pub_ = nh_.advertise<std_msgs::String>("/quad_cmd", 1, true);

    motor_info_sub_ = nh_.subscribe("/motor_info", 10, &MotorTester::MotorCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &MotorTester::OdomCallback, this);
    front_scan_sub_ = nh_.subscribe("/scan_front", 10, &MotorTester::FrontLidarCallback, this);
    rear_scan_sub_ = nh_.subscribe("/scan_rear", 10, &MotorTester::RearLidarCallback, this);
    left_scan_sub_ = nh_.subscribe("/scan_left", 10, &MotorTester::LeftLidarCallback, this);    
    right_scan_sub_ = nh_.subscribe("/scan_right", 10, &MotorTester::RightLidarCallback, this);
    start_sub_ = nh_.subscribe("/nc_motor_tester/start", 10, &MotorTester::AnswerCallback, this);
    
    // param_pub_.publish(msg);
    
    // Initialize parameters from ROS params
    ros::param::param<int>("/motion_base/n_kinematics_type", n_kinematics_, 0);
    // Set kinematics type
    if (n_kinematics_ == 0) s_kinematics_type_ = "DD";
    else if (n_kinematics_ == 1) s_kinematics_type_ = "QD";
    else if (n_kinematics_ == 2) s_kinematics_type_ = "SD";
    else if (n_kinematics_ == 3) s_kinematics_type_ = "OD";

    std::this_thread::sleep_for(std::chrono::seconds(2));
    // Constructor: 시나리오 실행
    boost::thread th1 = boost::thread(boost::bind(&MotorTester::StartTest, this));
}

MotorTester::~MotorTester()
{
}

void MotorTester::checkFLTractionMotor() {
    static int step = 0;
    static bool step_completed = false;
    ros::Rate rate(10); // 10Hz
    ros::Rate wait_rate(10);

    while (!step_completed && ros::ok()) {
        switch (step) {
            case 0:
                NLOG(info) << "--- FL 주행모터 테스트 시작 ---";
                NLOG(info) << "FL 주행모터가 실제로 움직이는지 확인해주세요.";
                NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
                // start 토픽 받기 전까지 대기
                while (ros::ok() && scenario_start_ != "true") {
                    ros::spinOnce();
                    wait_rate.sleep();
                }
                user_confirmed = (scenario_start_ == "true");
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋

                if (user_confirmed) {
                    for (int i = 0; i < 40 && ros::ok(); ++i) {
                        if (n_kinematics_ == 0) {
                            cmdVelocity(0.2, -0.349066);
                        } else {
                            // FL, FR, RL, RR 순서
                            cmdQuadVelocity(0.2, 0.0, 0.0, 0.0);
                        }

                        if (fl_encoder_past_ != 0) {
                            fl_encoder_ += robot_motor_info_.f_FL_traction_motor_encoder - fl_encoder_past_;
                        }
                        fl_encoder_past_ = robot_motor_info_.f_FL_traction_motor_encoder;

                        rate.sleep();
                    }
                    cmdVelocity(0, 0);
                    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);
            
                    user_confirmed = false;
                    NLOG(info) << "FL 주행모터가 전진 방향으로 움직였나요? (y/n)";
                    NLOG(info) << "움직였다면 /nc_motor_tester/start 토픽으로 y 를 입력해주세요. ---";

                    while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }

                    if (scenario_start_ == "y" || scenario_start_ == "Y") {
                        user_confirmed = true;
                    }  
                    else {
                        user_confirmed = false;
                    }

                    if (user_confirmed) {
                        NLOG(info) << "FL 주행모터 테스트 1단계 통과했습니다.(실제 방향성 정상)";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                        result_data_list_[0] = 1;
                        result_error_list_[0] = "OK";
                    } else {
                        result_data_list_[0] = 2;
                        result_error_list_[0] = (fl_encoder_ <= 0) ? "FL 엔코더 파라미터 이상" : "FL 방향성 이상";
                        vec_failed_scenario_name_.emplace_back("fl_traction_motor_check");

                        b_fl_wheel_real_dir_switch = !b_fl_wheel_real_dir_switch;
                        ros::param::set("/driver_traction/b_FL_traction_target_rpmd", b_fl_wheel_real_dir_switch);
                        NLOG(info) << "FL 주행모터 방향 전환";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                    }
                } else {
                    NLOG(info) << "테스트 취소됨.";
                    step_completed = true;
                }
                step ++;
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                user_confirmed = false;
                break;

            case 1: 
                NLOG(info) << "나비프라 UI 상에서 로봇이 실제로 움직인 방향과 일치하나요?";
                NLOG(info) << "일치한다면 /nc_motor_tester/start 토픽으로 y를 입력해주세요.";
                NLOG(info) << "FL 주행모터를 다시 구동시켜봐야 된다면 /nc_motor_tester/start 토픽으로 move를 입력해주세요.";
                
                if (scenario_start_ == "move") {
                    for (int i = 0; i < 40 && ros::ok(); ++i) {
                        if (n_kinematics_ == 0) {
                            cmdVelocity(0.2, -0.349066);
                        } else {
                            // FL, FR, RL, RR 순서
                            cmdQuadVelocity(0.2, 0.0, 0.0, 0.0);
                        }

                        if (fl_encoder_past_ != 0) {
                            fl_encoder_ += robot_motor_info_.f_FL_traction_motor_encoder - fl_encoder_past_;
                        }
                        fl_encoder_past_ = robot_motor_info_.f_FL_traction_motor_encoder;

                        rate.sleep();
                    }
                    cmdVelocity(0, 0);
                    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);
                    scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                    user_confirmed = false;
                    continue;
                }

                while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }
                if (scenario_start_ == "y" || scenario_start_ == "Y") {
                    user_confirmed = true;
                } 
                else {
                    user_confirmed = false;
                }

                if (user_confirmed) {
                        NLOG(info) << "FL 주행모터 테스트 2단계 통과했습니다.(ui 방향성 정상)";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                        result_data_list_[0] = 1;
                        result_error_list_[0] = "OK";
                } else {
                    result_data_list_[0] = 2;
                    result_error_list_[0] = "FL 피드백 방향성 이상";
                    vec_failed_scenario_name_.emplace_back("fl_traction_motor_check");

                    b_fl_wheel_ui_dir_switch = !b_fl_wheel_ui_dir_switch;
                    ros::param::set("/driver_traction/b_FL_traction_feedback_rpmd", b_fl_wheel_ui_dir_switch);
                    NLOG(info) << "FL 주행모터 UI 방향 전환";
                    NLOG(info) << "다음 단계로 넘어갑니다.";
                }
                step ++;
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                user_confirmed = false;
                break;

            case 2:
                NLOG(info) << "나비프라 UI에서 선속도가 0.2로 잘 나오나요?";
                NLOG(info) << "일치한다면 /nc_motor_tester/start 토픽으로 y를 입력해주세요.";
                NLOG(info) << "FL 주행모터를 다시 구동시켜봐야 된다면 /nc_motor_tester/start 토픽으로 move를 입력해주세요.";
                
                if (scenario_start_ == "move") {
                    for (int i = 0; i < 40 && ros::ok(); ++i) {
                        if (n_kinematics_ == 0) {
                            cmdVelocity(0.2, -0.349066);
                        } else {
                            // FL, FR, RL, RR 순서
                            cmdQuadVelocity(0.2, 0.0, 0.0, 0.0);
                        }

                        if (fl_encoder_past_ != 0) {
                            fl_encoder_ += robot_motor_info_.f_FL_traction_motor_encoder - fl_encoder_past_;
                        }
                        fl_encoder_past_ = robot_motor_info_.f_FL_traction_motor_encoder;

                        rate.sleep();
                    }
                    cmdVelocity(0, 0);
                    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);
                    scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                    user_confirmed = false;
                    continue;
                }

                while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }
                if (scenario_start_ == "y" || scenario_start_ == "Y") {
                    user_confirmed = true;
                } 
                else {
                    user_confirmed = false;
                }

                if (user_confirmed) {
                        NLOG(info) << "FL 주행모터 테스트 완료했습니다.(속도 출력 정상)";
                        result_data_list_[0] = 1;
                        result_error_list_[0] = "OK";
                } else {
                    result_data_list_[0] = 2;
                    result_error_list_[0] = "FL 엔코더 파라미터 이상";
                    vec_failed_scenario_name_.emplace_back("fl_traction_motor_check");

                    b_fl_wheel_enc_dir_switch = !b_fl_wheel_enc_dir_switch;
                    ros::param::set("/driver_traction/b_FL_traction_encoderd", b_fl_wheel_enc_dir_switch);
                    NLOG(info) << "FL 주행모터 엔코더 파라미터 전환";
                    NLOG(info) << "다음 단계로 넘어갑니다.";
                }
                step ++;
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                user_confirmed = false;
                step_completed = true;
                break;
        }
    }

    if (system("rosparam dump ~/navifra_solution/navicore/configs/param.yaml") != 0) {
        NLOG(error) << "Failed to dump parameters to param.yaml";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 변수 초기화
    cnt_ = 0;
    fl_encoder_ = 0;
    user_confirmed = false;
    step = 0;
    step_completed = false;

    saveResult();
    NLOG(info) << "--- FL 주행모터 테스트 완료 ---";
}

void MotorTester::checkRRTractionMotor() {
    static int step = 0;
    static bool step_completed = false;
    ros::Rate rate(10); // 10Hz
    ros::Rate wait_rate(10);


    while (!step_completed && ros::ok()) {
        switch (step) {
            case 0:
                NLOG(info) << "--- RR 주행모터 테스트 시작 ---";
                NLOG(info) << "RR 주행모터가 실제로 움직이는지 확인해주세요.";
                NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
                // start 토픽 받기 전까지 대기
                while (ros::ok() && scenario_start_ != "true") {
                    ros::spinOnce();
                    wait_rate.sleep();
                }
                user_confirmed = (scenario_start_ == "true");
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋

                if (user_confirmed) {
                    for (int i = 0; i < 40 && ros::ok(); ++i) {
                        if (n_kinematics_ == 0) {
                            cmdVelocity(0.2, -0.349066);
                        } else {
                            // FL, FR, RL, RR 순서
                            cmdQuadVelocity(0, 0, 0.2, 0.0);
                        }

                        if (rr_encoder_past_ != 0) {
                            rr_encoder_ += robot_motor_info_.f_RR_traction_motor_encoder - rr_encoder_past_;
                        }
                        rr_encoder_past_ = robot_motor_info_.f_RR_traction_motor_encoder;

                        rate.sleep();
                    }
                    cmdVelocity(0, 0);
                    cmdQuadVelocity(0, 0, 0, 0);
            
                    user_confirmed = false;
                    NLOG(info) << "RR 주행모터가 전진 방향으로 움직였나요? (y/n)";
                    NLOG(info) << "움직였다면 /nc_motor_tester/start 토픽으로 y 를 입력해주세요. ---";

                    while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }

                    if (scenario_start_ == "y" || scenario_start_ == "Y") {
                        user_confirmed = true;
                    }  
                    else {
                        user_confirmed = false;
                    }

                    if (user_confirmed) {
                        NLOG(info) << "RR 주행모터 테스트 1단계 통과했습니다.(실제 방향성 정상)";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                        result_data_list_[0] = 1;
                        result_error_list_[0] = "OK";
                    } else {
                        result_data_list_[0] = 2;
                        result_error_list_[0] = (rr_encoder_ <= 0) ? "RR 엔코더 파라미터 이상" : "RR 방향성 이상";
                        vec_failed_scenario_name_.emplace_back("rr_traction_motor_check");

                        b_rr_wheel_real_dir_switch = !b_rr_wheel_real_dir_switch;
                        ros::param::set("/driver_traction/b_RR_traction_target_rpmd", b_rr_wheel_real_dir_switch);
                        NLOG(info) << "RR 주행모터 방향 전환";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                    }
                } else {
                    NLOG(info) << "테스트 취소됨.";
                    step_completed = true;
                }
                step ++;
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                user_confirmed = false;
                break;

            case 1: 
                NLOG(info) << "나비프라 UI 상에서 로봇이 실제로 움직인 방향과 일치하나요?";
                NLOG(info) << "일치한다면 /nc_motor_tester/start 토픽으로 y를 입력해주세요.";
                NLOG(info) << "RR 주행모터를 다시 구동시켜봐야 된다면 /nc_motor_tester/start 토픽으로 move를 입력해주세요.";
                
                if (scenario_start_ == "move") {
                    for (int i = 0; i < 40 && ros::ok(); ++i) {
                        if (n_kinematics_ == 0) {
                            cmdVelocity(0.2, -0.349066);
                        } else {
                            // FL, FR, RL, RR 순서
                            cmdQuadVelocity(0, 0, 0.2, 0.0);
                        }

                        if (rr_encoder_past_ != 0) {
                            rr_encoder_ += robot_motor_info_.f_RR_traction_motor_encoder - rr_encoder_past_;
                        }
                        rr_encoder_past_ = robot_motor_info_.f_RR_traction_motor_encoder;

                        rate.sleep();
                    }
                    cmdVelocity(0, 0);
                    cmdQuadVelocity(0, 0, 0, 0);
                    scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                    user_confirmed = false;
                    continue;
                }

                while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }
                if (scenario_start_ == "y" || scenario_start_ == "Y") {
                    user_confirmed = true;
                } 
                else {
                    user_confirmed = false;
                }

                if (user_confirmed) {
                        NLOG(info) << "RR 주행모터 테스트 2단계 통과했습니다.(실제 방향성 정상)";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                        result_data_list_[0] = 1;
                        result_error_list_[0] = "OK";
                } else {
                    result_data_list_[0] = 2;
                    result_error_list_[0] = "RR 피드백 방향성 이상";
                    vec_failed_scenario_name_.emplace_back("rr_traction_motor_check");

                    b_rr_wheel_ui_dir_switch = !b_rr_wheel_ui_dir_switch;
                    ros::param::set("/driver_traction/b_RR_traction_feedback_rpmd", b_rr_wheel_ui_dir_switch);
                    NLOG(info) << "RR 주행모터 UI 방향 전환";
                    NLOG(info) << "다음 단계로 넘어갑니다.";
                }
                step ++;
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                user_confirmed = false;
                break;

            case 2:
                NLOG(info) << "나비프라 UI에서 선속도가 0.2로 잘 나오나요?";
                NLOG(info) << "일치한다면 /nc_motor_tester/start 토픽으로 y를 입력해주세요.";
                NLOG(info) << "RR 주행모터를 다시 구동시켜봐야 된다면 /nc_motor_tester/start 토픽으로 move를 입력해주세요.";
                
                if (scenario_start_ == "move") {
                    for (int i = 0; i < 40 && ros::ok(); ++i) {
                        if (n_kinematics_ == 0) {
                            cmdVelocity(0.2, -0.349066);
                        } else {
                            // FL, FR, RL, RR 순서
                            cmdQuadVelocity(0, 0, 0.2, 0);
                        }

                        if (rr_encoder_past_ != 0) {
                            rr_encoder_ += robot_motor_info_.f_RR_traction_motor_encoder - rr_encoder_past_;
                        }
                        rr_encoder_past_ = robot_motor_info_.f_RR_traction_motor_encoder;

                        rate.sleep();
                    }
                    cmdVelocity(0, 0);
                    cmdQuadVelocity(0, 0, 0, 0);
                    scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                    user_confirmed = false;
                    continue;
                }

                while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }
                if (scenario_start_ == "y" || scenario_start_ == "Y") {
                    user_confirmed = true;
                } 
                else {
                    user_confirmed = false;
                }

                if (user_confirmed) {
                        NLOG(info) << "RR 주행모터 테스트 완료했습니다.(속도 출력 정상)";
                        result_data_list_[0] = 1;
                        result_error_list_[0] = "OK";
                } else {
                    result_data_list_[0] = 2;
                    result_error_list_[0] = "RR 엔코더 파라미터 이상";
                    vec_failed_scenario_name_.emplace_back("rr_traction_motor_check");

                    b_rr_wheel_enc_dir_switch = !b_rr_wheel_enc_dir_switch;
                    ros::param::set("/driver_traction/b_RR_traction_encoderd", b_rr_wheel_enc_dir_switch);
                    NLOG(info) << "RR 주행모터 엔코더 파라미터 전환";
                    NLOG(info) << "다음 단계로 넘어갑니다.";
                }
                step ++;
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                user_confirmed = false;
                step_completed = true;
                break;
        }
    }

    if (system("rosparam dump ~/navifra_solution/navicore/configs/param.yaml") != 0) {
        NLOG(error) << "Failed to dump parameters to param.yaml";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 변수 초기화
    cnt_ = 0;
    rr_encoder_ = 0;
    user_confirmed = false;
    step = 0;
    step_completed = false;

    saveResult();
    NLOG(info) << "--- RR 주행모터 테스트 완료 ---";
}

void MotorTester::checkFLSteeringMotor() {
    static int step = 0;
    static bool step_completed = false;
    ros::Rate rate(10); // 10Hz
    ros::Rate wait_rate(10);

    while (!step_completed && ros::ok()) {
        switch (step) 
        {
            case 0:
                NLOG(info) << "--- FL 조향모터 테스트를 시작하겠습니다. ---";
                NLOG(info) << "--- FL 조향모터를 90도 회전하겠습니다. 실제 움직임을 확인해주세요.  ---";
                NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
                // start 토픽 받기 전까지 대기
                while (ros::ok() && scenario_start_ != "true") {
                    ros::spinOnce();
                    wait_rate.sleep();
                }
                user_confirmed = (scenario_start_ == "true");
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
        
                if (user_confirmed)
                {
                    // fl_angle_past_는 0도에서 -90도가 될 때까지 조향 명령 수행.
                    while (fl_angle_past_ > -90 && ros::ok()) {
                        fl_angle_past_ -= 5;
                        cmdQuadVelocity(0.0, fl_angle_past_, 0.0, 0.0);
                        cnt_++;
                        rate.sleep();
                    }
                    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

                    user_confirmed = false;
                    float feedback_deg = robot_motor_info_.f_FL_steer_motor_feedback_deg;
                    NLOG(info) << "--- FL 조향모터가 움직였습니다. 바퀴가 기대하신 방향으로 움직였나요? ---";
                    NLOG(info) << "--- 움직였다면 /nc_motor_tester/start 토픽으로 y 를 입력해주세요. ---";
                    
                    while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }
                    if (scenario_start_ == "y" || scenario_start_ == "Y") {
                        user_confirmed = true;
                    }  
                    else {
                        user_confirmed = false;
                    }
                
                    if (user_confirmed && feedback_deg > -95 && feedback_deg < -85 ) {
                        NLOG(info) <<"--- FL 조향모터 테스트 1단계 통과했습니다.(실제 방향성 정상) ---";
                        result_data_list_[2] = 1;
                        result_error_list_[2] = "OK";
                    } else {
                        NLOG(info) <<"--- FL 조향모터 테스트 1단계 실패했습니다.(실제 방향성 비정상) ---";
                        result_data_list_[2] = 2;
                        result_error_list_[2] = "Direction ERROR";
                        vec_failed_scenario_name_.emplace_back("fl_steering_motor_check");
                        NLOG(info) << "--- FL 조향모터 방향성 파라미터를 바꾸겠습니다. ---";
                    
                        b_fl_steer_real_dir_switch = !b_fl_steer_real_dir_switch;
                        ros::param::set("/driver_steer/b_FL_target_steer_angled", b_fl_steer_real_dir_switch);
                        NLOG(info) << "FL 조향모터 방향 전환";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                    }
                }
                else
                {
                    NLOG(info) << "--- FL 조향모터 확인 테스트 취소하셨습니다.---";
                    step_completed = true;
                }
                step++;
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                user_confirmed = false;
                break;

            case 1:
                NLOG(info) << "나비프라 UI 상에서 로봇이 실제로 움직인 방향과 일치하나요?";
                NLOG(info) << "일치한다면 /nc_motor_tester/start 토픽으로 y를 입력해주세요.";
                NLOG(info) << "FL 조향모터를 다시 구동시켜봐야 된다면 /nc_motor_tester/start 토픽으로 move를 입력해주세요.";
                
                if (scenario_start_ == "move") {
                    while (fl_angle_past_ > -90 && ros::ok()) {
                        fl_angle_past_ -= 5;
                        cmdQuadVelocity(0.0, fl_angle_past_, 0.0, 0.0);
                        cnt_++;
                        rate.sleep();
                    }
                    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);
                    scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                    user_confirmed = false;
                    continue;
                }

                while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }
                if (scenario_start_ == "y" || scenario_start_ == "Y") {
                    user_confirmed = true;
                } 
                else {
                    user_confirmed = false;
                }

                if (user_confirmed) {
                        NLOG(info) << "FL 조향모터 테스트 2단계 통과했습니다.(ui 방향성 정상)";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                        result_data_list_[2] = 1;
                        result_error_list_[2] = "OK";
                } else {
                    result_data_list_[2] = 2;
                    result_error_list_[2] = "FL 피드백 방향성 이상";
                    vec_failed_scenario_name_.emplace_back("fl_steering_motor_check");

                    b_fl_steer_ui_dir_switch = !b_fl_steer_ui_dir_switch;
                    ros::param::set("/driver_steer/b_FL_feedback_steer_angled", b_fl_steer_ui_dir_switch);
                    NLOG(info) << "FL 조향모터 UI 방향 전환";
                    NLOG(info) << "다음 단계로 넘어갑니다.";
                }       
        }
    }
    // 변경된 파라미터를 param.yaml에 저장
    if (system("rosparam dump ~/navifra_solution/navicore/configs/param.yaml") != 0) {
        NLOG(error) << "Failed to dump parameters to param.yaml";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 0.1초 대기
    
    // 변수 초기화
    cnt_ = 0;
    fl_angle_past_ = 0;
    user_confirmed = false;
    step = 0;
    step_completed = false;

    saveResult();
    NLOG(info) << "--- FL 조향모터 테스트 완료되었습니다. ---";
}

void MotorTester::checkRRSteeringMotor() {
    static int step = 0;
    static bool step_completed = false;
    ros::Rate rate(10); // 10Hz
    ros::Rate wait_rate(10);

    while (!step_completed && ros::ok()) {
        switch (step) 
        {
            case 0:
                NLOG(info) << "--- RR 조향모터 테스트를 시작하겠습니다. ---";
                NLOG(info) << "--- RR 조향모터를 90도 회전하겠습니다. 실제 움직임을 확인해주세요.  ---";
                NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
                // start 토픽 받기 전까지 대기
                while (ros::ok() && scenario_start_ != "true") {
                    ros::spinOnce();
                    wait_rate.sleep();
                }
                user_confirmed = (scenario_start_ == "true");
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
        
                if (user_confirmed)
                {
                    while (rr_angle_past_ > -90 && ros::ok()) {
                        rr_angle_past_ -= 5;
                        cmdQuadVelocity(0, 0 , 0, rr_angle_past_);
                        cnt_++;
                        rate.sleep();
                    }
                    cmdQuadVelocity(0.0, 0.0, 0.0, 0.0);

                    user_confirmed = false;
                    float feedback_deg = robot_motor_info_.f_RR_steer_motor_feedback_deg;
                    NLOG(info) << "--- RR 조향모터가 움직였습니다. 바퀴가 기대하신 방향으로 움직였나요? ---";
                    NLOG(info) << "--- 움직였다면 /nc_motor_tester/start 토픽으로 y 를 입력해주세요. ---";
                    
                    while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }
                    if (scenario_start_ == "y" || scenario_start_ == "Y") {
                        user_confirmed = true;
                    }  
                    else {
                        user_confirmed = false;
                    }
                
                    if (user_confirmed && feedback_deg > -95 && feedback_deg < -85 ) {
                        NLOG(info) <<"--- RR 조향모터 테스트 1단계 통과했습니다.(실제 방향성 정상) ---";
                        result_data_list_[2] = 1;
                        result_error_list_[2] = "OK";
                    } else {
                        NLOG(info) <<"--- RR 조향모터 테스트 1단계 실패했습니다.(실제 방향성 비정상) ---";
                        result_data_list_[2] = 2;
                        result_error_list_[2] = "Direction ERROR";
                        vec_failed_scenario_name_.emplace_back("rr_steering_motor_check");
                        NLOG(info) << "--- RR 조향모터 방향성 파라미터를 바꾸겠습니다. ---";
                    
                        b_rr_steer_real_dir_switch = !b_rr_steer_real_dir_switch;
                        ros::param::set("/driver_steer/b_RR_feedback_steer_angled", b_rr_steer_real_dir_switch);
                        NLOG(info) << "RR 조향모터 방향 전환";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                    }
                }
                else
                {
                    NLOG(info) << "--- RR 조향모터 확인 테스트 취소하셨습니다.---";
                    step_completed = true;
                }
                step++;
                scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                user_confirmed = false;
                break;

            case 1:
                NLOG(info) << "나비프라 UI 상에서 로봇이 실제로 움직인 방향과 일치하나요?";
                NLOG(info) << "일치한다면 /nc_motor_tester/start 토픽으로 y를 입력해주세요.";
                NLOG(info) << "RR 조향모터를 다시 구동시켜봐야 된다면 /nc_motor_tester/start 토픽으로 move를 입력해주세요.";
                
                if (scenario_start_ == "move") {
                    while (rr_angle_past_ > -90 && ros::ok()) {
                        rr_angle_past_ -= 5;
                        cmdQuadVelocity(0, 0 , 0, rr_angle_past_);
                        cnt_++;
                        rate.sleep();
                    }
                    cmdQuadVelocity(0, 0, 0, 0);
                    scenario_start_ = ""; // 다음 시나리오를 위해 리셋
                    user_confirmed = false;
                    continue;
                }

                while (ros::ok() && scenario_start_ != "y" && scenario_start_ != "Y" &&
                           scenario_start_ != "n" && scenario_start_ != "N") {
                        ros::spinOnce();
                        wait_rate.sleep();
                    }
                if (scenario_start_ == "y" || scenario_start_ == "Y") {
                    user_confirmed = true;
                } 
                else {
                    user_confirmed = false;
                }

                if (user_confirmed) {
                        NLOG(info) << "RR 조향모터 테스트 2단계 통과했습니다.(ui 방향성 정상)";
                        NLOG(info) << "다음 단계로 넘어갑니다.";
                        result_data_list_[2] = 1;
                        result_error_list_[2] = "OK";
                } else {
                    result_data_list_[2] = 2;
                    result_error_list_[2] = "RR 피드백 방향성 이상";
                    vec_failed_scenario_name_.emplace_back("rr_steering_motor_check");

                    b_rr_steer_ui_dir_switch = !b_rr_steer_ui_dir_switch;
                    ros::param::set("/driver_steer/b_RR_target_feedback_rpmd", b_rr_steer_ui_dir_switch);
                    NLOG(info) << "RR 조향모터 UI 방향 전환";
                    NLOG(info) << "다음 단계로 넘어갑니다.";
                }       
        }
    }
    // 변경된 파라미터를 param.yaml에 저장
    if (system("rosparam dump ~/navifra_solution/navicore/configs/param.yaml") != 0) {
        NLOG(error) << "Failed to dump parameters to param.yaml";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 0.1초 대기
    
    // 변수 초기화
    cnt_ = 0;
    rr_angle_past_ = 0;
    user_confirmed = false;
    step = 0;
    step_completed = false;

    saveResult();
    NLOG(info) << "--- RR 조향모터 테스트 완료되었습니다. ---";
}

void MotorTester::checkFrontLidar() {
    NLOG(info) << "FRONT 라이다 테스트 시작합니다...";
    if (!lidar_data_check_) {
        if (front_laser_ranges_size_ != 0) {
            NLOG(info) << "FRONT 라이다 데이터 출력 확인됐습니다.";
            NLOG(info) << "FRONT 라이다 10cm 이내로 손을 대주세요...";
            NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
            ros::Rate wait_rate(10);
            // start 토픽 받기 전까지 대기
            while (ros::ok() && scenario_start_ != "true") {
                ros::spinOnce();
                wait_rate.sleep();
            }
            user_confirmed = (scenario_start_ == "true");
            scenario_start_ = ""; // 다음 시나리오를 위해 리셋

            if (user_confirmed)
            {
                NLOG(info) << "FRONT 라이다 데이터 확인 중..." << front_laser_count_;
                if (front_laser_count_ > 500) {
                    NLOG(info) << "FRONT 라이다 셋팅은 정상입니다...";
                    result_data_list_[4] = 1;
                    result_error_list_[4] = "OK";
                    lidar_data_check_ = true;
                } else {
                    NLOG(error) << "FRONT 라이다 출력이 이상합니다..."  << front_laser_count_ << " / " << " 500";
                    result_data_list_[5] = 2;
                    result_error_list_[5] = "출력 에러";
                    vec_failed_scenario_name_.emplace_back("front_lidar_check");
                    lidar_data_check_ = false;
                }
            }
            else {
                NLOG(info) << "FRONT 라이다 테스트를 스킵하셨습니다.";
                lidar_data_check_ = false; // 사용자 확인 실패 시 false 유지
            }
            user_confirmed = false;
            saveResult();
        } else {
            NLOG(error) << "FRONT 라이다 데이터 출력이 안됩니다. 라이다 셋팅을 확인해주세요...";
            result_data_list_[5] = 2;
            result_error_list_[5] = "셋팅 에러";
            vec_failed_scenario_name_.emplace_back("front_lidar_check");
            lidar_data_check_ = false;
            user_confirmed = false;
            saveResult();
        }
    } else {
        NLOG(info) << "Front 라이다 테스트 재시작 하겠습니다. ";
        NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋

        if (user_confirmed) {
            NLOG(info) << "Checking Front LIDAR data...";
            if (front_laser_count_ > 500) {
                NLOG(info) << "Front LIDAR Setting OK";
                result_data_list_[5] = 1;
                result_error_list_[5] = "OK";
            } else {
                NLOG(error) << "Front LIDAR Setting FAILED";
                result_data_list_[5] = 2;
                result_error_list_[5] = "SETTING ERROR";
                vec_failed_scenario_name_.emplace_back("front_lidar_check");
            }
        } else {
            NLOG(info) << "User did not confirm hand placement. Skipping test.";
        }
        lidar_data_check_ = false; // 테스트 후 상태 초기화
        user_confirmed = false; // 사용자 확인 상태 초기화
        saveResult();
    }
    NLOG(info) << "FRONT 라이다 테스트 완료..." << std::endl;
}

void MotorTester::checkRearLidar() {
    NLOG(info) << "REAR 라이다 테스트 시작합니다....";
    if (!lidar_data_check_) {
        if (rear_laser_ranges_size_ != 0) {
            NLOG(info) << "REAR 라이다 데이터 출력 확인됐습니다.";
            NLOG(info) << "REAR 라이다 10cm 이내로 손을 대주세요...";
            NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
            ros::Rate wait_rate(10);
            // start 토픽 받기 전까지 대기
            while (ros::ok() && scenario_start_ != "true") {
                ros::spinOnce();
                wait_rate.sleep();
            }
            user_confirmed = (scenario_start_ == "true");
            scenario_start_ = ""; // 다음 시나리오를 위해 리셋

            if (user_confirmed) {
                NLOG(info) << "REAR 라이다 데이터 확인 중..." << rear_laser_count_;
                if (rear_laser_count_ > 500) {
                    NLOG(info) << "REAR 라이다 셋팅은 정상입니다...";
                    result_data_list_[6] = 1;
                    result_error_list_[6] = "OK";
                    lidar_data_check_ = true;
                } else {
                    NLOG(error) << "REAR 라이다 출력이 이상합니다..."  << rear_laser_count_ << " / " << " 500";
                    result_data_list_[5] = 2;
                    result_error_list_[5] = "출력 에러";
                    vec_failed_scenario_name_.emplace_back("rear_lidar_check");
                    lidar_data_check_ = false;
                }
            } else {
                NLOG(info) << "REAR 라이다 테스트를 스킵하셨습니다.";
                lidar_data_check_ = false;
            }
            user_confirmed = false;
            saveResult();
        } else {
            NLOG(error) << "REAR 라이다 데이터 출력이 안됩니다. 라이다 셋팅을 확인해주세요...";
            result_data_list_[6] = 2;
            result_error_list_[6] = "셋팅 에러";
            vec_failed_scenario_name_.emplace_back("rear_lidar_check");
            lidar_data_check_ = false;
            user_confirmed = false;
            saveResult();
        }
    } else {
        NLOG(info) << "REAR 라이다 테스트 재시작 하겠습니다. ";
        NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋

        if (user_confirmed) {
            NLOG(info) << "Checking Rear LIDAR data...";
            if (rear_laser_count_ > 500) {
                NLOG(info) << "Rear LIDAR Setting OK";
                result_data_list_[6] = 1;
                result_error_list_[6] = "OK";
            } else {
                NLOG(error) << "Rear LIDAR Setting FAILED";
                result_data_list_[6] = 2;
                result_error_list_[6] = "SETTING ERROR";
                vec_failed_scenario_name_.emplace_back("rear_lidar_check");
            }
        } else {
            NLOG(info) << "User did not confirm hand placement. Skipping test.";
        }
        lidar_data_check_ = false; // 테스트 후 상태 초기화
        user_confirmed = false; // 사용자 확인 상태 초기화
        saveResult();
    }
    NLOG(info) << "REAR 라이다 테스트 완료..." << std::endl;
}

void MotorTester::checkLeftLidar() {
    NLOG(info) << "LEFT 라이다 테스트 시작합니다....";
    if (!lidar_data_check_) {
        NLOG(info) << "Checking left_laser_ranges_size_: " << left_laser_ranges_size_;
        if (left_laser_ranges_size_ != 0) {
            NLOG(info) << "LEFT 라이다 데이터 출력 확인됐습니다.";
            NLOG(info) << "LEFT 라이다 10cm 이내로 손을 대주세요...";
            NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
            ros::Rate wait_rate(10);
            // start 토픽 받기 전까지 대기
            while (ros::ok() && scenario_start_ != "true") {
                ros::spinOnce();
                wait_rate.sleep();
            }
            user_confirmed = (scenario_start_ == "true");
            scenario_start_ = ""; // 다음 시나리오를 위해 리셋

            if (user_confirmed) {
                NLOG(info) << "LEFT 라이다 데이터 확인 중..." << left_laser_count_;
                if (left_laser_count_ > 500) {
                    NLOG(info) << "LEFT 라이다 셋팅은 정상입니다...";
                    result_data_list_[7] = 1;
                    result_error_list_[7] = "OK";
                    lidar_data_check_ = true;
                } else {
                    NLOG(error) << "LEFT 라이다 출력이 이상합니다..."  << left_laser_count_ << " / " << " 500";
                    result_data_list_[5] = 2;
                    result_error_list_[5] = "출력 에러";
                    vec_failed_scenario_name_.emplace_back("left_lidar_check");
                    lidar_data_check_ = false;
                }
            } else {
                NLOG(info) << "LEFT 라이다 테스트를 스킵하셨습니다.";
                lidar_data_check_ = false;
            }
            saveResult();
        } else {
            NLOG(error) << "LEFT 라이다 데이터 출력이 안됩니다. 라이다 셋팅을 확인해주세요...";
            result_data_list_[6] = 2;
            result_error_list_[6] = "셋팅 에러";
            vec_failed_scenario_name_.emplace_back("left_lidar_check");
            lidar_data_check_ = false;
            saveResult();
        }
    } else {
        NLOG(info) << "REAR 라이다 테스트 재시작 하겠습니다. ";
        NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋

        if (user_confirmed) {
            NLOG(info) << "Checking Left LIDAR data...";
            if (left_laser_count_ > 500) {
                NLOG(info) << "Left LIDAR Setting OK";
                result_data_list_[7] = 1;
                result_error_list_[7] = "OK";
            } else {
                NLOG(error) << "Left LIDAR Setting FAILED";
                result_data_list_[7] = 2;
                result_error_list_[7] = "SETTING ERROR";
                vec_failed_scenario_name_.emplace_back("left_lidar_check");
            }
        } else {
            NLOG(info) << "User did not confirm hand placement. Skipping test.";
        }
        lidar_data_check_ = false; // 테스트 후 상태 초기화
        saveResult();
    }
    NLOG(info) << "LEFT 라이다 테스트 완료..." << std::endl;
}

void MotorTester::checkRightLidar() {
    NLOG(info) << "RIGHT 라이다 테스트 시작합니다....";
    if (!lidar_data_check_) {
        NLOG(info) << "Checking right_laser_ranges_size_: " << right_laser_ranges_size_;
        if (right_laser_ranges_size_ != 0) {
            NLOG(info) << "RIGHT 라이다 데이터 출력 확인됐습니다.";
            NLOG(info) << "RIGHT 라이다 10cm 이내로 손을 대주세요...";
            NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
            ros::Rate wait_rate(10);
            // start 토픽 받기 전까지 대기
            while (ros::ok() && scenario_start_ != "true") {
                ros::spinOnce();
                wait_rate.sleep();
            }
            user_confirmed = (scenario_start_ == "true");
            scenario_start_ = ""; // 다음 시나리오를 위해 리셋

            if (user_confirmed) {
                NLOG(info) << "RIGHT 라이다 데이터 확인 중..." << right_laser_count_;
                if (right_laser_count_ > 500) {
                    NLOG(info) << "RIGHT 라이다 셋팅은 정상입니다...";
                    result_data_list_[8] = 1;
                    result_error_list_[8] = "OK";
                    lidar_data_check_ = true;
                } else {
                    NLOG(error) << "RIGHT 라이다 출력이 이상합니다..."  << right_laser_count_ << " / " << " 500";
                    result_data_list_[5] = 2;
                    result_error_list_[5] = "출력 에러";
                    vec_failed_scenario_name_.emplace_back("right_lidar_check");
                    lidar_data_check_ = false;
                }
            } else {
                NLOG(info) << "RIGHT 라이다 테스트를 스킵하셨습니다.";
                lidar_data_check_ = false;
            }
            saveResult();
        } else {
            NLOG(error) << "RIGHT 라이다 데이터 출력이 안됩니다. 라이다 셋팅을 확인해주세요...";
            result_data_list_[6] = 2;
            result_error_list_[6] = "셋팅 에러";
            vec_failed_scenario_name_.emplace_back("right_lidar_check");
            lidar_data_check_ = false;
            saveResult();
        }
    } else {
        NLOG(info) << "Right 라이다 테스트 재시작 하겠습니다. ";
        NLOG(info) << "시작하려면 /nc_motor_tester/start 토픽으로 true를 입력해주세요.";
        ros::Rate wait_rate(10);
        // start 토픽 받기 전까지 대기
        while (ros::ok() && scenario_start_ != "true") {
            ros::spinOnce();
            wait_rate.sleep();
        }
        user_confirmed = (scenario_start_ == "true");
        scenario_start_ = ""; // 다음 시나리오를 위해 리셋

        if (user_confirmed) {
            NLOG(info) << "Checking Right LIDAR data...";
            if (right_laser_count_ > 500) {
                NLOG(info) << "Right LIDAR Setting OK";
                result_data_list_[8] = 1;
                result_error_list_[8] = "OK";
            } else {
                NLOG(error) << "Right LIDAR Setting FAILED";
                result_data_list_[8] = 2;
                result_error_list_[8] = "SETTING ERROR";
                vec_failed_scenario_name_.emplace_back("right_lidar_check");
            }
        } else {
            NLOG(info) << "User did not confirm hand placement. Skipping test.";
        }
        lidar_data_check_ = false; // 테스트 후 상태 초기화
        saveResult();
    }
    NLOG(info) << "RIGHT 라이다 테스트 완료..." << std::endl;
}

void MotorTester::StartTest()
{
    NLOG(info) << "Starting motor and sensor test for robot type: " << s_kinematics_type_;

    // FL (traction motor)
    ros::param::param<bool>("driver_traction/b_FL_traction_target_rpmd", b_fl_wheel_real_dir_switch, true);  
    ros::param::param<bool>("driver_traction/b_FL_traction_feedback_rpmd", b_fl_wheel_ui_dir_switch, true);   
    ros::param::param<bool>("driver_traction/b_FL_traction_encoderd", b_fl_wheel_enc_dir_switch, true);   
    
    // FL (steering motor)
    ros::param::param<bool>("driver_steer/b_FL_target_steer_angled", b_fl_steer_real_dir_switch, true);
    ros::param::param<bool>("driver_steer/b_FL_feedback_steer_angled", b_fl_steer_ui_dir_switch, true);
    
    // RR (traction motor)
    ros::param::param<bool>("driver_traction/b_RR_traction_target_rpmd", b_rr_wheel_real_dir_switch, true);  
    ros::param::param<bool>("driver_traction/b_RR_traction_feedback_rpmd", b_rr_wheel_ui_dir_switch, true);   
    ros::param::param<bool>("driver_traction/b_RR_traction_encoderd", b_rr_wheel_enc_dir_switch, true);
    
    // RR (steering motor)
    ros::param::param<bool>("driver_steer/b_RR_target_steer_angled", b_rr_steer_real_dir_switch, true);
    ros::param::param<bool>("driver_steer/b_RR_feedback_steer_angled", b_rr_steer_ui_dir_switch, true);
    // 테스트 시퀀스 초기화
    n_motor_scenario_count_ = 0;

    if (ros::Time::now().toSec() - odom_time_ > 2) {
        NLOG(error) << "Motor communication error! Check connection.";
        return;
    }

    // DD 타입: Front LIDAR → Rear LIDAR → FL Traction Motor → RR Traction Motor
    if (n_kinematics_ == 0) {
        NLOG(info) << "Executing DD test sequence";
        checkFrontLidar();
        n_motor_scenario_count_++;
        checkRearLidar();
        n_motor_scenario_count_++;
        checkFLTractionMotor();
        n_motor_scenario_count_++;
        checkRRTractionMotor();
        n_motor_scenario_count_++;
    }
    // QD 타입: Front LIDAR → Rear LIDAR → FL Traction Motor → RR Traction Motor → FL Steering Motor → RR Steering Motor
    else if (n_kinematics_ == 1) {
        NLOG(info) << "Executing QD test sequence";
        checkFrontLidar();
        n_motor_scenario_count_++;
        checkRearLidar();
        n_motor_scenario_count_++;
        checkFLTractionMotor();
        n_motor_scenario_count_++;
        checkRRTractionMotor();
        n_motor_scenario_count_++;
        checkFLSteeringMotor();
        n_motor_scenario_count_++;
        checkRRSteeringMotor();
        n_motor_scenario_count_++;
    }
    // SD 타입: Front LIDAR → Rear LIDAR → FL Traction Motor → FL Steering Motor
    else if (n_kinematics_ == 2) {
        NLOG(info) << "Executing SD test sequence";
        checkFrontLidar();
        n_motor_scenario_count_++;
        // checkRearLidar();
        // n_motor_scenario_count_++;
        checkFLTractionMotor();
        n_motor_scenario_count_++;
        // checkRRTractionMotor();
        // n_motor_scenario_count_++;
        checkFLSteeringMotor();
        n_motor_scenario_count_++;
        // checkRRSteeringMotor();
        // n_motor_scenario_count_++;
    }
    // OD 타입: Front LIDAR → Rear LIDAR → Left LIDAR → Right LIDAR → FL Traction Motor → RR Traction Motor → FL Steering Motor → RR Steering Motor
    else if (n_kinematics_ == 3) {
        NLOG(info) << "Executing OD test sequence";
        checkFrontLidar();
        n_motor_scenario_count_++;
        checkRearLidar();
        n_motor_scenario_count_++;
        checkLeftLidar();
        n_motor_scenario_count_++;
        checkRearLidar();
        n_motor_scenario_count_++;
        checkFLTractionMotor();
        n_motor_scenario_count_++;
        checkRRTractionMotor();
        n_motor_scenario_count_++;
        // checkFRTractionMotor();
        // n_motor_scenario_count_++;
        // checkRLTractionMotor();
        // n_motor_scenario_count_++;
        checkFLSteeringMotor();
        n_motor_scenario_count_++;
        checkRRSteeringMotor();
        n_motor_scenario_count_++;
        // checkFRSteeringMotor();
        // n_motor_scenario_count_++;
        // checkRLSteeringMotor();
        // n_motor_scenario_count_++;
    }

    NLOG(info) << " All scenarios executed \n";

    NLOG(info) << "motor scenario count: " << n_motor_scenario_count_;
    NLOG(info) << "failed scenario count: " << vec_failed_scenario_name_.size();

    for (auto& name : vec_failed_scenario_name_) {
        NLOG(info) << name;
    }
}

void MotorTester::MotorCallback(const core_msgs::MotorInfo::ConstPtr& msg) {
    robot_motor_info_ = *msg;
    MotorErrorCheck();
}
void MotorTester::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_time_ = ros::Time::now().toSec();
    robot_odom_ = *msg;
}
void MotorTester::FrontLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    front_laser_ranges_size_ = msg->ranges.size();
    front_laser_count_ = 0;
    for (const auto& range : msg->ranges) {
        if (range > 0 && range <= 1) front_laser_count_++;
    }
}
void MotorTester::RearLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    rear_laser_ranges_size_ = msg->ranges.size();
    rear_laser_count_ = 0;
    for (const auto& range : msg->ranges) {
        if (range > 0 && range <= 1) rear_laser_count_++;
    }
}
void MotorTester::LeftLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    left_laser_ranges_size_ = msg->ranges.size();
    left_laser_count_ = 0;
    for (const auto& range : msg->ranges) {
        if (range > 0 && range <= 1) left_laser_count_++;
    }
}
void MotorTester::RightLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    right_laser_ranges_size_ = msg->ranges.size();
    right_laser_count_ = 0;
    for (const auto& range : msg->ranges) {
        if (range > 0 && range <= 1) right_laser_count_++;
    }
}
void MotorTester::AnswerCallback(const std_msgs::String::ConstPtr& msg) {
    scenario_start_ = msg->data;
}
void MotorTester::MotorErrorCheck() {
    if (robot_motor_info_.f_FL_traction_motor_target_rpm == 0) {
        fl_traction_target_time_ = ros::Time::now().toSec();
        if (std::abs(robot_motor_info_.f_FL_traction_motor_feedback_rpm) < 5) {
            fl_traction_feedback_time_ = ros::Time::now().toSec();
        }
    }
    if (robot_motor_info_.f_RR_traction_motor_target_rpm == 0) {
        rr_traction_target_time_ = ros::Time::now().toSec();
        if (std::abs(robot_motor_info_.f_RR_traction_motor_feedback_rpm) < 5) {
            rr_traction_feedback_time_ = ros::Time::now().toSec();
        }
    }
    if (robot_motor_info_.f_FL_steer_motor_target_rpm == 0) {
        fl_steer_target_time_ = ros::Time::now().toSec();
        if (std::abs(robot_motor_info_.f_FL_steer_motor_feedback_rpm) < 5) {
            fl_steer_feedback_time_ = ros::Time::now().toSec();
        }
    }
    if (robot_motor_info_.f_RR_steer_motor_target_rpm == 0) {
        rr_steer_target_time_ = ros::Time::now().toSec();
        if (std::abs(robot_motor_info_.f_RR_steer_motor_feedback_rpm) < 5) {
            rr_steer_feedback_time_ = ros::Time::now().toSec();
        }
    }

    if (fl_traction_target_time_ - fl_traction_feedback_time_ > 30) {
        NLOG(error) << "ERROR: Check FL traction motor wiring!";
    }
    if (rr_traction_target_time_ - rr_traction_feedback_time_ > 30) {
        NLOG(error) << "ERROR: Check RR traction motor wiring!";
    }
    if (fl_steer_target_time_ - fl_steer_feedback_time_ > 30) {
        NLOG(error) << "ERROR: Check FL steering motor wiring!";
    }
    if (rr_steer_target_time_ - rr_steer_feedback_time_ > 30) {
        NLOG(error) << "ERROR: Check RR steering motor wiring!";
    }
}

void MotorTester::cmdVelocity(float linear_x, float angular_z) {
    geometry_msgs::Twist vel;
    vel.linear.x = linear_x;
    vel.angular.z = angular_z;
    cmd_vel_pub_.publish(vel);
}
void MotorTester::cmdQuadVelocity(float linearVel1, float steerAngle1, float linearVel2, float steerAngle2) {
    std_msgs::String vel;
    vel.data = std::to_string(linearVel1) + "/" + std::to_string(steerAngle1) + "/" +
               std::to_string(linearVel2) + "/" + std::to_string(steerAngle2);
    quad_cmd_pub_.publish(vel);
}

void MotorTester::saveResult() {
    std::ofstream file(package_path_ + "/sensor_check_result.txt");
    if (file.is_open()) {
        file << robot_name_ << "/";
        for (int i = 0; i < 8; i++) {
            file << result_data_list_[i] << (i < 7 ? "/" : "\n");
        }
        for (int i = 0; i < 8; i++) {
            file << result_error_list_[i] << (i < 7 ? "/" : "/");
        }
        file.close();
        NLOG(info) << "Results saved to sensor_check_result.txt";
    } else {
        NLOG(error) << "Failed to save results!";
    }
}

void MotorTester::loadResult() {
    std::ifstream file(package_path_ + "/sensor_check_result.txt");
    if (file.is_open()) {
        std::string line;
        std::getline(file, line);
        std::stringstream ss(line);
        std::string token;

        std::getline(ss, token, '/');
        if (token != robot_name_) {
            NLOG(warning) << "Saved robot type mismatch: " << token << " vs " << robot_name_;
            file.close();
            return;
        }

        for (int i = 0; i < 8 && std::getline(ss, token, '/'); i++) {
            result_data_list_[i] = std::stoi(token);
        }

        std::getline(file, line);
        ss.clear();
        ss.str(line);
        for (int i = 0; i < 8 && std::getline(ss, token, '/'); i++) {
            result_error_list_[i] = token;
        }
        file.close();
        NLOG(info) << "Results loaded from sensor_check_result.txt";
    } else {
        NLOG(info) << "No previous results found.";
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_motor_tester_node");
    MotorTester tester;
    // ros::spin();
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
