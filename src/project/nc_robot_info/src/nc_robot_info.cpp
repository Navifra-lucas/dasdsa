#include "nc_robot_info.hpp"

using namespace std;
RobotInfo::RobotInfo()
{
    if (!nh.getParam("/nc_robot_info_node/pub_interval", pub_interval)) {
        pub_interval = 1.0;
        ROS_WARN("pub_interval parameter not set, using default value: 0.5");
    }
    else {
        ROS_INFO("Received pub_interval value: %f", pub_interval);
    }
    const char* homeDir = getenv("HOME");
    if (homeDir) {
        s_filename_ = std::string(homeDir) + "/navifra_solution/navicore/configs/pose.txt";
        s_mile_filename_ = std::string(homeDir) + "/navifra_solution/navicore/configs/mileage.txt";
    }
    
    if (!nh.getParam("/ipc_setting/cpu_overload_threshold", n_cpu_overload_percent_)) {
        n_cpu_overload_percent_ = 80; // Default CPU overload threshold
    }
    if (!nh.getParam("/ipc_setting/cpu_overload_time_second", n_cpu_overload_time_second_)) {
        n_cpu_overload_time_second_ = 20; // Default CPU overload threshold
    }
    if (!nh.getParam("/ipc_setting/cpu_temper_warning_c", n_cpu_temper_warning_c_)) {
        n_cpu_temper_warning_c_ = 60; // Default CPU overload threshold
    }
    if (!nh.getParam("/ipc_setting/mem_overload_percent", n_mem_overload_percent_)) {
        n_mem_overload_percent_ = 80; // Default CPU overload threshold
    }
    if (!nh.getParam("/ipc_setting/mem_overload_time_second", n_mem_overload_time_second_)) {
        n_mem_overload_time_second_ = 20; // Default CPU overload threshold
    }
    if (!nh.getParam("/ipc_setting/disk_avaliable_gbytes", n_disk_avaliable_gbytes_)) {
        n_disk_avaliable_gbytes_ = 10; // Default CPU overload threshold
    }
    if (!nh.getParam("/ipc_setting/disk_usage_percent", n_disk_usage_percent_)) {
        n_disk_usage_percent_ = 90; // Default CPU overload threshold
    }
    if (!nh.getParam("/ipc_setting/disk_usage_time_second", n_disk_usage_time_second_)) {
        n_disk_usage_time_second_ = 20; // Default CPU overload threshold
    }
    

    str_robot_log_pub_ = nh.advertise<std_msgs::String>("/navifra/log", 5, true);
    str_custom_pub_ = nh.advertise<std_msgs::String>("/etc_custom_message", 5, true);
    str_total_mileage_pub_ = nh.advertise<std_msgs::Float64>("mileage_total", 5, true);
    str_local_mileage_pub_ = nh.advertise<std_msgs::Float64>("mileage_local", 5, true);
    etc_info_pub_ = nh.advertise<core_msgs::EtcInfo>("/etc_info", 5, true);
    str_warn_pub_ = nh.advertise<std_msgs::String>("/navifra/warning", 5, true);
    str_error_pub_ = nh.advertise<std_msgs::Int64>("/navifra/error", 5, true);

    initpose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    initpose_correct_position_pub_ = nh.advertise<std_msgs::String>("correction_init_pos", 1);

    clock_pub_ = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

    robot_info_pub_timer_ = nh.createTimer(ros::Duration(pub_interval), &RobotInfo::PublishData, this);

    RosSubscriber();
    // th0_ = boost::thread(boost::bind(&RobotInfo::ClockThread, this));
}

// set subscriber
void RobotInfo::RosSubscriber()
{
    navifra_info_sub = nh.subscribe("navifra/info", 1, &RobotInfo::NavifraStatusDetailCallback, this);
    motor_info_sub = nh.subscribe("motor_info", 10, &RobotInfo::MotorCallback, this);
    lidar_info_sub = nh.subscribe("lidar_info", 1, &RobotInfo::LidarInfo, this);
    motion_info_sub = nh.subscribe("motion_info", 1, &RobotInfo::MotionInfo, this);
    localize_info_sub = nh.subscribe("localize_info", 1, &RobotInfo::LocalizeInfo, this);
    answer_report_sub = nh.subscribe("/answer/report", 1, &RobotInfo::LocalizeReport, this);
    imu_use_sub = nh.subscribe("/navifra/imu_use", 1, &RobotInfo::ImuUseFlag, this);
    imu_data_sub = nh.subscribe("/navifra/imu", 1, &RobotInfo::ImuData, this);
    task_info_sub = nh.subscribe("nc_task_manager/task_info", 1, &RobotInfo::onTaskInfo, this);
}

void RobotInfo::NavifraStatusDetailCallback(const core_msgs::NavicoreStatus::ConstPtr& msg)
{
    last_robot_msg_time_ = ros::Time::now();

    std::ostringstream robot_data_stream;
    m_navi_ = *msg;
    robot_data_stream << msg->s_status << " "
                      << "cn:" << msg->s_current_node << " "
                      << "nn:" << msg->s_next_node << " "
                      << "gn:" << msg->s_goal_node << " "
                      << "rp:" << msg->f_robot_pos_x_m << " " << msg->f_robot_pos_y_m << " " << msg->f_robot_pos_deg << " "
                      << "gp:" << msg->f_goal_pos_x_m << " " << msg->f_goal_pos_y_m << " " << msg->f_goal_pos_deg << " "
                      << "vx:" << msg->f_robot_current_linear_vel_x << " "
                      << "vy:" << msg->f_robot_current_linear_vel_y << " "
                      << "vw:" << msg->f_robot_current_angular_vel_w << " "
                      << "mile:" << msg->f_mileage_total << " "
                      << "mile_local:" << msg->f_mileage_local;

    robot_status_data_ = robot_data_stream.str();

    static int n_count = 0;
    n_count++;
    if (n_count > 10) {
        n_count = 0;
        // mileage
        SaveMileage();

        // pose.txt
        SavePos();
    }
    vec_warning_ = msg->s_warning;

    std_msgs::String ui_data;
    ui_data.data = GetUiDate();
    str_custom_pub_.publish(ui_data);
}

// 안전하게 파일을 열기
void RobotInfo::openFile(const std::string& s_name)
{
    stm_file_.open(s_name.c_str(), std::ios::trunc);  // 기존 파일 유지, 새 데이터 추가
    if (!stm_file_) {
        std::cerr << "파일을 열 수 없습니다: " << s_name << std::endl;
    }
}

// 파일을 안전하게 닫기
void RobotInfo::closeFile()
{
    if (stm_file_.is_open()) {
        stm_file_.flush();  // 버퍼에 있는 데이터를 디스크에 강제 저장
        stm_file_.close();
    }
}

// 파일을 강제 플러시 (데이터 유실 방지)
void RobotInfo::forceSync(const std::string& s_name)
{
    if (stm_file_.is_open()) {
        stm_file_.flush();
        int fd = open(s_name.c_str(), O_RDWR);
        if (fd != -1) {
            fsync(fd);  // 강제 디스크 동기화
            close(fd);
        }
    }
}

void RobotInfo::SavePos()
{
    static float f_pos_x_pre = 0;
    static float f_pos_y_pre = 0;
    static float f_pos_angle_pre = 0;

    float f_dist = hypot(m_navi_.f_robot_pos_x_m-f_pos_x_pre, m_navi_.f_robot_pos_y_m-f_pos_y_pre);
    float f_angle = fabs(m_navi_.f_robot_pos_deg - f_pos_angle_pre);
    // 1cm이상 변할 경우에 저장
    if ((fabs(m_navi_.f_robot_pos_x_m) > 0.001 || fabs(m_navi_.f_robot_pos_y_m) > 0.001) && (f_dist > 0.01 || f_angle > 2)) {
        std::lock_guard<std::mutex> lock(fileMutex);
        f_pos_x_pre = m_navi_.f_robot_pos_x_m;
        f_pos_y_pre = m_navi_.f_robot_pos_y_m;
        f_pos_angle_pre = m_navi_.f_robot_pos_deg;
        if (!stm_file_.is_open())
            openFile(s_filename_);  // 파일이 닫혔으면 다시 열기
        stm_file_ << to_string(m_navi_.f_robot_pos_x_m) << "\n"
                  << to_string(m_navi_.f_robot_pos_y_m) << "\n"
                  << to_string(m_navi_.f_robot_pos_deg) << std::endl;
        forceSync(s_filename_);  // 강제 저장
        closeFile();
    }

    static bool b_initpose = false;
    static auto start_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

    if (elapsed_time >= 5 && elapsed_time < 60 && !b_initpose && (fabs(m_navi_.f_robot_pos_x_m) < 0.001 && fabs(m_navi_.f_robot_pos_y_m) < 0.001)) 
    {  // 10~60초 사이에 제대로 위치초기화 안되면 initpose
        std::ifstream pose_file(s_filename_);
        if (pose_file.is_open()) {
            std::string line;
            std::vector<float> pose_vector;
            while (std::getline(pose_file, line)) {
                std::istringstream iss(line);
                float value;
                while (iss >> value) {
                    pose_vector.push_back(value);
                }
            }
            pose_file.close();
            if (pose_vector.size() == 3) {
                b_initpose = true;
                LOG_INFO("Read initial pose: {}, {}, {}", pose_vector[0], pose_vector[1], pose_vector[2]);
                float f_x = pose_vector[0];
                float f_y = pose_vector[1]; 
                float f_a = pose_vector[2] * M_PI / 180;
                tf2::Quaternion quat;
                quat.setRPY(0, 0, f_a);
                geometry_msgs::PoseWithCovarianceStamped initPoseMsg;
                initPoseMsg.pose.pose.position.x = f_x;
                initPoseMsg.pose.pose.position.y = f_y;
                initPoseMsg.pose.pose.position.z = 0;
                initPoseMsg.pose.pose.orientation.x = quat.x();
                initPoseMsg.pose.pose.orientation.y = quat.y();
                initPoseMsg.pose.pose.orientation.z = quat.z();
                initPoseMsg.pose.pose.orientation.w = quat.w();
                initpose_pub_.publish(initPoseMsg);
                LOG_INFO("Published initial pose: x={}, y={}, angle={}", f_x, f_y, f_a);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                initpose_correct_position_pub_.publish(std_msgs::String());
            }
            else
            {
                LOG_ERROR("Failed to open initial pose file");

            }
        }
        else {
            LOG_ERROR("Failed to open initial pose file");
        }
    }

}

void RobotInfo::SaveMileage()
{
    static float f_pre_x = m_navi_.f_robot_pos_x_m;
    static float f_pre_y = m_navi_.f_robot_pos_y_m;
    static bool b_init = false;
    float f_dist = hypot(m_navi_.f_robot_pos_x_m - f_pre_x, m_navi_.f_robot_pos_y_m - f_pre_y);
    if (f_dist > 3)  // 3m이상 점프면 이상한것. 그냥 초기화
    {
        f_pre_x = m_navi_.f_robot_pos_x_m;
        f_pre_y = m_navi_.f_robot_pos_y_m;
    }
    else if (f_dist > 0.1 || b_init == false) {  // 0.1m 이상 변화시 마일리지 쌓기
        f_pre_x = m_navi_.f_robot_pos_x_m;
        f_pre_y = m_navi_.f_robot_pos_y_m;

        float f_mileage = 0;
        std::ifstream in(s_mile_filename_);

        if (in.is_open()) {
            string line;
            if (getline(in, line)) {
                f_mileage = stof(line);
            }
            in.close();

            if (b_init)
                f_mileage += f_dist;

            std::lock_guard<std::mutex> lock(fileMutex);
            if (!stm_file_.is_open())
                openFile(s_mile_filename_);  // 파일이 닫혔으면 다시 열기
            stm_file_ << to_string(f_mileage) << std::endl;
            forceSync(s_mile_filename_);  // 강제 저장
            closeFile();
            std_msgs::Float64 f_mileage_msg;
            f_mileage_msg.data = f_mileage;
            str_total_mileage_pub_.publish(f_mileage_msg);
            str_local_mileage_pub_.publish(f_mileage_msg);
            core_msgs::EtcInfo msg_etc;
            msg_etc.b_mileage_total_update = true;
            msg_etc.b_mileage_local_update = true;
            msg_etc.f_mileage_total = f_mileage;
            msg_etc.f_mileage_local = f_mileage;
            etc_info_pub_.publish(msg_etc);
        }
        else {
            NLOG(severity_level::error) << "파일을 찾을 수 없습니다.";
            {
                std::lock_guard<std::mutex> lock(fileMutex);
                openFile(s_mile_filename_);
                f_mileage = 0.0f;
                stm_file_ << std::to_string(f_mileage) << std::endl;
                forceSync(s_mile_filename_);
                closeFile();
            }
        }
    }
}

void RobotInfo::MotorCallback(const core_msgs::MotorInfo::ConstPtr& msg)
{
    last_motor_msg_time_ = ros::Time::now();
    m_motor_ = *msg;
    std::ostringstream motor_data_stream;

    motor_data_stream << "TC:" << msg->f_FL_traction_motor_current << "," << msg->f_RR_traction_motor_current << " "
                      << "TV:" << msg->f_FL_traction_motor_voltage << "," << msg->f_RR_traction_motor_voltage << " "
                      << "TTR:" << msg->f_FL_traction_motor_target_rpm << "," << msg->f_RR_traction_motor_target_rpm << " "
                      << "TFR:" << msg->f_FL_traction_motor_feedback_rpm << "," << msg->f_RR_traction_motor_feedback_rpm << " "
                      << "TFRZ:" << msg->f_FL_traction_motor_feedback_rpm_hz << "," << msg->f_RR_traction_motor_feedback_rpm_hz << " "
                      << "TME:" << int(msg->f_FL_traction_motor_encoder) << "," << int(msg->f_RR_traction_motor_encoder) << " "
                      << "TMEZ:" << msg->f_FL_traction_motor_encoder_hz << "," << msg->f_RR_traction_motor_encoder_hz << " "
                      << "STD:" << msg->f_FL_steer_motor_target_deg << "," << msg->f_RR_steer_motor_target_deg << " "
                      << "SFD:" << msg->f_FL_steer_motor_feedback_deg << "," << msg->f_RR_steer_motor_feedback_deg << " "
                      << "STR:" << msg->f_FL_steer_motor_target_rpm << "," << msg->f_RR_steer_motor_target_rpm << " "
                      << "SFR:" << msg->f_FL_steer_motor_feedback_rpm << "," << msg->f_RR_steer_motor_feedback_rpm << " "
                      << "SC:" << msg->f_FL_steer_motor_current << "," << msg->f_RR_steer_motor_current << " "
                      << "SAFD:" << msg->f_FL_steer_absencoder_feedback_deg << "," << msg->f_RR_steer_absencoder_feedback_deg << " ";

    motor_data_ = motor_data_stream.str();
}

void RobotInfo::LidarInfo(const core_msgs::LidarInfoMsg::ConstPtr& msg)
{
    last_lidar_msg_time_ = ros::Time::now();

    std::ostringstream lidar_data_stream;

    ros::Time current_time = ros::Time::now();
    std::time_t tt = current_time.toSec();
    std::tm* t = std::gmtime(&tt);
}

void RobotInfo::MotionInfo(const core_msgs::MotionInfo::ConstPtr& msg)
{
    m_motion_ = *msg;
}

void RobotInfo::LocalizeInfo(const core_msgs::LocalizeInfo::ConstPtr& msg)
{
    m_local_ = *msg;

    static int n_count_angle = 0; 
    // static int n_count_dist = 0; 

    if(fabs(m_local_.f_correct_deg) > 1)
    {
        n_count_angle++;
    }
    else
    {
        n_count_angle = 0;
    }

    if(n_count_angle > 20)
    {
        std_msgs::String warn_msg;
        warn_msg.data = "[lo] angle slip detect";
        str_warn_pub_.publish(warn_msg);
    }
}

void RobotInfo::LocalizeReport(const std_msgs::String::ConstPtr& msg)
{
    static int n_count_localize = 0;
    n_count_localize++;
    if(n_count_localize > 10)
    {
        n_count_localize = 0;
        std_msgs::String s_msg;
        s_msg.data = "Local Data : "+(*msg).data;
        str_robot_log_pub_.publish(s_msg);
    }    
}

void RobotInfo::ImuUseFlag(const std_msgs::Bool::ConstPtr& msg)
{
    b_use_imu_ = msg->data;
}

void RobotInfo::ImuData(const sensor_msgs::Imu::ConstPtr& msg)
{
    f_imu_angular_vel_ = msg->angular_velocity.z;
}

std::vector<std::string> RobotInfo::splitString(const std::string& str, char delimiter)
{
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        result.push_back(token);  // 빈 문자열도 허용
    }

    // 문자열이 구분자로 끝날 경우 마지막 빈 토큰 추가
    if (!str.empty() && str.back() == delimiter) {
        result.push_back("");
    }

    // 특수 케이스: 입력이 구분자만 하나일 때 (예: "/")
    if (str == std::string(1, delimiter)) {
        result = {"", ""};
    }

    return result;
}

void RobotInfo::onTaskInfo(const std_msgs::String msg)
{
    try {
        std::vector<std::string> vec_uuid = splitString(msg.data, '/');
        if (vec_uuid.size() != 2) {
            NLOG(error) << "Invalid task info format: " << msg.data;
            return;
        }
        s_current_task_ = vec_uuid[0];
        s_previous_task_ = vec_uuid[1];
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what() << '\n';
    }
}


std::string RobotInfo::exec(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::istringstream stream(cmd);
    std::string word;

    bool b_is_value = false;  // 유효성 플래그
    while (stream >> word) {
        // 유효하지 않은 조건: "sudo"는 무시, 유효하지 않은 단어는 "|" 또는 "<" 또는 ">"
        if (word == "sudo")
            continue;
        if (word[0] == '|' || word[0] == '<' || word[0] == '>')
            continue;

        b_is_value = true;
        break;
    }

    // 유효하지 않은 경우 빈 문자열 반환
    if (!b_is_value) {
        return "";
    }

    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

string RobotInfo::GetPcDate()
{
    std::string cpu_data = "";
    std::string cpu_temper = "";
    std::string cpu_memory_total_bytes = "";
    std::string cpu_memory_total_gbytes = "";
    std::string cpu_memory_used_bytes = "";
    std::string cpu_memory_used_gbytes = "";
    std::string cpu_top1_process_name = "";
    std::string cpu_top1_process_use = "";
    std::string cpu_top2_process_name = "";
    std::string cpu_top2_process_use = "";
    std::string cpu_top3_process_name = "";
    std::string cpu_top3_process_use = "";
    std::string mem_top1_process_name = "";
    std::string mem_top1_process_use = "";
    std::string mem_top2_process_name = "";
    std::string mem_top2_process_use = "";
    std::string mem_top3_process_name = "";
    std::string mem_top3_process_use = "";
    std::string io_total = "";
    std::string io_top1_ = "";
    std::string disk_total_gbytes = "";
    std::string disk_available_gbytes = "";

    try {
        cpu_data = exec("top -bn1 | head -n 10 | grep \"%Cpu\" | sed 's/.*, *\\([0-9.]*\\)%* id.*/\\1/' | awk '{print 100 - $1}'");
        cpu_temper = exec("sensors | grep 'Core 0' | awk '{print $3}' | tr -d '+°C'");
        cpu_memory_total_bytes = exec("free | grep ^Mem | awk '{print $2}'");
        cpu_memory_total_gbytes = roundTo(stof(cpu_memory_total_bytes) / (1024 * 1024), 2);
        cpu_memory_used_bytes = exec("free | grep ^Mem | awk '{print $3}'");
        cpu_memory_used_gbytes = roundTo(stof(cpu_memory_used_bytes) / (1024 * 1024), 2);
        cpu_top1_process_name = exec("top -b -n 1 | awk 'NR == 8 {print $12}'");
        cpu_top1_process_use = exec("top -b -n 1 | awk 'NR == 8 {print $9}'");
        cpu_top2_process_name = exec("top -b -n 1 | awk 'NR == 9 {print $12}'");
        cpu_top2_process_use = exec("top -b -n 1 | awk 'NR == 9 {print $9}'");
        cpu_top3_process_name = exec("top -b -n 1 | awk 'NR == 10 {print $12}'");
        cpu_top3_process_use = exec("top -b -n 1 | awk 'NR == 10 {print $9}'");
        mem_top1_process_name = exec("top -o %MEM -b -n 1 | awk 'NR == 8 {print $12}'");
        mem_top1_process_use = exec("top -o %MEM -b -n 1 | awk 'NR == 8 {print $10}'");
        mem_top2_process_name = exec("top -o %MEM -b -n 1 | awk 'NR == 9 {print $12}'");
        mem_top2_process_use = exec("top -o %MEM -b -n 1 | awk 'NR == 9 {print $10}'");
        mem_top3_process_name = exec("top -o %MEM -b -n 1 | awk 'NR == 10 {print $12}'");
        mem_top3_process_use = exec("top -o %MEM -b -n 1 | awk 'NR == 10 {print $10}'");
    }
    catch (std::exception& e) {
        NLOG(error) << "error1" << e.what();
    }

    try {
        io_total = exec("sudo iotop -b -n 1 | awk 'NR == 2'");
        io_top1_ = exec("sudo iotop -b -n 1 | awk 'NR == 4 {print $4 $5 $6 $7 $9}'");
    }
    catch (std::exception& e) {
        NLOG(error) << "error2" << e.what();
    }

    try {
        // 전체 디스크 용량 가져오기 (GB)
        disk_total_gbytes = roundTo(stof(exec("df --total --block-size=1G | grep total | awk '{print $2}'")), 2);

        // 남은 디스크 용량 가져오기 (GB)
        disk_available_gbytes = roundTo(stof(exec("df --total --block-size=1G | grep total | awk '{print $4}'")), 2);
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what();
    }

    std::string pc_log = "cpu use : " + cpu_data + "% cpu temper : " + cpu_temper + "°C Memory total : " + cpu_memory_total_gbytes +
        "GB Memory used : " + cpu_memory_used_gbytes + "GB " + "cpu top1 : " + cpu_top1_process_name + " " + cpu_top1_process_use + "% " +
        "cpu top2 : " + cpu_top2_process_name + " " + cpu_top2_process_use + "% " + "cpu top3 : " + cpu_top3_process_name + " " +
        cpu_top3_process_use + "% " + "memory top1 : " + mem_top1_process_name + " " + mem_top1_process_use + "% " +
        "memory top2 : " + mem_top2_process_name + " " + mem_top2_process_use + "% " + "memory top3 : " + mem_top3_process_name + " " +
        mem_top3_process_use + "%" + " io_total : " + io_total + " io_top1_ : " + io_top1_ + " total disk : " + disk_total_gbytes + "GB " +
        "available disk : " + disk_available_gbytes + "GB";

    pc_log.erase(std::remove(pc_log.begin(), pc_log.end(), '\n'), pc_log.end());

    auto now = std::chrono::steady_clock::now();

    try {
        if (stof(cpu_data) > n_cpu_overload_percent_) {
            std_msgs::String warn_msg;
            warn_msg.data = "CPU use "+to_string(n_cpu_overload_percent_)+"% high";
            str_warn_pub_.publish(warn_msg);

            NLOG(info) << "CPU use " << n_cpu_overload_percent_ << "% high";

            if (!b_cpu_high_) {
                t_cpu_high_ = now;
                b_cpu_high_ = true;
            } else if (std::chrono::duration_cast<std::chrono::seconds>(now - t_cpu_high_).count() >= n_cpu_overload_time_second_) {
                std_msgs::Int64 err;
                err.data = core_msgs::NaviAlarm::ERROR_CPU_OVERLOAD;
                str_error_pub_.publish(err);
                LOG_INFO("CPU use 80% OVERLOAD");

                t_cpu_high_ = now;
            }
        }
        else {
            b_cpu_high_ = false;
        }
        if (stof(cpu_temper) > n_cpu_temper_warning_c_) {
            std_msgs::String warn_msg;
            warn_msg.data = "CPU temper "+to_string(n_cpu_temper_warning_c_)+"°C high";
            str_warn_pub_.publish(warn_msg);
        }
        float f_memory_percent = stof(cpu_memory_used_gbytes) / stof(cpu_memory_total_gbytes) * 100;
        if (f_memory_percent > n_mem_overload_percent_) {
            std_msgs::String warn_msg;
            warn_msg.data = "Memory use "+to_string(n_mem_overload_percent_)+"% high";
            str_warn_pub_.publish(warn_msg);

            if (!b_mem_high_) {
                t_mem_high_ = now;
                b_mem_high_ = true;
            } else if (std::chrono::duration_cast<std::chrono::seconds>(now - t_mem_high_).count() >= n_mem_overload_time_second_) {
                std_msgs::Int64 err;
                err.data = core_msgs::NaviAlarm::ERROR_MEMORY_OVERLOAD;
                str_error_pub_.publish(err);
                LOG_INFO("MEMORY use 80% OVERLOAD");

                t_mem_high_ = now;
            }
        }
        else {
            b_mem_high_ = false;
        }
        if (stof(disk_available_gbytes) < n_disk_avaliable_gbytes_) {
            std_msgs::String warn_msg;
            warn_msg.data = "Disk usable "+to_string(n_disk_avaliable_gbytes_)+"GB low";
            str_warn_pub_.publish(warn_msg);
        }

        float f_disk_total = stof(exec("df --total --block-size=1G | grep total | awk '{print $2}'"));
        float f_disk_used = stof(exec("df --total --block-size=1G | grep total | awk '{print $3}'"));
        float f_disk_usage = (f_disk_used / f_disk_total) * 100.0f;

        if (f_disk_usage > n_disk_usage_percent_) {
            if (!b_disk_high_) {
                t_disk_high_ = now;
                b_disk_high_ = true;
            } else if (std::chrono::duration_cast<std::chrono::seconds>(now - t_disk_high_).count() >= n_disk_usage_time_second_) {
                std_msgs::Int64 err;
                err.data = core_msgs::NaviAlarm::ERROR_DISK_OVERLOAD;
                str_error_pub_.publish(err);
                LOG_INFO("DISK use 90% OVERLOAD");
                
                t_disk_high_ = now;
            }
        } else {
            b_disk_high_ = false;
        }
    
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what();
    }

    return pc_log;
}

std::string RobotInfo::getCurrentTime()
{
    std::time_t now = std::time(nullptr);
    std::tm localTime = *std::localtime(&now);

    std::ostringstream oss;
    oss << (localTime.tm_mon + 1) << "/"  // 월 (0부터 시작하므로 +1)
        << localTime.tm_mday << " "  // 일
        << localTime.tm_hour << ":"  // 시
        << localTime.tm_min << ":"  // 분
        << localTime.tm_sec;  // 초

    return oss.str();
}

std::string RobotInfo::roundTo(double value, int n)
{
    double factor = std::pow(10.0, n);
    double factor2 = round(value * factor) / factor;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(n) << factor2;
    return oss.str();
}

std::string RobotInfo::GetUiDate()
{
    string motor_steer = roundTo(m_motor_.f_FL_steer_motor_feedback_deg, 1) + "/" + roundTo(m_motor_.f_RR_steer_motor_feedback_deg, 1);
    string motor_steer_one = roundTo(m_motor_.f_FL_steer_motor_feedback_deg, 1);
    string motor_steer_four = roundTo(m_motor_.f_FL_steer_motor_feedback_deg, 1) + "/" +
        roundTo(m_motor_.f_FR_steer_motor_feedback_deg, 1) + "/" + roundTo(m_motor_.f_RL_steer_motor_feedback_deg, 1) + "/" +
        roundTo(m_motor_.f_RR_steer_motor_feedback_deg, 1);
    string motor_trpm = roundTo(m_motor_.f_FL_traction_motor_target_rpm, 0) + "/" + roundTo(m_motor_.f_RR_traction_motor_target_rpm, 0);
    string motor_trpm_four = roundTo(m_motor_.f_FL_traction_motor_target_rpm, 0) + "/" +
        roundTo(m_motor_.f_FR_traction_motor_target_rpm, 0) + "/" + roundTo(m_motor_.f_RL_traction_motor_target_rpm, 0) + "/" +
        roundTo(m_motor_.f_RR_traction_motor_target_rpm, 0);
    string motor_frpm = roundTo(m_motor_.f_FL_traction_motor_feedback_rpm, 0) + "/" + roundTo(m_motor_.f_RR_traction_motor_feedback_rpm, 0);
    string motor_frpm_four = roundTo(m_motor_.f_FL_traction_motor_feedback_rpm, 0) + "/" +
        roundTo(m_motor_.f_FR_traction_motor_feedback_rpm, 0) + "/" + roundTo(m_motor_.f_RL_traction_motor_feedback_rpm, 0) + "/" +
        roundTo(m_motor_.f_RR_traction_motor_feedback_rpm, 0);

    string local_mode = m_local_.s_localizer_mode;
    string cmd_vel = roundTo(m_navi_.f_robot_target_linear_vel_x, 2) + "," + roundTo(m_navi_.f_robot_target_linear_vel_y, 2) + "," +
        roundTo(m_navi_.f_robot_target_angular_vel_w, 2);
    std::string time_string = getCurrentTime();
    int n_kinematics_type = 0;
    ros::param::param<int>("motion_base/n_kinematics_type", n_kinematics_type, 0);  // 0: DD, 1: QD , 2: SD , 3:QDOct

    if(local_mode.empty()) local_mode = "loading";
    if(local_mode == "map") local_mode = "ex";

    if(b_use_imu_) local_mode+="_I";
    string s_msg = "[" + local_mode + "]" + time_string + " wn!:" + to_string(vec_warning_.size()) + "\n cmd : " + cmd_vel;

    if (n_kinematics_type == 0)  // dd
        s_msg = s_msg + "\n trpm : " + motor_trpm + "\n frpm : " + motor_frpm;
    if (n_kinematics_type == 1)  // qd
        s_msg = s_msg + "\n st : " + motor_steer + "\n trpm : " + motor_trpm + "\n frpm : " + motor_frpm;
    if (n_kinematics_type == 2)  // sd
        s_msg = s_msg + "\n st:" + motor_steer_one + "\n trpm : " + motor_trpm + "\n frpm : " + motor_frpm;
    if (n_kinematics_type == 3)  // qd oct
        s_msg = s_msg + "\n st : " + motor_steer_four + "\n trpm : " + motor_trpm_four + "\n frpm : " + motor_frpm_four;

    s_msg = s_msg + "\nIMU_deg/s : "+ roundTo(f_imu_angular_vel_*180/M_PI, 2) + "\n";

    if (vec_warning_.size() > 0) {
        s_msg = s_msg + "\n warn list : ";
        for (int i = 0; i < vec_warning_.size(); i++) {
            s_msg = s_msg + vec_warning_[i] + "\n";
        }
    }

    string s_current_task_short = s_current_task_;
    if(s_current_task_.length() > 4)
        s_current_task_short = s_current_task_.substr(0,4);
    
    string s_previous_task_short = s_previous_task_;;
    if(s_previous_task_.length() > 4)
        s_previous_task_short = s_previous_task_.substr(0,4);

    s_msg = s_msg + "\n Task : " + s_current_task_short + " / " + s_previous_task_short;
    return s_msg;
}

void RobotInfo::ClockThread()
{
    std::cout<<"test"<<std::endl;
    ros::Duration sim_time(0.0);  // 시뮬레이션 시간 (0부터 시작)
    const double step_sec = 0.005;
    while (b_running_)
    {
        sim_time += ros::Duration(step_sec);  // 100Hz 기준
        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = ros::Time(sim_time.toSec());
        clock_pub_.publish(clock_msg);
        usleep(static_cast<useconds_t>(step_sec * 1e6));
    }
}

void RobotInfo::PublishData(const ros::TimerEvent& e)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!IsTopicAlive(last_robot_msg_time_)) {
        robot_status_data_ = "Robot Error: Not Received";
    }

    if (!IsTopicAlive(last_motor_msg_time_)) {
        motor_data_ = "Motor Error: Not Received";
    }

    if (!IsTopicAlive(last_lidar_msg_time_)) {
        lidar_data_ = "Lidar Error: Not Received";
    }

    std_msgs::String pc_data;
    pc_data.data = "PC Data> " + GetPcDate();
    str_robot_log_pub_.publish(pc_data);
    std_msgs::String robot_data;
    robot_data.data = "Robot Data> " + robot_status_data_;
    str_robot_log_pub_.publish(robot_data);
    std_msgs::String motor_data;
    motor_data.data = "Motor Data> " + motor_data_;
    str_robot_log_pub_.publish(motor_data);
    // std_msgs::String lidar_data;
    // lidar_data.data = "Lidar Data> "+ lidar_data_;
    // str_robot_log_pub_.publish(lidar_data);
}

bool RobotInfo::IsTopicAlive(const ros::Time& last_msg_time)
{
    return (ros::Time::now() - last_msg_time).toSec() < TIMEOUT_DURATION;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_robot_info");

    RobotInfo robot_info;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    // ros::spin();

    return 0;
}