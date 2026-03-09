#include "nc_auto_repeat_driver.hpp"

using namespace std;
namespace NaviFra {
RepeatDriver::RepeatDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
    , b_is_running_(true)
    , last_log_time_(ros::Time::now())

{
    Initialize();

    th_ = boost::thread(&RepeatDriver::DoThreadProcess, this);
}

RepeatDriver::~RepeatDriver()
{
    b_is_running_ = false;
}

void RepeatDriver::Initialize()
{
    robot_status_ = RobotStatus();
    qr_error_ = QrError();
    repeat_info_ = RepeatTestInfo();

    DateTime();
    RegistTalker();
    RegistListener();
}

void RepeatDriver::RegistTalker()
{
    cmd_goal_pub_ = nh_.advertise<core_msgs::Goal>("/navifra/goal_id", 1);
    init_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    log_pub_ = nh_.advertise<std_msgs::String>("/repeat_test_log", 1);
    navi_cmd_pub_ = nh_.advertise<std_msgs::String>("/navifra/cmd", 1);

    b_talker_init_ = true;
}

void RepeatDriver::RegistListener()
{
    navi_status_sub_ = nh_.subscribe("/navifra/info", 1, &RepeatDriver::NaviStatusCallback, this);
    qr_offset_sub_ = nh_.subscribe("qr_code_offset", 1, &RepeatDriver::QrOffsetCallback, this);
    repeat_test_sub_ = nh_.subscribe("/repeat_test", 1, &RepeatDriver::RepeatCallback, this);
    repeat_test_sub_cmd_ = nh_.subscribe("/repeat_test/cmd", 1, &RepeatDriver::RepeatCmdCallback, this);
    node_link_sub_ = nh_.subscribe("/map/nodelink", 1, &RepeatDriver::NodeLinkCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &RepeatDriver::OdomCallback, this);
    cmd_sub_ = nh_.subscribe("/navifra/cmd", 1, &RepeatDriver::CmdCallback, this);

    b_listener_init_ = true;
}

void RepeatDriver::DateTime()
{
    std::time_t now = ros::Time::now().toSec();
    std::tm* local_time = std::localtime(&now);

    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y_%m%d_%H:%M:%S");
    s_curr_time_ = oss.str();
    NLOG(info) << "Current Time: " << s_curr_time_;
}

void RepeatDriver::NaviStatusCallback(const core_msgs::NavicoreStatus::ConstPtr& msg)
{
    switch (msg->n_status) {
        case core_msgs::NavicoreStatus::READY:
            robot_status_.s_robot_status_ = "idle";
            break;
        case core_msgs::NavicoreStatus::RUNNING:
            robot_status_.s_robot_status_ = "running";
            break;
        case core_msgs::NavicoreStatus::PAUSED_BY_STATUS:
        case core_msgs::NavicoreStatus::PAUSED_BY_USER:
        case core_msgs::NavicoreStatus::PAUSED_BY_OBS:
            robot_status_.s_robot_status_ = "paused";
            break;
    }

    robot_status_.s_curr_node_ = msg->s_current_node;
    robot_status_.s_next_node_ = msg->s_next_node;
    robot_status_.s_goal_node_ = msg->s_goal_node;

    robot_status_.f_x_error_m_ = std::round(msg->f_error_pos_x_m * 1000) / 1000;
    robot_status_.f_y_error_m_ = std::round(msg->f_error_pos_y_m * 1000) / 1000;
    robot_status_.f_deg_error_ = std::round(msg->f_error_pos_deg * 1000) / 1000;
}

void RepeatDriver::QrOffsetCallback(const core_msgs::PGVPoseList::ConstPtr& msg)
{
    std::string s_tag_tmp_id;

    try {
        double rounded_value = std::round(std::stod(msg->tag_id) * 100.0) / 100.0;

        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2) << rounded_value;
        s_tag_tmp_id = stream.str();
    }
    catch (const std::exception& e) {
        s_tag_tmp_id = msg->tag_id;
    }

    if (msg->tag_id != "null") {
        if (qr_time_list_.empty()) {
            qr_time_list_.push_back(std::make_pair(s_tag_tmp_id, ros::Time::now().toSec()));
        }
        else {
            if (qr_time_list_.back().first == s_tag_tmp_id) {
                qr_time_list_.back().second = ros::Time::now().toSec();
            }
            else if (qr_time_list_.back().first != s_tag_tmp_id) {
                qr_time_list_.push_back(std::make_pair(s_tag_tmp_id, ros::Time::now().toSec()));
            }
            if (qr_time_list_.size() > 3) {
                qr_time_list_.erase(qr_time_list_.begin());
            }
        }
    }
    qr_error_.s_qr_tag_ = s_tag_tmp_id;
    qr_error_.f_qr_x_error_m_ = std::round(msg->pgv_poses[1].x_m / 1000) * 1000;
    qr_error_.f_qr_y_error_m_ = std::round(msg->pgv_poses[1].y_m / 1000) * 1000;
    qr_error_.f_qr_deg_error_ = std::round((msg->pgv_poses[1].angle_rad / M_PI * 180) * 1000) / 1000;
}

std::string RepeatDriver::SetPrecision(float value, int len)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(len) << value;
    return stream.str();
}

void RepeatDriver::RepeatCallback(const core_msgs::RepeatTestMsg::ConstPtr& msg)
{
    ResetData();

    try {
        for (auto id : msg->recordNodeIdArray) {
            node_list_.push_back(node_link_[id]);
        }
        node_list_.push_back("q");
        for (auto id : msg->repeatDrivingNodeIdArray) {
            save_node_list_.push_back(node_link_[id]);
        }

        repeat_info_.n_repeat_cnt_ = msg->numberOfIterations;
        repeat_info_.f_stop_sec_ = msg->stopSeconds;
        robot_status_.s_cmd_ = "running";

        std_msgs::String cmd_msg;
        cmd_msg.data = "resume";
        navi_cmd_pub_.publish(cmd_msg);
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error in RepeatCallback: %s", e.what());
    }
}

void RepeatDriver::RepeatCmdCallback(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String cmd_msg;
    if (msg->data == "resume") {
        robot_status_.s_cmd_ = "running";
        cmd_msg.data = "resume";
        navi_cmd_pub_.publish(cmd_msg);
    }
    else if (msg->data == "pause") {
        robot_status_.s_cmd_ = "idle";
        cmd_msg.data = "pause";
        navi_cmd_pub_.publish(cmd_msg);
    }
    else if (msg->data == "stop") {
        robot_status_.s_cmd_ = "idle";
        cmd_msg.data = "cancel";
        navi_cmd_pub_.publish(cmd_msg);
        ResetData();
    }
}

void RepeatDriver::CmdCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "cancel") {
        ResetData();
    }
}

void RepeatDriver::NodeLinkCallback(const core_msgs::JsonList::ConstPtr& msg)
{
    for (const auto& node : msg->nodes) {
        node_link_[node.id] = node.name;
        NLOG(info) << node.name;
    }
}

void RepeatDriver::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (fabs(msg->twist.twist.linear.x) > 1.75) {
        d_decel_time_ = ros::Time::now().toSec();
    }
    robot_status_.f_robot_speed_ = fabs(msg->twist.twist.linear.x);
}

void RepeatDriver::SaveError()
{
    if (std::find(save_node_list_.begin(), save_node_list_.end(), robot_status_.s_curr_node_) == save_node_list_.end()) {
        return;  // 현재 노드가 저장할 목록에 없으면 리턴
    }

    NLOG(warning) << " Saving Error";

    // 홈 디렉토리에 `navifra_solution/navicore/configs` 폴더 경로 지정
    std::string home_dir = std::getenv("HOME");
    std::string save_dir = home_dir + "/navifra_solution/navicore/configs";
    std::string file_path = save_dir + "/node_repeat_test.txt";

    std::ifstream filecheck(file_path);
    bool file_exist = filecheck.good();
    filecheck.close();

    std::ofstream file(file_path, std::ios::app);

    if (!file) {
        NLOG(warning) << "Failed to open error log file : " << file_path;
    }

    if (!file_exist) {
        file << "time,current_repeat,total_repeat,node,x_error,y_error,deg_error,decel_time,qr_time\n";
    }

    // 현재 시간 가져오기
    std::time_t now = std::time(nullptr);
    char time_buffer[20];
    std::strftime(time_buffer, sizeof(time_buffer), "%Y_%m%d_%H:%M:%S", std::localtime(&now));

    // 오류 데이터 저장할 문자열 생성
    std::string error_data = std::string(time_buffer) + "," + std::to_string(repeat_info_.n_curr_repeat_) + "," +
        std::to_string(repeat_info_.n_repeat_cnt_) + "," + robot_status_.s_curr_node_ + "," + SetPrecision(robot_status_.f_x_error_m_, 3) +
        "," + SetPrecision(robot_status_.f_y_error_m_, 3) + "," + SetPrecision(robot_status_.f_deg_error_, 3) + "," +
        SetPrecision(d_decel_time_result_, 2) + "," + SetPrecision(d_decel_qr_result_, 2) + "," + qr_error_.s_qr_tag_ + "\n";

    file << error_data;
    file.close();

    // ROS 로그에도 저장된 내용 출력
    NLOG(warning) << "Saved error data : " << error_data;

    std_msgs::String log_msg;
    log_msg.data = error_data;
    log_pub_.publish(log_msg);
}

void RepeatDriver::ResetData()
{
    std::lock_guard<std::mutex> lock(status_mtx_);
    b_start_once_ = false;
    node_list_.clear();
    save_node_list_.clear();
    repeat_info_.n_repeat_cnt_ = 0;
    repeat_info_.f_stop_sec_ = 0;
    repeat_info_.n_curr_repeat_ = 0;
    repeat_info_.n_target_idx_ = 0;
}
void RepeatDriver::excuteTask(std::string target_node)
{
    static ros::ServiceClient client = nh_.serviceClient<core_msgs::CommonString>("/nc_task_manager_srv/task_add");

    std::string json_template = R"({
        "action": "add_task",
        "data": [
            {
                "type": "move",
                "uuid": "4d622886-d67a-4a79-955c-d87d469f2bc9",
                "target_node": "",
                "path_nodes": [],
                "vec_move_data": []
            }
        ],
        "uuid": "76f6ce59-9bc6-467e-9e13-e69f149b3c8f"
    })";

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(json_template);
    Poco::JSON::Object::Ptr jsonObject = result.extract<Poco::JSON::Object::Ptr>();
    Poco::JSON::Array::Ptr dataArray = jsonObject->getArray("data");
    Poco::JSON::Object::Ptr dataObject = dataArray->getObject(0);

    std::string goal_name = target_node;
    dataObject->set("target_node", goal_name);

    std::ostringstream oss;
    jsonObject->stringify(oss);
    std::string modified_json = oss.str();

    core_msgs::CommonString srv;
    srv.request.data = modified_json;  // 수정된 JSON 문자열 사용

    if (!client.call(srv)) {
        NLOG(error) << "Service call failed";
    }
    else {
        std::lock_guard<std::mutex> lock(status_mtx_);
        robot_status_.s_robot_status_ = "running";
    }
}

void RepeatDriver::DoThreadProcess()
{
    // core_msgs::Goal goal;
    int n_tmp_cnt = 0;
    while (ros::ok()) {
        if (node_list_.empty() || robot_status_.s_cmd_ == "idle") {
            ros::Duration(0.1).sleep();
            n_tmp_cnt++;
            if (n_tmp_cnt > 100) {
                n_tmp_cnt = 0;
            }
            continue;
        }

        // ROS가 동작하지 않는다면 스테이트머신도 동작하지 않는다.
        if (!b_talker_init_ || !b_listener_init_) {
            ros::Duration(0.1).sleep();
            continue;
        }

        std::string s_current_status;
        std::string s_current_node;
        float f_robot_speed;

        {
            std::lock_guard<std::mutex> lock(status_mtx_);
            s_current_status = robot_status_.s_robot_status_;
            s_current_node = robot_status_.s_curr_node_;
            f_robot_speed = robot_status_.f_robot_speed_;
        }

        std::vector<std::string> target_list = node_list_;

        if (!b_start_once_) {
            excuteTask(target_list[0]);
            // goal.name = target_list[0];
            // goal.type = "Goal";
            // cmd_goal_pub_.publish(goal);
            NLOG(info) << target_list[0] << " publish!!!";

            b_start_once_ = true;

            {
                std::lock_guard<std::mutex> lock(status_mtx_);
                s_current_status = robot_status_.s_robot_status_;
            }
        }
        //로봇상태가 대기중 이면서 타겟노드와 현재노드가 일치할때
        if (s_current_status == "idle" && target_list[repeat_info_.n_target_idx_] == s_current_node && f_robot_speed < 0.1) {
            if (ros::Time::now() - last_log_time_ >= ros::Duration(2.0)) {
                NLOG(info) << "status : " << s_current_status << " current node : " << s_current_node
                           << " target node : " << target_list[repeat_info_.n_target_idx_] << " progress : " << repeat_info_.n_curr_repeat_
                           << "/" << repeat_info_.n_repeat_cnt_;
                last_log_time_ = ros::Time::now();
            }

            d_decel_qr_result_ = 0.0;
            double d_current_time = ros::Time::now().toSec();  // 현재 시간 (초 단위)

            if (qr_time_list_.size() == 3) {
                d_decel_qr_result_ = std::round((d_current_time - qr_time_list_[0].second) * 100) / 100.0;
            }
            d_decel_time_result_ = std::round((d_current_time - d_decel_time_) * 100) / 100.0;

            n_error_check_cnt_ = 0;

            ros::Duration(repeat_info_.f_stop_sec_).sleep();
            if (save_node_list_.size() != 1) {
                SaveError();
            }
            ros::Duration(1).sleep();

            repeat_info_.n_target_idx_++;

            //마지막 노드 도착 시 (target_list[idx]가 'q'일 때)
            if (target_list[repeat_info_.n_target_idx_] == "q") {
                repeat_info_.n_curr_repeat_++;
                repeat_info_.n_target_idx_ = 0;
            }

            if (repeat_info_.n_curr_repeat_ == repeat_info_.n_repeat_cnt_) {
                robot_status_.s_cmd_ = "idle";
                ResetData();
                ros::Duration(2).sleep();
                continue;
            }
            excuteTask(target_list[repeat_info_.n_target_idx_]);

            {
                std::lock_guard<std::mutex> lock(status_mtx_);
                s_current_status = robot_status_.s_robot_status_;
            }

            // goal.name = target_list[repeat_info_.n_target_idx_];
            // goal.type = "Goal";
            // cmd_goal_pub_.publish(goal);
            NLOG(info) << target_list[repeat_info_.n_target_idx_] << " publish!!!";
        }

        if (ros::Time::now() - last_log_time_ >= ros::Duration(2.0)) {
            NLOG(info) << "status : " << s_current_status << " current node : " << s_current_node
                       << " target node : " << target_list[repeat_info_.n_target_idx_] << " progress : " << repeat_info_.n_curr_repeat_
                       << "/" << repeat_info_.n_repeat_cnt_;
            last_log_time_ = ros::Time::now();
        }

        //대기중이면서 타겟 노드와 현재 노드가 불일치(노드 이동 명령 스킵 될 경우)
        if (s_current_status == "idle" && target_list[repeat_info_.n_target_idx_] != s_current_node && f_robot_speed < 0.1) {
            n_error_check_cnt_++;
        }

        if (n_error_check_cnt_ > 50) {
            NLOG(info) << "goal skip";
            ros::Duration(repeat_info_.f_stop_sec_).sleep();
            NLOG(info) << "republish goal";
            ros::Duration(1).sleep();
            excuteTask(target_list[repeat_info_.n_target_idx_]);

            {
                std::lock_guard<std::mutex> lock(status_mtx_);
                s_current_status = robot_status_.s_robot_status_;
            }

            n_error_check_cnt_ = 0;

            // goal.name = target_list[repeat_info_.n_target_idx_];
            // goal.type = "Goal";
            // cmd_goal_pub_.publish(goal);
            // NLOG(info) << target_list[repeat_info_.n_target_idx_] << " publish!!!";
        }

        ros::Duration(0.1).sleep();
    }
}

}  // namespace NaviFra
int main(int argc, char** argv)
{
    ros::init(argc, argv, "repeat_driver");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    NaviFra::RepeatDriver repeatdriver(nh, nhp);
    ros::spin();
    return 0;
}
