#include "nc_total_tester.hpp"

#include "core/util/logger.hpp"

#include <jsoncpp/json/json.h>

TotalTester::TotalTester()
    : nh("~")
{
    NLOG(info) << "=== Program Started at ";

    package_path_ = ros::package::getPath("nc_total_tester") + "/launch";
    json_publisher_ = nh_.advertise<core_msgs::MapJson>("map/all", 1, true);
    nodes_publisher_ = nh_.advertise<core_msgs::JsonList>("map/nodelink", 1, true);
    map_db_pub_ = nh_.advertise<core_msgs::MapDB>("map/db", 1, true);
    param_pub_ = nh_.advertise<std_msgs::String>("navifra/param_update", 1, true);
    nav_cmd_pub_ = nh_.advertise<std_msgs::String>("/navifra/cmd", 1, true);
    task_cmd_pub_ = nh_.advertise<std_msgs::String>("nc_task_manager/task_cmd", 1, true);

    pc_log_sub_ = nh_.subscribe("navifra/log", 10, &TotalTester::RecvLog, this);

    // param_pub_.publish(msg);
    LoadMapJson();
    std::this_thread::sleep_for(std::chrono::seconds(20));
    // Constructor: 시나리오 실행
    th1_ = boost::thread(boost::bind(&TotalTester::StratTest, this));
}

TotalTester::~TotalTester()
{
    b_terminate_ = true;
    LOG_INFO("thread wait");
    if (th1_.joinable()) {
        th1_.join();
    }
    LOG_INFO("destructor");
}

void TotalTester::StratTest()
{
    Json::Value scenarios;
    if (!loadScenarios(package_path_ + "/scenarios.json", scenarios)) {
        return;
    }

    int n_scenario_start = 1;
    ros::param::param<int>("/nc_total_tester/scenario", n_scenario_start, 1);

    int n_scenario_loop = 1;
    ros::param::param<int>("/nc_total_tester/n_scenario_loop", n_scenario_loop, 1);
    int n_kinematics_type = -1;
    ros::param::param<int>("/nc_total_tester/kinematics", n_kinematics_type, -1);
    n_kinematics_ = 0;
    NLOG(info) << "n_scenario_start " << n_scenario_start;
    n_scenario_start--;

    for (int loop = 0; loop < n_scenario_loop; loop++) {
        NLOG(info) << " Scenario Loop : " << loop + 1 << "/" << n_scenario_loop;
        n_kinematics_ = 0;
        n_now_loop_ = loop;

        for (int index = n_kinematics_; index < 4; index++) {
            if(n_kinematics_type != -1)
            {
                n_kinematics_ = n_kinematics_type;
            }
            ros::param::set("motion_base/n_kinematics_type", n_kinematics_);
            std_msgs::String msg;
            param_pub_.publish(msg);
            if (n_kinematics_ == 0) {
                s_kinematics_type_ = "DD";
            }
            else if (n_kinematics_ == 1) {
                s_kinematics_type_ = "QD";
            }
            else if (n_kinematics_ == 2) {
                s_kinematics_type_ = "SD";
            }
            else if (n_kinematics_ == 3) {
                s_kinematics_type_ = "OD";
            }
            std::this_thread::sleep_for(std::chrono::seconds(5));

            for (int i = n_scenario_start; i < scenarios.size(); i++) {
                if (b_terminate_) {
                    break;  // ROS가 종료되면 for문 중간에 탈출
                }
                executeScenario(scenarios[i]);
            }
            n_kinematics_++;
        }
    }

    NLOG(info) << " All scenarios executed \n";

    NLOG(info) << "total scenario count: " << n_total_scenario_count_;
    NLOG(info) << "failed scenario count: " << vec_failed_scenario_name_.size();

    for (auto& name : vec_failed_scenario_name_) {
        NLOG(info) << name;
    }
}

void TotalTester::RecvLog(const std_msgs::String::ConstPtr& msg)
{
    // NLOG(info) << msg->data;
    // NLOG(info) << "msg->data[0] " << msg->data[0];
    static int n_count = 0;
    if (msg->data.size() > 0 && msg->data[0] == 'P') {
        n_count++;

        if (n_count > 50) {
            n_count = 0;
            LOG_INFO("%s", msg->data.c_str());
        }
    }
}

bool TotalTester::loadScenarios(const std::string& filename, Json::Value& scenarios)
{
    std::ifstream file(filename);
    if (!file.is_open())
        return false;

    Json::Reader reader;
    return reader.parse(file, scenarios);
}

void TotalTester::executeScenario(const Json::Value& scenario)
{
    static int n_total_scenario_count_ = 0;
    n_total_scenario_count_++;
    
    if (vec_failed_scenario_name_.size() > 0) {
        NLOG(error) << "TEST FAIL";
        for (auto& name : vec_failed_scenario_name_) {
            NLOG(info) << name;
        }
        exit(1);
    }
    NLOG(info) << "Executing scenario: " << scenario["name"].asCString() << "/" << s_kinematics_type_+ "/loop:" + std::to_string(n_now_loop_)+" /total scenario :"+std::to_string(n_total_scenario_count_);

    std_msgs::String cmd_msg;
    cmd_msg.data = "cancel";
    task_cmd_pub_.publish(cmd_msg);
    ros::Duration delay(0.1);  // 시간차 설정 (필요 시 조정)
    delay.sleep();

    setInitialPose(scenario["initial_pose"]);
    executeTask(scenario["task"]);

    if (!monitorDuringRun(scenario)) {
        NLOG(error) << "failed during real-time monitoring " << scenario["name"].asCString() << "/" << s_kinematics_type_;
        std::string s_error = std::string(scenario["name"].asCString()) + "/loop:" + std::to_string(n_now_loop_);
        vec_failed_scenario_name_.emplace_back(s_error);
        return;
    }

    if (!monitorAndEvaluate(scenario["expected_result"])) {
        NLOG(error) << "failed final evaluation " << scenario["name"].asCString() << "/" << s_kinematics_type_;
        std::string s_error = std::string(scenario["name"].asCString()) + "/loop:" + std::to_string(n_now_loop_);
        vec_failed_scenario_name_.emplace_back(s_error);
        return;
    }
    NLOG(info) << "passed successfully: " << scenario["name"] << "/" << s_kinematics_type_;
    NLOG(info) << "===============================================";
}

std::string TotalTester::getCurrentTime()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}

void TotalTester::setInitialPose(const Json::Value& pose)
{
    static ros::Publisher init_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = ros::Time::now();
    init_pose.header.frame_id = "map";
    init_pose.pose.pose.position.x = pose["x"].asFloat();
    init_pose.pose.pose.position.y = pose["y"].asFloat();
    float yaw = pose["theta"].asFloat() / 180 * 3.141592;
    init_pose.pose.pose.orientation.z = sin(yaw / 2.0);
    init_pose.pose.pose.orientation.w = cos(yaw / 2.0);

    ros::Duration(1.0).sleep();  // 안정성 확보
    init_pub.publish(init_pose);
    ros::Duration(2.0).sleep();  // 초기화 대기
}

void TotalTester::executeTask(const Json::Value& task)
{
    static ros::ServiceClient client = nh.serviceClient<core_msgs::CommonString>("/nc_task_manager_srv/task_add");

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

    // JSON 파싱
    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(json_template);
    Poco::JSON::Object::Ptr jsonObject = result.extract<Poco::JSON::Object::Ptr>();
    Poco::JSON::Array::Ptr dataArray = jsonObject->getArray("data");
    Poco::JSON::Object::Ptr dataObject = dataArray->getObject(0);
    // task 변수에서 goal_name 가져오기
    std::string goal_name;
    if (task.isMember("goal_name")) {
        goal_name = task["goal_name"].asString();
        // target_node 수정
        dataObject->set("target_node", goal_name);  // goal_name으로 변경
    }
    else if (task.isMember("path_nodes")) {
        Poco::JSON::Array::Ptr nodesArray = new Poco::JSON::Array();
        for (const auto& node : task["path_nodes"]) {
            if (node.isString()) {
                nodesArray->add(node.asString());
            }
        }
        dataObject->set("path_nodes", nodesArray);
    }
    else if (task.isMember("vec_move_data")) {
        Poco::JSON::Array::Ptr moveDataArray = new Poco::JSON::Array();
        for (const auto& moveData : task["vec_move_data"]) {
            Poco::JSON::Object::Ptr moveObj = new Poco::JSON::Object();
            moveObj->set("s_name", moveData["s_name"].asString());
            moveObj->set("n_drive_type", moveData["n_drive_type"].asInt());
            moveObj->set("f_speed_ms", moveData["f_speed_ms"].asFloat());
            moveObj->set("f_x_m", moveData["f_x_m"].asFloat());
            moveObj->set("f_y_m", moveData["f_y_m"].asFloat());
            moveObj->set("f_angle_deg", moveData["f_angle_deg"].asFloat());
            if (moveData.isMember("b_start_quick"))
                moveObj->set("b_start_quick", moveData["b_start_quick"].asBool());
            if (moveData.isMember("b_stop_quick"))
                moveObj->set("b_stop_quick", moveData["b_stop_quick"].asBool());
            if (moveData.isMember("f_curve_radius"))
                moveObj->set("f_curve_radius", moveData["f_curve_radius"].asFloat());
            if (moveData.isMember("b_diagonal_align_skip"))
                moveObj->set("b_diagonal_align_skip", moveData["b_diagonal_align_skip"].asBool());
            if (moveData.isMember("f_diagonal_heading_bias"))
                moveObj->set("f_diagonal_heading_bias", moveData["f_diagonal_heading_bias"].asFloat());
            if (moveData.isMember("n_avoid_type"))
                moveObj->set("n_avoid_type", moveData["n_avoid_type"].asInt());
            if (moveData.isMember("f_avoid_lanewidth"))
                moveObj->set("f_avoid_lanewidth", moveData["f_avoid_lanewidth"].asFloat());  
            moveDataArray->add(moveObj);
        }
        dataObject->set("vec_move_data", moveDataArray);
    }

    // 수정된 JSON을 문자열로 변환
    std::ostringstream oss;
    jsonObject->stringify(oss);
    std::string modified_json = oss.str();

    core_msgs::CommonString srv;
    srv.request.data = modified_json;  // 수정된 JSON 문자열 사용

    if (!client.call(srv)) {
        LOG_ERROR("Service call failed");
        vec_failed_scenario_name_.emplace_back("fail");
    }

    // 이동 중 두 번째 목표 (Node 2) 추가 여부 확인
    if (task.isMember("interrupt_goal")) {
        std::string interrupt_goal = task["interrupt_goal"].asString();  // Node 2
        // 일정 시간 대기 (예: 3초 후 Node 2 전송)
        ros::Duration delay(3.0);  // 시간차 설정 (필요 시 조정)
        delay.sleep();

        // 두 번째 목표를 위한 JSON 수정
        result = parser.parse(json_template);  // 새 JSON 객체 생성
        jsonObject = result.extract<Poco::JSON::Object::Ptr>();
        dataArray = jsonObject->getArray("data");
        dataObject = dataArray->getObject(0);
        dataObject->set("target_node", interrupt_goal);

        oss.str("");  // 스트림 초기화
        jsonObject->stringify(oss);
        modified_json = oss.str();

        srv.request.data = modified_json;
        if (client.call(srv)) {
            LOG_INFO("Interrupt task (Node 2) sent: %s", interrupt_goal.c_str());
        }
        else {
            LOG_ERROR("Failed to call service for Node 2");
        }
    }
    if (task.isMember("stop_test")) {
        ros::Duration delay(3.0);  // 시간차 설정 (필요 시 조정)
        delay.sleep();
        std_msgs::String cmd_msg;
        cmd_msg.data = "cancel";
        // nav_cmd_pub_.publish(cmd_msg);
        task_cmd_pub_.publish(cmd_msg);
    }
    if (task.isMember("stop_test2")) {
        if (!b_stop_test_) {
            b_stop_test_ = true;
            ros::Duration delay(3.0);  // 시간차 설정 (필요 시 조정)
            delay.sleep();
            std_msgs::String cmd_msg;
            cmd_msg.data = "cancel";
            // nav_cmd_pub_.publish(cmd_msg);
            task_cmd_pub_.publish(cmd_msg);
            delay.sleep();
            executeTask(task);
        }
        else {
            b_stop_test_ = false;
        }
    }
    if (task.isMember("pause_test")) {
        ros::Duration delay(5.0);  // 시간차 설정 (필요 시 조정)
        delay.sleep();
        std_msgs::String cmd_msg;
        cmd_msg.data = "pause";
        // nav_cmd_pub_.publish(cmd_msg);
        task_cmd_pub_.publish(cmd_msg);
    }
}

bool TotalTester::monitorDuringRun(const Json::Value& scenario)
{
    ros::Rate rate(10);  // 10Hz 모니터링
    float timeout = scenario.get("timeout", 100.0).asFloat();
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        auto nav_status = ros::topic::waitForMessage<core_msgs::NavicoreStatus>("/navifra/info", ros::Duration(0.5));
        if (!nav_status) {
            LOG_WARNING("nav_statusetry data timeout");
            continue;
        }
        float current_speed = std::hypot(nav_status->f_robot_current_linear_vel_x, nav_status->f_robot_current_linear_vel_y);

        // speed check
        if (scenario["expected_result"].isMember("speed_check")) {
            bool b_max_speed_check = scenario["expected_result"]["speed_check"].asBool();
            if (b_max_speed_check) {
                float max_speed_ms = scenario["expected_result"]["max_speed_ms"].asFloat();
                if (current_speed - max_speed_ms > 0.2) {
                    LOG_ERROR("Abnormal speed detected. Expected: %.2f, Actual: %.2f", max_speed_ms, current_speed);
                    std::string s_error = std::string(scenario["name"].asCString()) + "/loop:" + std::to_string(n_now_loop_);
                    vec_failed_scenario_name_.emplace_back(s_error);
                    return false;
                }
            }
        }

        if (scenario["expected_result"].isMember("lateral_check")) {
            bool b_lateral_check = scenario["expected_result"]["lateral_check"].asBool();
            if (b_lateral_check) {
                // Lateral check
                float lateral_error = nav_status->f_path_error_dist_m;
                float lateral_tolerance = scenario["expected_result"]["max_lateral_error"].asFloat();
                if (lateral_error > lateral_tolerance && (ros::Time::now() - start_time).toSec() > 3) {
                    LOG_ERROR("Lateral error too large. Expected: %.2f, Actual: %.2f", lateral_tolerance, lateral_error);
                    std::string s_error = std::string(scenario["name"].asCString()) + "/loop:" + std::to_string(n_now_loop_);
                    vec_failed_scenario_name_.emplace_back(s_error);
                    return false;
                }
            }
        }
        if (scenario["expected_result"].isMember("time_check")) {
            bool b_time_check = scenario["expected_result"]["time_check"].asBool();
            if (b_time_check) {
                // Time check
                float elapsed_time = (ros::Time::now() - start_time).toSec();
                float expected_time = scenario["expected_result"]["expected_time"].asFloat();
                if (elapsed_time > expected_time) {
                    LOG_ERROR("Time exceeded. Expected: %.2f, Actual: %.2f", expected_time, elapsed_time);
                    std::string s_error = std::string(scenario["name"].asCString()) + "/loop:" + std::to_string(n_now_loop_);
                    vec_failed_scenario_name_.emplace_back(s_error);
                    return false;
                }
            }
        }

        static bool b_reach_speed = false;
        if (scenario["expected_result"].isMember("link_speed_check") && scenario["expected_result"]["link_speed_check"].asBool()) {
            bool b_link_speed_check = scenario["expected_result"]["link_speed_check"].asBool();
            if (b_link_speed_check) {
                // Link speed check
                float link_speed = nav_status->f_max_linear_vel_of_path;
                float link_speed_tolerance = scenario["expected_result"]["link_speed_tolerance"].asFloat();
                if (fabs(link_speed - current_speed) < link_speed_tolerance) {
                    b_reach_speed = true;
                }
            }
            else {
                b_reach_speed = true;
            }
        }
        else {
            b_reach_speed = true;
        }

        if (scenario["expected_result"].isMember("obstacle_stop_check")) {
            bool b_obs_check = scenario["expected_result"]["obstacle_stop_check"].asBool();
            if (b_obs_check) {
                // Obstacle check
                if (nav_status->n_status == 112) {
                    b_reach_speed = true;
                    if (b_reach_speed) {
                        ros::Duration(1.0).sleep();  // 1초 동안 sleep
                        std_msgs::String cmd_msg;
                        cmd_msg.data = "cancel";
                        // nav_cmd_pub_.publish(cmd_msg);
                        task_cmd_pub_.publish(cmd_msg);
                    }
                }
            }
        }

        if (scenario["expected_result"].isMember("pause_check")) {
            bool b_pause_check = scenario["expected_result"]["pause_check"].asBool();
            if (b_pause_check) {
                // Pause check
                if (nav_status->n_status == 111) {
                    b_reach_speed = true;
                    if (b_reach_speed) {
                        ros::Duration(2.0).sleep();  // 2초 동안 sleep
                        std_msgs::String cmd_msg;
                        cmd_msg.data = "resume";
                        task_cmd_pub_.publish(cmd_msg);
                    }
                }
            }
        }

        // 주행 완료 확인
        if (nav_status && nav_status->s_status == "idle" && (ros::Time::now() - start_time).toSec() > 1 && !b_stop_test_) {
            LOG_INFO("Robot arrived at destination. sec %.3f", (ros::Time::now() - start_time).toSec());
            if (b_reach_speed)
                return true;
            else {
                LOG_ERROR("Link speed not reached.");
                std::string s_error = std::string(scenario["name"].asCString()) + "/loop:" + std::to_string(n_now_loop_);
                vec_failed_scenario_name_.emplace_back(s_error);
                return false;
            }
        }

        if ((ros::Time::now() - start_time).toSec() > timeout) {
            LOG_ERROR("Scenario [%s] timeout.", scenario["name"].asCString());
            std::string s_error = std::string(scenario["name"].asCString()) + "/loop:" + std::to_string(n_now_loop_);
            vec_failed_scenario_name_.emplace_back(s_error);
            return false;
        }

        rate.sleep();
    }
    return false;
}

bool TotalTester::monitorAndEvaluate(const Json::Value& scenario)
{
    auto odom_msg = ros::topic::waitForMessage<core_msgs::NavicoreStatus>("/navifra/info", ros::Duration(5.0));
    if (!odom_msg) {
        LOG_ERROR("No odometry data received.");
        return false;
    }
    bool b_check = scenario["final_pose_check"].asBool();
    float tol_pos = scenario["tolerance"]["position"].asFloat();
    float tol_ang = scenario["tolerance"]["angle"].asFloat();

    float dx = odom_msg->f_error_pos_x_m;
    float dy = odom_msg->f_error_pos_y_m;
    float da = odom_msg->f_error_pos_deg;

    // LOG_INFO("Robot Position: %.2f, %.2f, %.2f", dx, dy, da);
    float pos_error = sqrt(dx * dx + dy * dy);
    float angle_error = fabs(da);

    LOG_INFO("Final Position Error: %.2f angle %.2f", pos_error, angle_error);

    if ((pos_error > tol_pos || angle_error > tol_ang) && b_check) {
        LOG_WARNING("Final position out of tolerance.");
        return false;
    }

    return true;
}

void TotalTester::LoadMapJson()
{
    NaviFra::JsonStream o_json_stream;
    std::string s_map_path = package_path_ + "/map.json";
    o_json_stream.ReadJsonFromFile(s_map_path);
    rapidjson::Document& json_doc = o_json_stream.GetJsonFile();

    if (json_doc.HasMember("id")) {
        std_msgs::String cur_map_name_msg;
        cur_map_name_msg.data = json_doc["id"].GetString();
    }
    else {
        LOG_ERROR("No map.json id");
        return;
    }
    core_msgs::MapJson o_map_json;
    core_msgs::MapDB o_map_db;
    core_msgs::JsonList o_map_nodelink;

    o_map_nodelink = GetNodeLinkData(json_doc);
    nodes_publisher_.publish(o_map_nodelink);

    if (json_doc.HasMember("occupied_space")) {
        // Extract map size and origin from JSON
        const float f_map_resolution_m = json_doc["resolution"].GetFloat();
        const float f_map_size_x_m = json_doc["width"].GetInt() * f_map_resolution_m;
        const float f_map_size_y_m = json_doc["height"].GetInt() * f_map_resolution_m;
        const float f_map_origin_x_m = -f_map_size_x_m / 2 + json_doc["offset_x"].GetFloat();
        const float f_map_origin_y_m = -f_map_size_y_m / 2 + json_doc["offset_y"].GetFloat();
        const float f_map_width = json_doc["width"].GetInt();
        const float f_map_height = json_doc["height"].GetInt();

        // Create map info message
        float f_map_origin_x = f_map_origin_x_m;
        float f_map_origin_y = f_map_origin_y_m;

        try {
            o_map_json.revision = int64_t(json_doc["revision"].GetFloat());
        }
        catch (...) {
            o_map_json.revision = 0;
        }
        float f_map_min_x = 1000;
        float f_map_min_y = 1000;
        float f_map_max_x = 0;
        float f_map_max_y = 0;

        // Extract occupied space data from JSON and add to pose array
        const auto& db_list = json_doc["occupied_space"];

        for (const auto& db_item : db_list.GetArray()) {
            geometry_msgs::Pose o_pos;
            o_pos.position.x = db_item[0].GetFloat();
            o_pos.position.y = db_item[1].GetFloat();

            o_map_db.map_db.poses.emplace_back(o_pos);
            if (f_map_min_x > o_pos.position.x)
                f_map_min_x = o_pos.position.x;
            if (f_map_min_y > o_pos.position.y)
                f_map_min_y = o_pos.position.y;
            if (f_map_max_x < o_pos.position.x)
                f_map_max_x = o_pos.position.x;
            if (f_map_max_y < o_pos.position.y)
                f_map_max_y = o_pos.position.y;
        }
        o_map_db.map_min_x = f_map_min_x;
        o_map_db.map_min_y = f_map_min_y;
        o_map_db.map_max_x = f_map_max_x;
        o_map_db.map_max_y = f_map_max_y;
        o_map_db.map_width = f_map_width;
        o_map_db.map_height = f_map_height;
        o_map_db.map_origin_x = f_map_origin_x;
        o_map_db.map_origin_y = f_map_origin_y;

        // Build and publish messages
        o_map_json.jsonlist = o_map_nodelink;
        o_map_json.map = o_map_db;
        map_db_pub_.publish(o_map_db);
        json_publisher_.publish(o_map_json);
    }
    else {
        LOG_WARNING("No map.json map_data or cmd only update nodelink");
        return;
    }
}

core_msgs::JsonList TotalTester::GetNodeLinkData(rapidjson::Document& doc)
{
    core_msgs::JsonList o_json_list;
    auto GetNodeType = [](const std::string& s_type) {
        if (s_type.compare("D40301") == 0) {
            return core_msgs::JsonNodeList::NONE;
        }
        else if (s_type.compare("D40302") == 0) {
            return core_msgs::JsonNodeList::POI;
        }
        else if (s_type.compare("D40303") == 0) {
            return core_msgs::JsonNodeList::SPIN_TURN;
        }
        else {
            LOG_WARNING("No Type %s", s_type.c_str());
            return core_msgs::JsonNodeList::NONE;
        }
    };

    auto GetPolyType = [](const std::string& s_type) {
        if (s_type.compare("D40201") == 0) {
            return core_msgs::JsonNodeList::POLY1;
        }
        else if (s_type.compare("D40202") == 0) {
            return core_msgs::JsonNodeList::POLY2;
        }
        else if (s_type.compare("D40203") == 0) {
            return core_msgs::JsonNodeList::POLY3;
        }
        else if (s_type.compare("D40204") == 0) {
            return core_msgs::JsonNodeList::POLY4;
        }
        else if (s_type.compare("D40205") == 0) {
            return core_msgs::JsonNodeList::POLY5;
        }
        else {
            LOG_WARNING("No Type %s", s_type.c_str());
            return core_msgs::JsonNodeList::POLY1;
        }
    };

    if (doc.HasMember("nodes")) {
        const Value& NODES = doc["nodes"];
        if (false == NODES.Empty()) {
            for (int i = 0; i < NODES.Size(); i++) {
                core_msgs::JsonNodeList o_node;
                o_node.id = NODES[i]["id"].GetString();
                o_node.name = NODES[i]["name"].GetString();
                LOG_DEBUG("id : %s, name : %s", o_node.id.c_str(), o_node.name.c_str());
                if (NODES[i].HasMember("position")) {
                    o_node.position.x = NODES[i]["position"]["x"].GetFloat();
                    o_node.position.y = NODES[i]["position"]["y"].GetFloat();
                    o_node.position.z = NODES[i]["position"]["z"].GetFloat();
                    LOG_DEBUG("position : %.lf %.lf %.lf", o_node.position.x, o_node.position.y, o_node.position.z);
                }
                if (NODES[i].HasMember("orientation")) {
                    o_node.orientation.w = NODES[i]["orientation"]["w"].GetFloat();
                    o_node.orientation.x = NODES[i]["orientation"]["x"].GetFloat();
                    o_node.orientation.y = NODES[i]["orientation"]["y"].GetFloat();
                    o_node.orientation.z = NODES[i]["orientation"]["z"].GetFloat();
                    LOG_DEBUG(
                        "orientation(w,x,y,z) : %.lf %.lf %.lf %.lf", o_node.orientation.w, o_node.orientation.x, o_node.orientation.y,
                        o_node.orientation.z);
                }
                if (NODES[i].HasMember("alarm")) {
                    o_node.alarm_flag = NODES[i]["alarm"]["flag"].GetBool();
                    o_node.alarm_mm = NODES[i]["alarm"]["mm"].GetFloat();
                    o_node.alarm_deg = NODES[i]["alarm"]["deg"].GetFloat();
                }
                try {
                    if (NODES[i].HasMember("align_type_cd")) {
                        o_node.type = GetNodeType(NODES[i]["align_type_cd"].GetString());
                    }
                    if (NODES[i].HasMember("obspolygon_cd")) {
                        o_node.polytype = GetPolyType(NODES[i]["obspolygon_cd"].GetString());
                    }
                }
                catch (std::runtime_error& e) {
                    LOG_ERROR("align type, obspolygon error");
                }

                try {
                    if (NODES[i].HasMember("extra")) {
                        if (NODES[i]["extra"].HasMember("barcode")) {
                            o_node.barcode = NODES[i]["extra"]["barcode"].GetFloat();
                            NLOG(debug) << "barcode " << o_node.barcode;
                        }
                        else {
                            o_node.barcode = 0;
                        }
                    }
                    else {
                        o_node.barcode = 0;
                    }
                }
                catch (std::runtime_error& e) {
                    LOG_ERROR("align type, obspolygon error");
                }

                o_json_list.nodes.push_back(o_node);
            }
        }
    }
    else {
        LOG_ERROR("json no has nodes!");
    }
    NLOG(info) << "Node number : " << o_json_list.nodes.size();

    auto GetLinkType = [](std::string s_type) {
        if (s_type.compare(core_msgs::JsonLinkList::LINE_CD) == 0) {
            return core_msgs::JsonLinkList::LINE;
        }
        else if (s_type.compare(core_msgs::JsonLinkList::ARC_CD) == 0) {
            return core_msgs::JsonLinkList::ARC;
        }
        else {
            LOG_WARNING("does not have %s link type ...", s_type.c_str());
            return core_msgs::JsonLinkList::NONE;
        }
    };

    auto GetLinkDriveDirType = [](std::string s_type) {
        if (s_type.compare("front") == 0) {
            return core_msgs::JsonLinkList::FRONT;
        }
        else if (s_type.compare("rear") == 0) {
            return core_msgs::JsonLinkList::REAR;
        }
        else if (s_type.compare("side") == 0) {
            return core_msgs::JsonLinkList::SIDE;
        }
        else if (s_type.compare("front_diagonal") == 0) {
            return core_msgs::JsonLinkList::FRONTDIAGONAL;
        }
        else if (s_type.compare("rear_diagonal") == 0) {
            return core_msgs::JsonLinkList::REARDIAGONAL;
        }
        else if (s_type.compare("front_dock") == 0) {
            return core_msgs::JsonLinkList::FRONTDOCK;
        }
        else if (s_type.compare("rear_dock") == 0) {
            return core_msgs::JsonLinkList::REARDOCK;
        }
        else {
            LOG_WARNING("does not have %s JsonLinkList type ...", s_type.c_str());
            return core_msgs::JsonLinkList::FRONT;
        }
    };
    if (doc.HasMember("links")) {
        const Value& LINKS = doc["links"];
        if (false == LINKS.Empty()) {
            for (int i = 0; i < LINKS.Size(); i++) {
                core_msgs::JsonLinkList o_link;
                o_link.id = LINKS[i]["id"].GetString();
                o_link.type = GetLinkType(LINKS[i]["link_type_cd"].GetString());
                if (LINKS[i].HasMember("connected")) {
                    o_link.from_link = LINKS[i]["connected"]["from"].GetString();
                    o_link.to_link = LINKS[i]["connected"]["to"].GetString();
                }
                if (LINKS[i].HasMember("from_link_drive")) {
                    o_link.from_driving_direction = GetLinkDriveDirType(LINKS[i]["from_link_drive"]["direction"].GetString());
                    o_link.from_driving_linear_speed = LINKS[i]["from_link_drive"]["linear_speed"].GetFloat();
                    NLOG(debug) << "o_link.from_driving_linear_speed " << o_link.from_driving_linear_speed;
                    if (fabs(o_link.from_driving_linear_speed) > 10) {
                        o_link.from_driving_linear_speed = std::stof(LINKS[i]["from_link_drive"]["linear_speed"].GetString());
                        NLOG(debug) << "o_link.from_driving_linear_speed2 " << o_link.from_driving_linear_speed;
                    }
                    o_link.from_driving_diagonal_curve = LINKS[i]["from_link_drive"]["diagonal_curve"].GetFloat();
                    o_link.from_driving_active = LINKS[i]["from_link_drive"]["active"].GetBool();
                    o_link.from_driving_start_smooth = LINKS[i]["from_link_drive"]["start_smooth"].GetBool();  // brain 추가 예정
                    o_link.from_driving_stop_smooth = LINKS[i]["from_link_drive"]["stop_smooth"].GetBool();  // brain 추가 예정
                    o_link.from_driving_avoidance_right = LINKS[i]["from_link_drive"]["avoidance_right"].GetBool();
                    o_link.from_driving_avoidance_left = LINKS[i]["from_link_drive"]["avoidance_left"].GetBool();
                    o_link.from_driving_avoidance_step = LINKS[i]["from_link_drive"]["avoidance_step"].GetInt();

                    if (LINKS[i]["from_link_drive"].HasMember("collision")) {
                        o_link.from_driving_collision_enable = LINKS[i]["from_link_drive"]["collision"]["enabled"].GetBool();
                        o_link.from_driving_collision_margin_front_m = LINKS[i]["from_link_drive"]["collision"]["front"].GetFloat();
                        o_link.from_driving_collision_margin_rear_m = LINKS[i]["from_link_drive"]["collision"]["rear"].GetFloat();
                        o_link.from_driving_collision_margin_left_m = LINKS[i]["from_link_drive"]["collision"]["left"].GetFloat();
                        o_link.from_driving_collision_margin_right_m = LINKS[i]["from_link_drive"]["collision"]["right"].GetFloat();

                        NLOG(debug) << "from collision " << o_link.from_driving_collision_enable << " "
                                    << o_link.from_driving_collision_margin_front_m << " " << o_link.from_driving_collision_margin_rear_m
                                    << " " << o_link.from_driving_collision_margin_left_m << " "
                                    << o_link.from_driving_collision_margin_right_m;
                    }
                    LOG_DEBUG(
                        "active %d start %d stop %d", o_link.from_driving_active, o_link.from_driving_start_smooth,
                        o_link.from_driving_stop_smooth);
                    LOG_DEBUG(
                        "avoid from %d %d %d", o_link.from_driving_avoidance_right, o_link.from_driving_avoidance_left,
                        o_link.from_driving_avoidance_step);
                }

                if (LINKS[i].HasMember("to_link_drive")) {
                    o_link.to_driving_direction = GetLinkDriveDirType(LINKS[i]["to_link_drive"]["direction"].GetString());
                    o_link.to_driving_linear_speed = LINKS[i]["to_link_drive"]["linear_speed"].GetFloat();
                    NLOG(debug) << "o_link.to_driving_linear_speed " << o_link.to_driving_linear_speed;
                    if (fabs(o_link.to_driving_linear_speed) > 10) {
                        o_link.to_driving_linear_speed = std::stof(LINKS[i]["to_link_drive"]["linear_speed"].GetString());
                        NLOG(debug) << "o_link.to_driving_linear_speed2 " << o_link.to_driving_linear_speed;
                    }
                    o_link.to_driving_diagonal_curve = LINKS[i]["to_link_drive"]["diagonal_curve"].GetFloat();
                    o_link.to_driving_active = LINKS[i]["to_link_drive"]["active"].GetBool();
                    o_link.to_driving_start_smooth = LINKS[i]["to_link_drive"]["start_smooth"].GetBool();  // brain 추가 예정
                    o_link.to_driving_stop_smooth = LINKS[i]["to_link_drive"]["stop_smooth"].GetBool();  // brain 추가 예정
                    o_link.to_driving_avoidance_right = LINKS[i]["to_link_drive"]["avoidance_right"].GetBool();
                    o_link.to_driving_avoidance_left = LINKS[i]["to_link_drive"]["avoidance_left"].GetBool();
                    o_link.to_driving_avoidance_step = LINKS[i]["to_link_drive"]["avoidance_step"].GetInt();

                    if (LINKS[i]["to_link_drive"].HasMember("collision")) {
                        o_link.to_driving_collision_enable = LINKS[i]["to_link_drive"]["collision"]["enabled"].GetBool();
                        o_link.to_driving_collision_margin_front_m = LINKS[i]["to_link_drive"]["collision"]["front"].GetFloat();
                        o_link.to_driving_collision_margin_rear_m = LINKS[i]["to_link_drive"]["collision"]["rear"].GetFloat();
                        o_link.to_driving_collision_margin_left_m = LINKS[i]["to_link_drive"]["collision"]["left"].GetFloat();
                        o_link.to_driving_collision_margin_right_m = LINKS[i]["to_link_drive"]["collision"]["right"].GetFloat();

                        NLOG(debug) << "to collision " << o_link.to_driving_collision_enable << " "
                                    << o_link.to_driving_collision_margin_front_m << " " << o_link.to_driving_collision_margin_rear_m << " "
                                    << o_link.to_driving_collision_margin_left_m << " " << o_link.to_driving_collision_margin_right_m;
                    }

                    LOG_DEBUG(
                        "active %d start %d stop %d", o_link.to_driving_active, o_link.to_driving_start_smooth,
                        o_link.to_driving_stop_smooth);
                    LOG_DEBUG(
                        "avoid to %d %d %d", o_link.to_driving_avoidance_right, o_link.to_driving_avoidance_left,
                        o_link.to_driving_avoidance_step);
                }

                if (LINKS[i].HasMember("arc_info")) {
                    if (o_link.type == core_msgs::JsonLinkList::ARC) {
                        o_link.entry_angle_deg = LINKS[i]["arc_info"]["from_angle"].GetFloat();
                        o_link.exit_angle_deg = LINKS[i]["arc_info"]["to_angle"].GetFloat();

                        o_link.arc_center_x = LINKS[i]["arc_info"]["arc_center"]["x"].GetFloat();
                        o_link.arc_center_y = LINKS[i]["arc_info"]["arc_center"]["y"].GetFloat();

                        o_link.arc_start_position_x = LINKS[i]["arc_info"]["arc_start"]["x"].GetFloat();
                        o_link.arc_start_position_y = LINKS[i]["arc_info"]["arc_start"]["y"].GetFloat();
                        o_link.radius =
                            hypotf(o_link.arc_start_position_x - o_link.arc_center_x, o_link.arc_start_position_y - o_link.arc_center_y);
                        o_link.arc_end_position_x = LINKS[i]["arc_info"]["arc_end"]["x"].GetFloat();
                        o_link.arc_end_position_y = LINKS[i]["arc_info"]["arc_end"]["y"].GetFloat();

                        o_link.line_1_start_position_x = LINKS[i]["arc_info"]["line1_start"]["x"].GetFloat();
                        o_link.line_1_start_position_y = LINKS[i]["arc_info"]["line1_start"]["y"].GetFloat();
                        o_link.line_1_end_position_x = LINKS[i]["arc_info"]["line1_end"]["x"].GetFloat();
                        o_link.line_1_end_position_y = LINKS[i]["arc_info"]["line1_end"]["y"].GetFloat();

                        o_link.line_2_start_position_x = LINKS[i]["arc_info"]["line2_start"]["x"].GetFloat();
                        o_link.line_2_start_position_y = LINKS[i]["arc_info"]["line2_start"]["y"].GetFloat();
                        o_link.line_2_end_position_x = LINKS[i]["arc_info"]["line2_end"]["x"].GetFloat();
                        o_link.line_2_end_position_y = LINKS[i]["arc_info"]["line2_end"]["y"].GetFloat();

                        o_link.b_arc_is_ccw = LINKS[i]["arc_info"]["arc_is_ccw"].GetBool();
                    }
                }
                o_json_list.links.push_back(o_link);
            }
        }
    }
    else {
        LOG_ERROR("json no has links!");
    }
    return o_json_list;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_total_tester_node");
    TotalTester tester;
    // ros::spin();
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
