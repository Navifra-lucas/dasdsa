#include "answer_core_bridge/answer_core_bridge.h"

AnswerCoreBridge::AnswerCoreBridge()
{
    ROS_INFO("AnswerCoreBridge created");
    Initialize();
    localize_result_error_code_ = 0;
    slam_start_ = false;
}
AnswerCoreBridge::~AnswerCoreBridge()
{
}

void AnswerCoreBridge::Initialize()
{
    service_client_ = nh_.serviceClient<answer::common_service>("answer/control");
    // map_request_sub_ = nh_.subscribe("map_request", 10, &AnswerCoreBridge::MapRequestCallback, this);
    // answer_control_sub_ = nh_.subscribe("answer/bridge", 10, &AnswerCoreBridge::AnswerControlCallback, this);
    answer_report_sub_ = nh_.subscribe("answer/report", 10, &AnswerCoreBridge::AnswerReportCallback, this);
    answer_status_sub_ = nh_.subscribe("answer/status", 10, &AnswerCoreBridge::AnswerStatusCallback, this);
    answer_detected_reflectors_sub_ =
        nh_.subscribe("answer/detected_reflectors", 10, &AnswerCoreBridge::AnswerDetectedReflectorsCallback, this);
    localize_info_pub_ = nh_.advertise<core_msgs::LocalizeInfo>("localize_info", 10);

    reflectors_pub_ = nh_.advertise<std_msgs::String>("answer/set_reflectors", 1);
    ROS_INFO("AnswerCoreBridge initialized");
    std::vector<std::string> nodes;
    nodes.emplace_back("answer");
    if (!ros::master::getNodes(nodes)) {
        ROS_ERROR("answer node is not known");
    }
    else {
        // initialize answer node
        // StartDefaultMode();
        // ROS_INFO("answer node is running");
    }
    ReadReflectorMap();
}
void AnswerCoreBridge::Terminate()
{
    service_client_.shutdown();
    map_request_sub_.shutdown();
    answer_control_sub_.shutdown();
    answer_report_sub_.shutdown();
    answer_status_sub_.shutdown();
    localize_info_pub_.shutdown();
    ROS_INFO("AnswerCoreBridge terminated");
}

void AnswerCoreBridge::AnswerDetectedReflectorsCallback(const std_msgs::String::ConstPtr& msg)
{
#if 0
    Poco::JSON::Object::Ptr reflector_map_json = new Poco::JSON::Object();
    Poco::JSON::Array::Ptr reflectors = new Poco::JSON::Array();
    bool already_exists = false;
    for (const auto& pose : msg->poses) {
        std::string str_uuid;
        for (const auto& [uuid, point] : reflector_map_) {
            if (std::hypotf(point.x - pose.position.x, point.y - pose.position.y) < 0.05f) {
                // If the point already exists in the map, skip adding it again
                already_exists = true;
                str_uuid = uuid;
                break;
            }
        }

        if (!already_exists) {
            str_uuid = Poco::UUIDGenerator::defaultGenerator().create().toString();
            reflector_map_[str_uuid] = Point2D{pose.position.x, pose.position.y};
        }
        // Convert geometry_msgs::Pose to Eigen::Vector2f
        Eigen::Vector2f reflector_position;
        reflector_position.x() = pose.position.x;
        reflector_position.y() = pose.position.y;
        Poco::JSON::Object::Ptr point = new Poco::JSON::Object();
        point->set("x", reflector_position.x());
        point->set("y", reflector_position.y());
        point->set("uuid", str_uuid);
        reflectors->add(point);
    }
    #if 0
    *reflector_map_json = reflector_map_json_;
    #endif

    reflector_map_json->set("detected", reflectors);

    std::ostringstream oss;
    try {
        reflector_map_json->stringify(oss);
        // ROS_INFO("Reflector map read successfully {%s}", oss.str().c_str());
        std_msgs::String reflectors_msg;
        reflectors_msg.data = oss.str();
        reflectors_pub_.publish(reflectors_msg);
    }
    catch (Poco::Exception& ex) {
        ROS_ERROR("Error %s", ex.displayText().c_str());
    }
#endif
    reflectors_pub_.publish(*msg);
}

void AnswerCoreBridge::AnswerStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    Poco::JSON::Object::Ptr jsonObject = nullptr;
    try {
        // Create a Poco JSON parser
        Poco::JSON::Parser parser;

        // Parse the JSON string
        Poco::Dynamic::Var result = parser.parse(msg->data.c_str());

        // Convert to JSON Object
        jsonObject = result.extract<Poco::JSON::Object::Ptr>();
        if (jsonObject->has("alarm")) {
            std::string alarm = jsonObject->getValue<std::string>("alarm");
            // find listed alarm in string. It is distiguished by multiple '/' behind "list"
            std::string::size_type pos = alarm.find("list");
            if (alarm.size() > 4 && pos != std::string::npos) {
                std::string alarm_list = alarm.substr(pos + 5);
                std::istringstream iss(alarm_list);
                std::string item;
                while (std::getline(iss, item, '/')) {
                    ROS_INFO("alarm: %s", item.c_str());
                    if (item == "LOCALIZATION_LARGE_CORRECTION") {
                        {
                            std::lock_guard<std::mutex> lock(status_mutex_);
                            localize_result_error_code_ = core_msgs::NaviAlarm::LOCALIZATION_LARGE_CORRECTION;
                        }
                    }
                    else if (item == "LOCALIZATION_NOT_INITIALIZED") {
                    }
                    else if (item == "LOCALIZATION_MAP_NOT_READY") {
                        {
                            std::lock_guard<std::mutex> lock(status_mutex_);
                            localize_result_error_code_ = core_msgs::NaviAlarm::ERROR_MAP_NOT_LOADED;
                        }
                    }
                    else {
                        {
                            std::lock_guard<std::mutex> lock(status_mutex_);
                            localize_result_error_code_ = 0;
                        }
                    }
                }
            }
        }
        if (jsonObject->has("status")) {
            std::string status = jsonObject->getValue<std::string>("status");
            // ROS_INFO("status: %s", status.c_str());
            if (status == "NONE") {
                {
                    bool temp;
                    {
                        std::lock_guard<std::mutex> lock(slam_start_mutex_);
                        temp = slam_start_;
                    }
                    if (temp == false) {
                        StartDefaultMode();

                        ROS_INFO("StartDefaultMode called");
                        // core_msgs::LocalizeInfo localize_info;
                        // localize_info.b_localizer_ready = true;
                        // localize_info.n_confidence = 100;
                        // localize_info_pub_.publish(localize_info);
                    }
                }
            }
            else if (status == "LOCALZATION") {
                // b_localizer_ready_ = true;
            }
            else if (status == "SLAM") {
            }
            else if (status == "ERROR") {
            }
        }
    }
    catch (Poco::Exception& ex) {
        ROS_ERROR("Error %s", ex.displayText().c_str());
    }

    // ReadReflectorMap();
}
void AnswerCoreBridge::MapRequestCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("map request received");
    std::string map_full_path = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/map.json";

    answer::common_service srv;
    Poco::File file(map_full_path);
    if (file.exists()) {
        std::ostringstream oss;
        Poco::FileInputStream fis(map_full_path);
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(fis);
        Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

        object->set("command", "load_map");
        object->stringify(oss, 4);

        srv.request.data = oss.str();
        if (service_client_.call(srv)) {
            ROS_INFO("Service call success");
        }
        else {
            ROS_ERROR("Service call failed");
        }
    }
}

void AnswerCoreBridge::AnswerReportCallback(const std_msgs::String::ConstPtr& msg)
{
    Poco::JSON::Object::Ptr jsonObject = nullptr;
    try {
        // Create a Poco JSON parser
        Poco::JSON::Parser parser;

        // Parse the JSON string
        Poco::Dynamic::Var result = parser.parse(msg->data.c_str());

        // Convert to JSON Object
        jsonObject = result.extract<Poco::JSON::Object::Ptr>();
        if (jsonObject->has("report")) {
            auto report = jsonObject->getObject("report");
            std::string localizer_mode = "lidar";
            if (report->has("mode")) {
                localizer_mode = report->get("mode").convert<std::string>();
            }
            if (report->has("matching_result")) {
                // get confidence in matching_result
                auto matching_result = report->getObject("matching_result");
                if (matching_result->has("confidence")) {
                    float confidence = matching_result->get("confidence").convert<float>();
                    auto correct_result = matching_result->getObject("correction");
                    float f_correct_x_m = correct_result->get("x").convert<float>();
                    float f_correct_y_m = correct_result->get("y").convert<float>();
                    float f_correct_deg = correct_result->get("t").convert<float>();

                    core_msgs::LocalizeInfo localize_info;
                    {
                        std::lock_guard<std::mutex> lock(status_mutex_);
                        localize_info.localize_result_error_code = 0;
                    }
                    localize_info.b_localizer_ready = true;
                    localize_info.n_confidence = std::lroundf(confidence);

                    localize_info.localize_angle_error_code = 0;
                    localize_info.s_localizer_mode = localizer_mode;
                    localize_info.f_correct_x_m = f_correct_x_m;
                    localize_info.f_correct_y_m = f_correct_y_m;
                    localize_info.f_correct_deg = f_correct_deg;
                    localize_info_pub_.publish(localize_info);
                }
            }
        }
    }
    catch (Poco::Exception& ex) {
        ROS_ERROR("Error %s", ex.displayText().c_str());
    }
    // localize_info_pub_
}

bool AnswerCoreBridge::AnswerControlCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string command = msg->data;
    std::ostringstream oss;
    Poco::JSON::Object::Ptr object = new Poco::JSON::Object();
    bool slam_start = false;
    if (command == "start_slam") {
        object->set("command", "start_slam");
        slam_start = true;
    }
    else if (command == "terminate_slam") {
        object->set("command", "terminate_slam");
    }
    else if (command == "start_localization") {
        object->set("command", "start_localization");
    }
    else if (command == "terminate_localization") {
        object->set("command", "terminate_localization");
    }
    else {
        ROS_ERROR("Unknown command: %s", command.c_str());
        return false;
    }

    // core_msgs::CommonString srv;
    answer::common_service srv;

    object->stringify(oss, 4);

    srv.request.data = oss.str();
    if (service_client_.call(srv)) {
        // if (slam_start) {
        //     std::lock_guard<std::mutex> lock(slam_start_mutex_);
        //     slam_start_ = true;
        // }
        // else {
        //     std::lock_guard<std::mutex> lock(slam_start_mutex_);
        //     slam_start_ = false;
        // }
        ROS_INFO("Service call success %s", oss.str().c_str());
    }
    else {
        {
            std::lock_guard<std::mutex> lock(slam_start_mutex_);
            slam_start_ = false;
        }
        ROS_ERROR("Service call failed %s", oss.str().c_str());
        return false;
    }
    return true;
}

void AnswerCoreBridge::StartDefaultMode()
{
    std_msgs::String msg;

    msg.data = "terminate_slam";
    boost::shared_ptr<const std_msgs::String> terminate_slam_ptr = boost::make_shared<std_msgs::String>(msg);
    AnswerControlCallback(terminate_slam_ptr);
    msg.data = "terminate_localization";
    boost::shared_ptr<const std_msgs::String> terminate_msg_ptr = boost::make_shared<std_msgs::String>(msg);
    AnswerControlCallback(terminate_msg_ptr);
    msg.data = "start_localization";
    boost::shared_ptr<const std_msgs::String> start_msg_ptr = boost::make_shared<std_msgs::String>(msg);
    AnswerControlCallback(start_msg_ptr);
}

void AnswerCoreBridge::ReadReflectorMap()
{
    std::string reflector_info = std::string(std::getenv("HOME")) + "/.ros/answer/reflectors.json";
    if (!Poco::File(reflector_info).exists()) {
        ROS_WARN("Reflector map file does not exist Generate new one");
    }
    else {
        // Open and read the file
        Poco::FileInputStream fileStream(reflector_info);
        // Parse JSON content
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(fileStream);
        Poco::JSON::Object::Ptr obj = result.extract<Poco::JSON::Object::Ptr>();
#if 0
        Poco::JSON::Array::Ptr reflectors = obj->getArray("reflectors");
        ROS_INFO("Registered Reflectors {%ld}", reflectors->size());

        if (obj->has("reflectors") == false) {
            ROS_WARN("Reflector map is empty");
            return;
        }
        for (size_t i = 0; i < reflectors->size(); ++i) {
            // Poco::JSON::Array::Ptr row = reflectors->getArray(i);
            Poco::Dynamic::Var row = reflectors->get(i);
            Poco::JSON::Object::Ptr row_obj = row.extract<Poco::JSON::Object::Ptr>();
            if (row_obj->has("x") == false || row_obj->has("y") == false || row_obj->has("uuid") == false) {
                ROS_WARN("Reflector map row is not valid, expected x, y, uuid ");
                continue;
            }
            float x = row_obj->getValue<float>("x");
            float y = row_obj->getValue<float>("y");
            std::string uuid = row_obj->getValue<std::string>("uuid");
            // ROS_INFO("Reflector : ({%.3f}, {%.3f} {%s})", x, y, uuid.c_str());
            // reflector_map_[uuid] = Eigen::Vector2f(x, y);
        }
        // reflector_map_ =
#endif
        reflector_map_json_ = *obj;
        // std::ostringstream oss;
        // try {
        //     obj->stringify(oss);
        //     str_reflector_map_ = oss.str();
        //     ROS_INFO("Reflector map read successfully {%s}", str_reflector_map_.c_str());
        // }
        // catch (Poco::Exception& ex) {
        //     ROS_ERROR("Error %s", ex.displayText().c_str());
        // }
    }
}
