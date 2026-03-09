#include "core_msgs/CommonString.h"
#include "nc_brain_map_server/nc_brain_map_server_pch.h"

#include <nc_brain_map_server/nc_brain_map_server_main.hpp>

using namespace NaviFra;

BrainMapServer::BrainMapServer(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    LOG_DEBUG("");

    service_client_ = nh_.serviceClient<core_msgs::CommonString>("answer/control");
    json_publisher_ = nh_.advertise<core_msgs::MapJson>("map/all", 1, true);
    nodes_publisher_ = nh_.advertise<core_msgs::JsonList>("map/nodelink", 1, true);
    map_db_pub_ = nh_.advertise<core_msgs::MapDB>("map/db", 1, true);
    map_request_sub_ = nh_.subscribe("map_request", 10, &BrainMapServer::MapRequestCallback, this);
    map_db_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("map_db_cloud", 1, true);

    std::string s_json_path;
    std::string s_json_folder;
    char* homePath = std::getenv("HOME");
    std::string homeDir(homePath);
    homeDir += "/navifra_solution/navicore/configs/map/latest/";
    LOG_DEBUG("map json path : %s", homeDir.c_str());
    mapDir_ = homeDir;
    LoadMapJson(mapDir_, "");
}

BrainMapServer::~BrainMapServer()
{
    LOG_DEBUG("");
}

void BrainMapServer::MapRequestCallback(const std_msgs::String::ConstPtr& msg)
{
    LOG_INFO("requested %s ", msg->data.c_str());
    LoadMapJson(mapDir_, msg->data);
}

void BrainMapServer::LoadMapJson(const std::string& map_json_path, const std::string& cmd)
{
    JsonStream o_json_stream;
    o_json_stream.ReadJsonFromFile(map_json_path + "map.json");
    rapidjson::Document& json_doc = o_json_stream.GetJsonFile();

    JsonStream o_json_stream_teach;
    o_json_stream_teach.ReadJsonFromFile(map_json_path + "teaching.json");
    rapidjson::Document& json_doc_teach = o_json_stream_teach.GetJsonFile();

    if (json_doc.HasMember("id")) {
        std_msgs::String cur_map_name_msg;
        cur_map_name_msg.data = json_doc["id"].GetString();
        // cur_map_pub_.publish(cur_map_name_msg);
    }
    else {
        LOG_ERROR("No map.json id");
        return;
    }
    core_msgs::MapJson o_map_json;
    core_msgs::MapDB o_map_db;
    core_msgs::JsonList o_map_nodelink;

    o_map_nodelink = GetNodeLinkData(json_doc);
    // teaching 데이터로 덮어쓰기
    o_map_nodelink = GetTeachingData(o_map_nodelink, json_doc_teach);
    nodes_publisher_.publish(o_map_nodelink);

    if (json_doc.HasMember("occupied_space") && (cmd == "" || cmd == "map")) {
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

        sensor_msgs::PointCloud map_db_cloud;
        geometry_msgs::Point32 point;

        for (const auto& db_item : db_list.GetArray()) {
            geometry_msgs::Pose o_pos;
            o_pos.position.x = db_item[0].GetFloat();
            o_pos.position.y = db_item[1].GetFloat();

            point.x = o_pos.position.x;
            point.y = o_pos.position.y;
            map_db_cloud.points.emplace_back(point);

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

        map_db_cloud.header.frame_id = "map";
        map_db_cloud_pub_.publish(map_db_cloud);

        // answer api
        // 1. add command member for answer
        rapidjson::Document::AllocatorType& allocator = json_doc.GetAllocator();
        json_doc.AddMember("command", "load_map", allocator);

        // 2. stringify json
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        json_doc.Accept(writer);
        std::string json_string = buffer.GetString();

        // 3. call service
        core_msgs::CommonString srv;
        srv.request.data = json_string;
        if (service_client_.call(srv)) {
            LOG_INFO("service call ok! %d", srv.response.success);
        }
        else {
            LOG_WARNING("service call failed!");
        }
    }
    else {
        LOG_WARNING("No map.json map_data or cmd : %s only update nodelink", cmd.c_str());
        return;
    }
}
core_msgs::JsonList BrainMapServer::GetTeachingData(const core_msgs::JsonList& json_list, rapidjson::Document& doc)
{
    core_msgs::JsonList o_original_list = json_list;

    if (doc.HasMember("nodes")) {
        const Value& NODES = doc["nodes"];
        if (false == NODES.Empty()) {
            for (int i = 0; i < NODES.Size(); i++) {
                LOG_DEBUG("find id : %s", NODES[i]["id"].GetString());

                for (int j = 0; j < o_original_list.nodes.size(); j++) {
                    if (o_original_list.nodes[j].id == NODES[i]["id"].GetString()) {
                        LOG_DEBUG(
                            "find ok! id : %s, name : %s", o_original_list.nodes[j].id.c_str(), o_original_list.nodes[j].name.c_str());

                        if (NODES[i].HasMember("position")) {
                            o_original_list.nodes[j].position.x = NODES[i]["position"]["x"].GetFloat();
                            o_original_list.nodes[j].position.y = NODES[i]["position"]["y"].GetFloat();
                            o_original_list.nodes[j].position.z = NODES[i]["position"]["z"].GetFloat();
                            LOG_DEBUG(
                                "position : %.3f %.3f %.3f", o_original_list.nodes[j].position.x, o_original_list.nodes[j].position.y,
                                o_original_list.nodes[j].position.z);
                        }
                        if (NODES[i].HasMember("orientation")) {
                            o_original_list.nodes[j].orientation.w = NODES[i]["orientation"]["w"].GetFloat();
                            o_original_list.nodes[j].orientation.x = NODES[i]["orientation"]["x"].GetFloat();
                            o_original_list.nodes[j].orientation.y = NODES[i]["orientation"]["y"].GetFloat();
                            o_original_list.nodes[j].orientation.z = NODES[i]["orientation"]["z"].GetFloat();
                            LOG_DEBUG(
                                "orientation(w,x,y,z) : %.3f %.3f %.3f %.3f", o_original_list.nodes[j].orientation.w,
                                o_original_list.nodes[j].orientation.x, o_original_list.nodes[j].orientation.y,
                                o_original_list.nodes[j].orientation.z);
                        }
                        break;
                    }
                }
            }
        }
    }
    else {
        LOG_ERROR("json no has nodes!");
    }

    return o_original_list;
}

core_msgs::JsonList BrainMapServer::GetNodeLinkData(rapidjson::Document& doc)
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
                    LOG_ERROR("extra type, extra error");
                }

                try {
                    if (NODES[i].HasMember("action") && !NODES[i]["action"].IsNull() && NODES[i]["action"].IsString()) {
                        o_node.action = NODES[i]["action"].GetString();
                    }
                    else {
                        o_node.action = "sim_action";
                    }
                }
                catch (std::runtime_error& e) {
                    LOG_ERROR("action error");
                }

                try {
                    if (NODES[i].HasMember("parameters") && NODES[i]["parameters"].HasMember("custom")) {
                        const auto& custom = NODES[i]["parameters"]["custom"];
                        
                        rapidjson::StringBuffer buffer;
                        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
                        custom.Accept(writer);
                        o_node.custom = buffer.GetString();
                    }
                }
                catch (std::runtime_error& e) {
                    LOG_ERROR("custom parsing error");
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
    ros::init(argc, argv, "nc_brain_map_server", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    NaviFra::Logger::get().SetSeverityMin(severity_level::info);

    NaviFra::BrainMapServer ms(nh, nhp);
    try {
        ros::spin();
    }
    catch (std::runtime_error& e) {
        LOG_ERROR("map_server exception: %s", e.what());
        return -1;
    }
    return 0;
}
