#include "nc_task_manager/waypoint_planner.hpp"

#include "core/util/logger.hpp"

namespace NaviFra {
WaypointPlanner::WaypointPlanner()
{
    param_update_sub_ = nh_.subscribe("navifra/param_update", 1, &WaypointPlanner::RecvParamUpdate, this);
    InitParameter();
}
WaypointPlanner::~WaypointPlanner()
{
}

void WaypointPlanner::RecvParamUpdate(const std_msgs::String::ConstPtr& msg)
{
    InitParameter();
}

void WaypointPlanner::InitParameter()
{
    ros::param::param<float>("planner_base/f_start_thr_dist_m", f_start_thr_dist_m_, 0.25);
}

NaviFra::Pos::DriveInfo_t WaypointPlanner::GetDriveInfo(const NaviNode& node_from, const NaviNode& node_to)
{
    NaviFra::Pos::DriveInfo_t o_drive;
    bool b_find = false;
    for (int i = 0; i < vec_all_node_.size(); i++) {
        if (vec_all_node_.at(i).GetID() == node_from.GetID()) {
            std::vector<NaviFra::NaviEdge> vec_edges = vec_all_node_.at(i).GetAllEdge();

            for (int j = 0; j < vec_edges.size(); j++) {
                if (CalcPosDistance_(vec_edges.at(j).GetStartPos(), node_to) < 0.01) {
                    o_drive = vec_edges.at(j).GetToDriveInfo();
                    b_find = true;
                    break;
                }
                else if (CalcPosDistance_(vec_edges.at(j).GetEndPos(), node_to) < 0.01) {
                    o_drive = vec_edges.at(j).GetFromDriveInfo();
                    b_find = true;
                    break;
                }
            }
        }
        if (b_find) {
            break;
        }
    }
    if (!b_find) {
        NLOG(info) << "GetDriveInfo fail";
    }

    return o_drive;
}

void WaypointPlanner::SetMap(const std::vector<NaviNode>& vec_topological_map)
{
    LOG_INFO("vec_topological_map size : %d", vec_topological_map.size());
    vec_edge_data_.clear();

    vec_all_node_ = vec_topological_map;
    int n_node_size = vec_all_node_.size();
    if (n_node_size == 0) {
        LOG_WARNING("vec_topological_map size Finished! : %d", vec_topological_map.size());
        return;
    }
    o_dijkstra_algorithm_.SetNode(n_node_size);

    for (size_t i = 0; i < n_node_size; ++i) {
        MakeGraph(vec_all_node_.at(i));
    }

    int n_edge_cnt = 0;
    for (int i = 0; i < n_node_size; i++) {
        n_edge_cnt += vec_all_node_.at(i).GetAllEdge().size();
    }
    vec_edge_data_.reserve(n_edge_cnt);

    for (int i = 0; i < n_node_size; i++) {
        std::vector<NaviFra::NaviEdge> vec_edges = vec_all_node_.at(i).GetAllEdge();
        vec_edge_data_.insert(vec_edge_data_.end(), vec_edges.begin(), vec_edges.end());
    }
}

void WaypointPlanner::SetMap(const core_msgs::JsonList& msg)
{
    std::map<string, int> map_node;
    std::vector<NaviFra::NaviNode> vec_graph;
    int idx = -1;
    for (int i = 0; i < msg.nodes.size(); i++) {
        NaviFra::NaviNode o_node;
        geometry_msgs::Pose o_tmp_pos;
        o_tmp_pos.position = msg.nodes.at(i).position;
        o_tmp_pos.orientation = msg.nodes.at(i).orientation;

        o_node.SetXm(o_tmp_pos.position.x);
        o_node.SetYm(o_tmp_pos.position.y);
        o_node.SetQuaternion(o_tmp_pos.orientation.w, o_tmp_pos.orientation.x, o_tmp_pos.orientation.y, o_tmp_pos.orientation.z);

        if (msg.nodes.at(i).type == core_msgs::JsonNodeList::NONE) {
            o_node.SetType(NaviFra::Pos::NODE_TYPE::NONE);
        }
        else if (msg.nodes.at(i).type == core_msgs::JsonNodeList::POI) {
            o_node.SetType(NaviFra::Pos::NODE_TYPE::POI);
        }
        else if (msg.nodes.at(i).type == core_msgs::JsonNodeList::SPIN_TURN) {
            o_node.SetType(NaviFra::Pos::NODE_TYPE::SPIN_TURN);
        }
        else {
            o_node.SetType(NaviFra::Pos::NODE_TYPE::NONE);
        }
        if (msg.nodes.at(i).alarm_flag) {
            o_node.SetAlarmData(msg.nodes.at(i).alarm_mm / 1000, msg.nodes.at(i).alarm_deg);
        }

        o_node.SetID(msg.nodes.at(i).id);
        o_node.SetName(msg.nodes.at(i).name);
        o_node.SetActionName(msg.nodes.at(i).action);
        idx++;
        o_node.SetNodeNum(idx);
        vec_graph.emplace_back(o_node);
        map_node.insert(make_pair(msg.nodes.at(i).id, o_node.GetNumber()));
    }

    for (int i = 0; i < msg.links.size(); i++) {
        NaviFra::Pos::DRIVE_TYPE from_driving_dir;
        NaviFra::Pos::DRIVE_TYPE to_driving_dir;

        if (msg.links.at(i).from_driving_direction == core_msgs::JsonLinkList::FRONT)
            from_driving_dir = NaviFra::Pos::DRIVE_TYPE::FRONT;
        if (msg.links.at(i).from_driving_direction == core_msgs::JsonLinkList::REAR)
            from_driving_dir = NaviFra::Pos::DRIVE_TYPE::REAR;
        if (msg.links.at(i).to_driving_direction == core_msgs::JsonLinkList::FRONT)
            to_driving_dir = NaviFra::Pos::DRIVE_TYPE::FRONT;
        if (msg.links.at(i).to_driving_direction == core_msgs::JsonLinkList::REAR)
            to_driving_dir = NaviFra::Pos::DRIVE_TYPE::REAR;

        if (msg.links.at(i).from_driving_direction == core_msgs::JsonLinkList::FRONTDIAGONAL)
            from_driving_dir = NaviFra::Pos::DRIVE_TYPE::FRONT_DIAGONAL;  // for quad type robot
        if (msg.links.at(i).from_driving_direction == core_msgs::JsonLinkList::REARDIAGONAL)
            from_driving_dir = NaviFra::Pos::DRIVE_TYPE::REAR_DIAGONAL;  // for quad type robot
        if (msg.links.at(i).to_driving_direction == core_msgs::JsonLinkList::FRONTDIAGONAL)
            to_driving_dir = NaviFra::Pos::DRIVE_TYPE::FRONT_DIAGONAL;  // for quad type robot
        if (msg.links.at(i).to_driving_direction == core_msgs::JsonLinkList::REARDIAGONAL)
            to_driving_dir = NaviFra::Pos::DRIVE_TYPE::REAR_DIAGONAL;  // for quad type robot

        NaviFra::Pos::DriveInfo_t s_from_drive_info;
        s_from_drive_info.e_drive_type = from_driving_dir;
        s_from_drive_info.f_linear = msg.links.at(i).from_driving_linear_speed;
        s_from_drive_info.f_diagonal_curve = msg.links.at(i).from_driving_diagonal_curve;
        s_from_drive_info.b_active = msg.links.at(i).from_driving_active;
        s_from_drive_info.b_start_smooth = msg.links.at(i).from_driving_start_smooth;
        s_from_drive_info.b_stop_smooth = msg.links.at(i).from_driving_stop_smooth;

        s_from_drive_info.b_avoidance_right = msg.links.at(i).from_driving_avoidance_right;
        s_from_drive_info.b_avoidance_left = msg.links.at(i).from_driving_avoidance_left;
        s_from_drive_info.n_avoidance_step = msg.links.at(i).from_driving_avoidance_step;

        if (bool(msg.links.at(i).from_driving_collision_enable)) {
            s_from_drive_info.f_obstacle_outline_margin_front = msg.links.at(i).from_driving_collision_margin_front_m;
            s_from_drive_info.f_obstacle_outline_margin_rear = msg.links.at(i).from_driving_collision_margin_rear_m;
            s_from_drive_info.f_obstacle_outline_margin_left = msg.links.at(i).from_driving_collision_margin_left_m;
            s_from_drive_info.f_obstacle_outline_margin_right = msg.links.at(i).from_driving_collision_margin_right_m;
        }
        else {
            s_from_drive_info.f_obstacle_outline_margin_front = 0.5;
            s_from_drive_info.f_obstacle_outline_margin_rear = 0.5;
            s_from_drive_info.f_obstacle_outline_margin_left = 0.5;
            s_from_drive_info.f_obstacle_outline_margin_right = 0.5;
        }

        NaviFra::Pos::DriveInfo_t s_to_drive_info;
        s_to_drive_info.e_drive_type = to_driving_dir;
        s_to_drive_info.f_linear = msg.links.at(i).to_driving_linear_speed;
        s_to_drive_info.f_diagonal_curve = msg.links.at(i).to_driving_diagonal_curve;
        s_to_drive_info.b_active = msg.links.at(i).to_driving_active;
        s_to_drive_info.b_start_smooth = msg.links.at(i).to_driving_start_smooth;
        s_to_drive_info.b_stop_smooth = msg.links.at(i).to_driving_stop_smooth;

        s_to_drive_info.b_avoidance_right = msg.links.at(i).to_driving_avoidance_right;
        s_to_drive_info.b_avoidance_left = msg.links.at(i).to_driving_avoidance_left;
        s_to_drive_info.n_avoidance_step = msg.links.at(i).to_driving_avoidance_step;

        if (bool(msg.links.at(i).to_driving_collision_enable)) {
            s_to_drive_info.f_obstacle_outline_margin_front = msg.links.at(i).to_driving_collision_margin_front_m;
            s_to_drive_info.f_obstacle_outline_margin_rear = msg.links.at(i).to_driving_collision_margin_rear_m;
            s_to_drive_info.f_obstacle_outline_margin_left = msg.links.at(i).to_driving_collision_margin_left_m;
            s_to_drive_info.f_obstacle_outline_margin_right = msg.links.at(i).to_driving_collision_margin_right_m;
        }
        else {
            s_to_drive_info.f_obstacle_outline_margin_front = 0.5;
            s_to_drive_info.f_obstacle_outline_margin_rear = 0.5;
            s_to_drive_info.f_obstacle_outline_margin_left = 0.5;
            s_to_drive_info.f_obstacle_outline_margin_right = 0.5;
        }

        auto auto_start = map_node.find(msg.links.at(i).from_link);
        if (auto_start == map_node.end())
            continue;

        auto auto_end = map_node.find(msg.links.at(i).to_link);
        if (auto_end == map_node.end())
            continue;

        int n_start_node_num = auto_start->second;
        int n_end_node_num = auto_end->second;

        auto start_node = find(vec_graph.begin(), vec_graph.end(), n_start_node_num);
        auto end_node = find(vec_graph.begin(), vec_graph.end(), n_end_node_num);

        if (msg.links.at(i).type == core_msgs::JsonLinkList::LINE) {
            s_from_drive_info.e_curve_type = NaviFra::Pos::LINE_TYPE::LINE;
            s_to_drive_info.e_curve_type = NaviFra::Pos::LINE_TYPE::LINE;

            if (msg.links.at(i).from_driving_active) {
                float f_dist_m = hypot((*start_node).GetXm() - (*end_node).GetXm(), (*start_node).GetYm() - (*end_node).GetYm());
                vec_graph.at((*start_node).GetNumber())
                    .AddEdge(
                        *start_node, *end_node, (*start_node).GetNumber(), (*end_node).GetNumber(), s_from_drive_info, s_to_drive_info,
                        f_dist_m);
            }
            if (msg.links.at(i).to_driving_active) {
                float f_dist_m = hypot((*end_node).GetXm() - (*start_node).GetXm(), (*end_node).GetYm() - (*start_node).GetYm());
                vec_graph.at((*end_node).GetNumber())
                    .AddEdge(
                        *end_node, *start_node, (*end_node).GetNumber(), (*start_node).GetNumber(), s_to_drive_info, s_from_drive_info,
                        f_dist_m);
            }
        }
        else if (msg.links.at(i).type == core_msgs::JsonLinkList::ARC) {
            NaviFra::NaviNode o_start_node = *start_node;
            NaviFra::NaviNode o_end_node = *end_node;
            NaviFra::NaviNode o_start_arc_node;
            NaviFra::NaviNode o_end_arc_node;

            o_start_arc_node.SetXm(msg.links.at(i).arc_start_position_x);
            o_start_arc_node.SetYm(msg.links.at(i).arc_start_position_y);
            o_start_arc_node.SetDeg(o_start_node.GetDeg());

            o_end_arc_node.SetXm(msg.links.at(i).arc_end_position_x);
            o_end_arc_node.SetYm(msg.links.at(i).arc_end_position_y);
            o_end_arc_node.SetDeg(o_end_node.GetDeg());

            {
                idx++;
                o_start_arc_node.SetNodeNum(idx);
                o_start_arc_node.SetID("ARCS" + o_start_node.GetID());
                o_start_arc_node.SetName("ARCS" + o_start_node.GetName());
                s_from_drive_info.e_curve_type = NaviFra::Pos::LINE_TYPE::LINE;
                s_to_drive_info.e_curve_type = NaviFra::Pos::LINE_TYPE::LINE;

                if (msg.links.at(i).from_driving_active) {
                    float f_dist_m =
                        hypot(o_start_node.GetXm() - o_start_arc_node.GetXm(), o_start_node.GetYm() - o_start_arc_node.GetYm());
                    vec_graph.at(n_start_node_num)
                        .AddEdge(
                            o_start_node, o_start_arc_node, n_start_node_num, o_start_arc_node.GetNumber(), s_from_drive_info,
                            s_to_drive_info, f_dist_m);
                }
                if (msg.links.at(i).to_driving_active) {
                    float f_dist_m =
                        hypot(o_start_arc_node.GetXm() - o_start_node.GetXm(), o_start_arc_node.GetYm() - o_start_node.GetYm());
                    o_start_arc_node.AddEdge(
                        o_start_arc_node, o_start_node, o_start_arc_node.GetNumber(), n_start_node_num, s_to_drive_info, s_from_drive_info,
                        f_dist_m);
                }
                vec_graph.emplace_back(o_start_arc_node);
            }

            {
                idx++;
                o_end_arc_node.SetNodeNum(idx);
                o_end_arc_node.SetID("ARCE" + o_end_node.GetID());
                o_end_arc_node.SetName("ARCE" + o_end_node.GetName());
                s_from_drive_info.e_curve_type = NaviFra::Pos::LINE_TYPE::LINE;
                s_to_drive_info.e_curve_type = NaviFra::Pos::LINE_TYPE::LINE;
                if (msg.links.at(i).from_driving_active) {
                    float f_dist_m = hypot(o_end_arc_node.GetXm() - o_end_node.GetXm(), o_end_arc_node.GetYm() - o_end_node.GetYm());
                    o_end_arc_node.AddEdge(
                        o_end_arc_node, o_end_node, o_end_arc_node.GetNumber(), n_end_node_num, s_from_drive_info, s_to_drive_info,
                        f_dist_m);
                }
                if (msg.links.at(i).to_driving_active) {
                    float f_dist_m = hypot(o_end_node.GetXm() - o_end_arc_node.GetXm(), o_end_node.GetYm() - o_end_arc_node.GetYm());
                    vec_graph.at(n_end_node_num)
                        .AddEdge(
                            o_end_node, o_end_arc_node, n_end_node_num, o_end_arc_node.GetNumber(), s_to_drive_info, s_from_drive_info,
                            f_dist_m);
                }
                vec_graph.emplace_back(o_end_arc_node);
            }
            float angle1 =
                atan2(o_end_arc_node.GetYm() - msg.links.at(i).arc_center_y, o_end_arc_node.GetXm() - msg.links.at(i).arc_center_x);
            float angle2 =
                atan2(o_start_arc_node.GetYm() - msg.links.at(i).arc_center_y, o_start_arc_node.GetXm() - msg.links.at(i).arc_center_x);
            float arcAngle = (angle2 - angle1) * RADtoDEG;

            if (angle1 < 0)
                angle1 += M_PI * 2;
            if (angle2 < 0)
                angle2 += M_PI * 2;

            float f_circle_angle;  // = NaviFra::CoreCalculator::WrapAnglePiToPiDeg_(arcAngle);

            if (bool(msg.links.at(i).b_arc_is_ccw)) {
                f_circle_angle = (angle1 - angle2) * RADtoDEG;
                if (f_circle_angle < 0)
                    f_circle_angle += 360;
            }
            else {
                f_circle_angle = -(angle2 - angle1) * RADtoDEG;
                if (f_circle_angle > 0)
                    f_circle_angle -= 360;
            }

            float f_arc_length = fabs(2 * M_PI * msg.links.at(i).radius * (f_circle_angle / 360));

            s_from_drive_info.e_curve_type = NaviFra::Pos::LINE_TYPE::CURVE;
            s_from_drive_info.f_circle_angle = f_circle_angle;
            s_from_drive_info.f_circle_pos_x = msg.links.at(i).arc_center_x;
            s_from_drive_info.f_circle_pos_y = msg.links.at(i).arc_center_y;
            s_to_drive_info.e_curve_type = NaviFra::Pos::LINE_TYPE::CURVE;
            s_to_drive_info.f_circle_angle = -f_circle_angle;
            s_to_drive_info.f_circle_pos_x = msg.links.at(i).arc_center_x;
            s_to_drive_info.f_circle_pos_y = msg.links.at(i).arc_center_y;
            if (msg.links.at(i).from_driving_active) {
                vec_graph.at(o_start_arc_node.GetNumber())
                    .AddEdge(
                        o_start_arc_node, o_end_arc_node, o_start_arc_node.GetNumber(), o_end_arc_node.GetNumber(), s_from_drive_info,
                        s_to_drive_info, f_arc_length);
            }
            if (msg.links.at(i).to_driving_active) {
                vec_graph.at(o_end_arc_node.GetNumber())
                    .AddEdge(
                        o_end_arc_node, o_start_arc_node, o_end_arc_node.GetNumber(), o_start_arc_node.GetNumber(), s_to_drive_info,
                        s_from_drive_info, f_arc_length);
            }
        }
    }
    SetMap(vec_graph);
}

NaviFra::NaviNode WaypointPlanner::GetNode(const string& node_id)
{
    NaviFra::NaviNode o_result;
    for (auto& node : vec_all_node_) {
        if (node.GetID() == node_id || node.GetName() == node_id) {
            o_result = node;
            break;
        }
    }
    return o_result;
}

NaviFra::Pos WaypointPlanner::GetNodePoseByName(const string& node_name)
{
    NaviFra::Pos o_result;
    for (auto& node : vec_all_node_) {
        if (node.GetName() == node_name) {
            o_result = node;
            break;
        }
    }
    return o_result;
}

NaviFra::Pos WaypointPlanner::GetNodePoseByID(const string& node_id)
{
    NaviFra::Pos o_result;
    for (auto& node : vec_all_node_) {
        if (node.GetID() == node_id) {
            o_result = node;
            break;
        }
    }
    return o_result;
}

std::string WaypointPlanner::GetNodeNameByNodeID(const string& node_id)
{
    std::string o_found_node_name = "";
    for (auto& node : vec_all_node_) {
        if (node.GetID() == node_id) {
            o_found_node_name = node.GetName();
            break;
        }
    }
    return o_found_node_name;
}

bool WaypointPlanner::GetOnPath(const Pos& o_robot_pos)
{
    std::vector<NaviFra::NaviNode> vec_closest_node = FindNeighboringNodeVec(o_robot_pos, 1);
    std::vector<NaviFra::NaviEdge> vec_closest_edge = FindNeighboringEdgeVec(o_robot_pos, 3);
    if (vec_closest_edge.size() > 0) {
        if (vec_closest_edge.at(0).GetDistRobot2EdgeM() < f_start_thr_dist_m_) {
            return true;
        }
    }
    if (vec_closest_node.size() > 0) {
        if (CalcPosDistance_(vec_closest_node.at(0), o_robot_pos) < 0.25) {
            return true;
        }
    }
    return false;
}

bool WaypointPlanner::GetMatchingNodes(const vector<std::string>& vec_node_id, vector<NaviFra::NaviNode>& vec_path_node_raw)
{
    for (int i = 0; i < vec_node_id.size(); ++i) {
        bool b_find = false;
        for (int j = 0; j < vec_all_node_.size(); j++) {
            if (vec_all_node_.at(j).GetID() == vec_node_id.at(i) || vec_all_node_.at(j).GetName() == vec_node_id.at(i)) {
                vec_path_node_raw.emplace_back(vec_all_node_.at(j));
                b_find = true;
                break;
            }
        }
        if (!b_find) {
            LOG_ERROR("Not found node! %s <- ", vec_node_id.at(i).c_str());
            return false;
        }
        LOG_INFO("path %s -> ", vec_node_id.at(i).c_str());
    }
    if (vec_path_node_raw.size() < 2) {
        LOG_ERROR("Path is too short");
        return false;
    }

    return true;
}

bool WaypointPlanner::GetMatchingNode(const std::string& str_node_id, NaviFra::NaviNode& o_node)
{
    bool b_find = false;
    for (auto& node : vec_all_node_) {
        if (node.GetID() == str_node_id || node.GetName() == str_node_id) {
            b_find = true;
            o_node = node;
            break;
        }
    }
    if (!b_find) {
        LOG_ERROR("Can't find goal!");
        return false;
    }

    LOG_INFO("node number : %d, node name : %s", o_node.GetNumber(), o_node.GetName().c_str());
    return true;
}

bool WaypointPlanner::RemoveDuplicateNodes(const vector<NaviFra::NaviNode>& vec_path_node_raw, vector<NaviFra::NaviNode>& vec_path_node)
{
    vec_path_node.emplace_back(vec_path_node_raw.at(0));
    for (int i = 1; i < vec_path_node_raw.size(); i++) {
        if (vec_path_node_raw.at(i - 1).GetID() != vec_path_node_raw.at(i).GetID()) {
            vec_path_node.emplace_back(vec_path_node_raw.at(i));
        }
    }
    if (vec_path_node.size() < 2) {
        LOG_ERROR("Path is too short");
        return false;
    }
    return true;
}

bool WaypointPlanner::ConnectNodePath(const vector<NaviFra::NaviNode>& vec_path_node, vector<NaviFra::NaviNode>& vec_path_node2)
{
    vec_path_node2.emplace_back(vec_path_node.at(0));

    for (size_t i = 1; i < vec_path_node.size(); i++) {
        bool b_find = false;
        NaviFra::NaviNode o_from_node = vec_path_node.at(i - 1);
        NaviFra::NaviNode o_to_node = vec_path_node.at(i);
        std::vector<NaviFra::NaviEdge> vec_from_node_edges = o_from_node.GetAllEdge();

        for (size_t j = 0; j < vec_from_node_edges.size(); j++) {
            if (vec_from_node_edges.at(j).GetEndNodeNumber() == o_to_node.GetNumber()) {
                b_find = true;
                break;
            }
        }
        if (!b_find) {
            o_dijkstra_algorithm_.FindShortestPath(o_to_node.GetNumber());
            vector<int> vec_path_num = o_dijkstra_algorithm_.GetPath(o_to_node.GetNumber(), o_from_node.GetNumber());
            if (vec_path_num.size() > 4) {
                LOG_ERROR("우회 경로로 생성됨. %d", vec_path_num.size());
                vec_path_.clear();
                return false;
            }
            std::vector<NaviFra::NaviNode> vec_dij;  // Path Planning Node Sequence
            for (int i = 0; i < vec_path_num.size(); ++i) {
                auto it = find(vec_all_node_.begin(), vec_all_node_.end(), vec_path_num.at(i));
                vec_dij.push_back(*it);
                LOG_INFO("b_find path %s <- ", (*it).GetName().c_str());
            }
            std::reverse(vec_dij.begin(), vec_dij.end());
            for (int i = 1; i < vec_dij.size(); i++) {
                LOG_INFO("b_find path %s <- ", vec_dij.at(i).GetName().c_str());
                vec_path_node2.emplace_back(vec_dij.at(i));
            }
        }
        else {
            LOG_INFO("b_find path %s <- ", vec_path_node.at(i).GetName().c_str());
            vec_path_node2.emplace_back(vec_path_node.at(i));
        }
    }

    return true;
}

bool WaypointPlanner::ConnectNodePath(
    const NaviFra::NaviNode& o_start_node, const NaviFra::NaviNode& o_goal_node, const NaviFra::NaviNode& o_edge_node,
    vector<NaviFra::NaviNode>& vec_path_node)
{
    vector<int> vec_path_num;
    if (o_goal_node.GetNumber() != o_start_node.GetNumber())
        vec_path_num = o_dijkstra_algorithm_.GetPath(o_goal_node.GetNumber(), o_start_node.GetNumber());
    else
        vec_path_num.push_back(o_goal_node.GetNumber());

    if (o_start_node.GetNumber() != o_edge_node.GetNumber())  // link path plan
    {
        if (o_dijkstra_algorithm_.CheckDist(o_edge_node.GetNumber()) > 20000.f) {
            LOG_ERROR("The cost of creating a pass is too high.!");
            return false;
        }
        vec_path_num.push_back(o_edge_node.GetNumber());
    }
    if (vec_path_num.size() == 0) {
        LOG_ERROR("vec_path_num is empty");
        return false;
    }

    for (int i = 0; i < vec_path_num.size(); ++i) {
        auto it = find(vec_all_node_.begin(), vec_all_node_.end(), vec_path_num.at(i));
        vec_path_node.push_back(*it);
        LOG_INFO("path %s <- ", (*it).GetName().c_str());
    }

    return true;
}

bool WaypointPlanner::GetNodePath(const Pos& o_robot_pos, const NaviFra::NaviNode& o_goal_node, vector<NaviFra::NaviNode>& vec_path_node)
{
    std::vector<NaviFra::NaviEdge> vec_closest_edge = FindNeighboringEdgeVec(o_robot_pos, 10);
    std::vector<NaviFra::NaviNode> vec_start = FindNeighboringNodeVec(o_robot_pos, 3);

    if (vec_start.size() == 0) {
        LOG_ERROR("Can't find start robot pos node!");
        return false;
    }

    // Find Start Pos Algorithm
    // 1. 로봇에 가장 가까운 노드와 가장 가까운 엣지를 찾는다.
    // 2. 찾은 노드와 엣지가 모두 거리차가 일정거리(1m) 이상이면 error 처리한다.
    // 3. 찾은 노드와 로봇간 거리차가 일정거리 (20cm) 이하이면 스타트 포인트가 그 노드가 된다.
    // 4. 그 외 경우 엣지가 라인인지 커브인지 확인한다.
    // 5. dist = dijkstra(dist) - edgepoint to startnode(dist) + edgepoint to robot(dist) 코스트가 적은 노드를 찾는다
    // 6. 찾은 노드에서 골 노드까지 순서를 구하고 reverse

    NaviFra::NaviNode o_start_node;
    NaviFra::NaviNode o_edge_node;
    float f_dist_to_node = CalcPosDistance_(vec_start.at(0), o_robot_pos);  // 1. 로봇에 가장 가까운 노드와 가장 가까운 엣지를 찾는다.
    float f_dist_to_edge = INF;
    float f_total_cost = INF;
    float f_dist_to_goal = CalcPosDistance_(vec_start.at(0), o_goal_node);

    NLOG(info) << "vec_closest_edge size : " << vec_closest_edge.size();
    if (vec_closest_edge.size() > 0)
        f_dist_to_edge = vec_closest_edge.at(0).GetDistRobot2EdgeM();
    LOG_INFO("f_dist_to_node %.3f f_dist_to_edge %.3f f_start_thr_dist_m_ %.3f ", f_dist_to_node, f_dist_to_edge, f_start_thr_dist_m_);

    float f_deg_gap = CalcAngleDomainDeg_(vec_start.at(0).GetDeg() - o_robot_pos.GetDeg());

    if (f_dist_to_node > f_start_thr_dist_m_ &&
        f_dist_to_edge > f_start_thr_dist_m_)  // 2. 찾은 노드와 엣지가 모두 거리차가 일정거리(1m) 이상이면 error 처리한다.
    {
        LOG_ERROR("No nodes or edges around the robot..");
        return false;
    }
    else if (
        f_dist_to_node < f_start_thr_dist_m_ && f_dist_to_goal > f_start_thr_dist_m_ &&
        ((fabs(f_dist_to_node - f_dist_to_edge) < 0.05 && f_dist_to_edge > 0.05) || f_dist_to_edge > f_start_thr_dist_m_ ||
         f_dist_to_node < 0.05 ||
         fabs(f_deg_gap) < 5))  // 3. 찾은 노드와 로봇간 거리차가 일정거리 (20cm) 이하이면 스타트 포인트가 그 노드가 된다.
    {
        LOG_INFO("Find the nearest node... %s", vec_start.at(0).GetName().c_str());
        o_start_node = vec_start.at(0);
        o_edge_node = vec_start.at(0);
        f_total_cost = f_dist_to_node + o_dijkstra_algorithm_.CheckDist(vec_start.at(0).GetNumber());

        NLOG(info) << "vec_closest_edge : " << vec_closest_edge.size();

        if (vec_start.size() > 1 && vec_closest_edge.size() > 0) {
            for (int i = 1; i < vec_start.size(); i++) {
                float f_dist_to_node = CalcPosDistance_(vec_start.at(i), o_robot_pos);
                // float f_dist_to_edge = vec_closest_edge.at(0).GetDistRobot2EdgeM();
                float f_dist_to_edge = INF;
                if (vec_closest_edge.size() > 0)
                    f_dist_to_edge = vec_closest_edge.at(0).GetDistRobot2EdgeM();
                float f_deg_gap = CalcAngleDomainDeg_(vec_start.at(i).GetDeg() - o_robot_pos.GetDeg());
                if (f_dist_to_node < f_start_thr_dist_m_ && f_dist_to_goal > f_start_thr_dist_m_ &&
                    ((fabs(f_dist_to_node - f_dist_to_edge) < 0.05 && f_dist_to_edge > 0.05) || f_dist_to_edge > f_start_thr_dist_m_ ||
                     f_dist_to_node < 0.05 ||
                     fabs(f_deg_gap) < 5))  // 3. 찾은 노드와 로봇간 거리차가 일정거리 (20cm) 이하이면 스타트 포인트가 그 노드가 된다.
                {
                    LOG_INFO("Find the nearest node... %s", vec_start.at(i).GetName().c_str());
                    float f_tmp_total_cost = f_dist_to_node + o_dijkstra_algorithm_.CheckDist(vec_start.at(i).GetNumber());
                    if (f_tmp_total_cost < f_total_cost) {
                        NLOG(info) << "find another node " << vec_start.at(i).GetName();
                        f_total_cost = f_tmp_total_cost;
                        o_start_node = vec_start.at(i);
                        o_edge_node = vec_start.at(i);
                    }
                }
            }
        }
    }
    else if (f_dist_to_node < 0.01) {
        LOG_INFO("Find the nearest node... %s", vec_start.at(0).GetName().c_str());
        o_start_node = vec_start.at(0);
        o_edge_node = vec_start.at(0);
        f_total_cost = f_dist_to_node + o_dijkstra_algorithm_.CheckDist(vec_start.at(0).GetNumber());
    }
    else  // Node is far from robot pose and egde is close. make multi edge path
    {
        LOG_INFO("find close edge or far node");
        for (int j = 0; j < vec_closest_edge.size(); j++) {
            LOG_INFO("%d vec_closest_edge dist %.3f", j, vec_closest_edge.at(j).GetDistRobot2EdgeM());
            auto start_node = find(vec_all_node_.begin(), vec_all_node_.end(), vec_closest_edge.at(j).GetStartNodeNumber());
            auto end_node = find(vec_all_node_.begin(), vec_all_node_.end(), vec_closest_edge.at(j).GetEndNodeNumber());
            LOG_INFO("find start %s end %s ", (*start_node).GetName().c_str(), (*end_node).GetName().c_str());
            NLOG(info) << "f_start_thr_dist_m_ : " << f_start_thr_dist_m_;
            if (vec_closest_edge.at(j).GetDistRobot2EdgeM() < f_start_thr_dist_m_) {
                float f_distance_m = INF;
                if (vec_closest_edge.at(j).GetToDriveInfo().e_curve_type == NaviFra::Pos::LINE_TYPE::LINE)  //엣지가 Line
                {
                    NaviFra::Pos o_pos_on_edge =
                        CalcPosDotOnLine_(vec_closest_edge.at(j).GetStartPos(), vec_closest_edge.at(j).GetEndPos(), o_robot_pos);
                    // 2를 곱해준 이유는 로봇과 엣지랑 거리가 작을 수록 주행에 유리하기 때문에 먼저 탐색하도록 수정
                    LOG_INFO(
                        "LINE %.3f %.3f %.3f", o_dijkstra_algorithm_.CheckDist((*end_node).GetNumber()),
                        CalcPosDistance_(o_pos_on_edge, (*end_node)), vec_closest_edge.at(j).GetDistRobot2EdgeM() * 2);

                    f_distance_m = o_dijkstra_algorithm_.CheckDist((*end_node).GetNumber()) + CalcPosDistance_(o_pos_on_edge, (*end_node)) +
                        vec_closest_edge.at(j).GetDistRobot2EdgeM() *
                            2;  // dist = dijkstra(dist) - edgepoint to end_node(dist) + edgepoint to robot(dist)
                }
                else if (vec_closest_edge.at(j).GetToDriveInfo().e_curve_type == NaviFra::Pos::LINE_TYPE::CURVE)  //엣지가 Curve
                {
                    NaviFra::Pos o_circle_pos(
                        vec_closest_edge.at(j).GetToDriveInfo().f_circle_pos_x, vec_closest_edge.at(j).GetToDriveInfo().f_circle_pos_y);
                    float f_f_circle_angle = CalcAngleDifferenceRad_((*end_node), o_circle_pos, o_robot_pos);
                    float f_radius = CalcPosDistance_((*end_node), o_circle_pos);
                    float f_arc_length = fabs(f_radius * f_f_circle_angle);  // find arc dist from robot to end node.

                    LOG_INFO(
                        "CURVE %.3f %.3f %.3f", o_dijkstra_algorithm_.CheckDist((*end_node).GetNumber()), f_arc_length,
                        vec_closest_edge.at(j).GetDistRobot2EdgeM() * 2);

                    f_distance_m = o_dijkstra_algorithm_.CheckDist((*end_node).GetNumber()) + f_arc_length +
                        vec_closest_edge.at(j).GetDistRobot2EdgeM() *
                            2;  // dist = dijkstra(dist) - edgepoint to startnode(arc dist) + edgepoint to robot(dist)
                }

                vector<int> vec_path_num = o_dijkstra_algorithm_.GetPath(o_goal_node.GetNumber(), (*end_node).GetNumber());
                bool b_possible_path = false;

                if (o_goal_node.GetNumber() == (*end_node).GetNumber()) {
                    b_possible_path = true;
                    vec_path_num.clear();
                    vec_path_num.emplace_back(o_goal_node.GetNumber());
                }

                // for debug
                NLOG(info) << "find start " << (*start_node).GetName() << " end " << (*end_node).GetName();

                string s_path = "";
                for (auto num : vec_path_num) {
                    auto it = find(vec_all_node_.begin(), vec_all_node_.end(), num);
                    NLOG(info) << (*it).GetName() << " <- ";
                    s_path += (*it).GetName();
                    s_path += " <- ";
                }
                LOG_INFO("path %s ", s_path.c_str());

                if (vec_path_num.size() > 1)  // 경로가 시작 노드를 지나는지 확인만 하면됨
                {
                    NLOG(info) << "vec_path_num[vec_path_num.size() - 2] " << vec_path_num.at(vec_path_num.size() - 2)
                               << " (*start_node).GetNumber() " << (*start_node).GetNumber();
                    if (vec_path_num.at(vec_path_num.size() - 2) != (*start_node).GetNumber())  // 시작 노드를 안지난다면 가능한 경로
                    {
                        b_possible_path = true;
                    }
                    else if (vec_path_num.size() == 3) {
                        if (vec_path_num.at(vec_path_num.size() - 1) == vec_path_num.at(vec_path_num.size() - 3)) {
                            b_possible_path = true;
                            vec_path_num.pop_back();
                        }
                    }
                    else if (vec_path_num.size() == 2) {
                        if (vec_path_num.at(vec_path_num.size() - 1) == vec_path_num.at(vec_path_num.size() - 2)) {
                            b_possible_path = true;
                            vec_path_num.pop_back();
                        }
                    }
                }

                NLOG(info) << "b_possible_path " << b_possible_path;
                if (f_total_cost > f_distance_m && b_possible_path) {
                    f_total_cost = f_distance_m;
                    o_start_node = (*end_node);
                    o_edge_node = (*start_node);
                }
            }
        }

        if (f_total_cost > 20000 && f_dist_to_node < f_start_thr_dist_m_) {
            LOG_INFO("Find the nearest node...");
            o_start_node = vec_start.at(0);
            o_edge_node = vec_start.at(0);
            f_total_cost = f_dist_to_node + o_dijkstra_algorithm_.CheckDist(vec_start.at(0).GetNumber());
        }
    }
    LOG_INFO("start node number : %d, start node name : %s", o_start_node.GetNumber(), o_start_node.GetName().c_str());

    if (f_total_cost > 20000.f || f_total_cost < 0) {
        LOG_ERROR("The cost of creating a pass is too high.! cost : %lf", f_total_cost);
        return false;
    }

    if (!ConnectNodePath(o_start_node, o_goal_node, o_edge_node, vec_path_node)) {
        return false;
    }

    std::reverse(vec_path_node.begin(), vec_path_node.end());

    return true;
}

std::vector<NaviFra::NaviNode> WaypointPlanner::PlanPath(const vector<std::string>& vec_node_id)
{
    std::vector<NaviFra::NaviNode> vec_path;
    // node id들 중 map의 node와 일치하는 node 들만 반환
    std::vector<NaviFra::NaviNode> vec_path_node_raw;
    if (!GetMatchingNodes(vec_node_id, vec_path_node_raw)) {
        return vec_path;
    }

    // node들 중 연속으로 중복되는 node 제거
    std::vector<NaviFra::NaviNode> vec_path_node;
    if (!RemoveDuplicateNodes(vec_path_node_raw, vec_path_node)) {
        return vec_path;
    }

    // 다음 노드와 연결되지 않는 노드는 가장 가까운 경로를 찾아서 연결된 노드들을 반환
    std::vector<NaviFra::NaviNode> vec_path_node2;
    if (!ConnectNodePath(vec_path_node, vec_path_node2)) {
        return vec_path;
    }

    return vec_path_node2;
}

std::vector<NaviFra::NaviNode> WaypointPlanner::PlanPath(const Pos& o_robot_pos, const std::string& str_node_id)
{
    std::vector<NaviFra::NaviNode> vec_path;
    LOG_INFO("robot pos x : %f, y : %f, goal id : %s", o_robot_pos.GetXm(), o_robot_pos.GetYm(), str_node_id.c_str());

    // str_node_id와 일치하는 map의 node를 반환
    NaviFra::NaviNode o_goal_node;
    if (!GetMatchingNode(str_node_id, o_goal_node)) {
        return vec_path;
    }

    o_dijkstra_algorithm_.FindShortestPath(o_goal_node.GetNumber());

    // Find Start Pos Algorithm && 연결된 노드 리스트 저장
    std::vector<NaviFra::NaviNode> vec_path_node;  // Path Planning Node Sequence
    if (!GetNodePath(o_robot_pos, o_goal_node, vec_path_node)) {
        return vec_path;
    }

    return vec_path_node;
}

std::vector<NaviFra::NaviEdge> WaypointPlanner::FindNeighboringEdgeVec(const NaviFra::Pos& o_robot_pos, int n_find_num)
{
    std::vector<pair<float, int>> vec_edge_sort;
    std::vector<NaviEdge> result;
    for (int i = 0; i < vec_edge_data_.size(); i++) {
        Pos o_start_pos = vec_edge_data_.at(i).GetStartPos();
        Pos o_end_pos = vec_edge_data_.at(i).GetEndPos();

        float f_inc = -(o_end_pos.GetXm() - o_start_pos.GetXm()) / (o_end_pos.GetYm() - o_start_pos.GetYm());
        float f_start_y = f_inc * (o_robot_pos.GetXm() - o_start_pos.GetXm()) + o_start_pos.GetYm();
        float f_end_y = f_inc * (o_robot_pos.GetXm() - o_end_pos.GetXm()) + o_end_pos.GetYm();

        float f_start_x = (o_robot_pos.GetYm() - o_start_pos.GetYm()) / f_inc + o_start_pos.GetXm();
        float f_end_x = (o_robot_pos.GetYm() - o_end_pos.GetYm()) / f_inc + o_end_pos.GetXm();

        if ((o_robot_pos.GetYm() <= f_end_y && o_robot_pos.GetYm() >= f_start_y) ||
            (o_robot_pos.GetYm() >= f_end_y && o_robot_pos.GetYm() <= f_start_y) ||
            std::min(fabs(o_robot_pos.GetYm() - f_end_y), fabs(o_robot_pos.GetYm() - f_start_y)) <
                0.03)  // 두 점 사이에 로봇 위치가 있지 않으면 제외
        {
            if (vec_edge_data_.at(i).GetFromDriveInfo().e_curve_type == NaviFra::Pos::LINE_TYPE::LINE) {
                vec_edge_sort.push_back({fabs(CalcDistanceFromDotToLine_(o_start_pos, o_end_pos, o_robot_pos)), i});
            }
        }
        else if (
            (o_robot_pos.GetXm() <= f_end_x && o_robot_pos.GetXm() >= f_start_x) ||
            (o_robot_pos.GetXm() >= f_end_x && o_robot_pos.GetXm() <= f_start_x) ||
            std::min(fabs(o_robot_pos.GetXm() - f_end_x), fabs(o_robot_pos.GetXm() - f_start_x)) < 0.03) {
            if (vec_edge_data_.at(i).GetFromDriveInfo().e_curve_type == NaviFra::Pos::LINE_TYPE::LINE) {
                vec_edge_sort.push_back({fabs(CalcDistanceFromDotToLine_(o_start_pos, o_end_pos, o_robot_pos)), i});
            }
        }

        if (vec_edge_data_.at(i).GetFromDriveInfo().e_curve_type == NaviFra::Pos::LINE_TYPE::CURVE) {
            Pos o_circle_pos(
                vec_edge_data_.at(i).GetFromDriveInfo().f_circle_pos_x, vec_edge_data_.at(i).GetFromDriveInfo().f_circle_pos_y);
            float f_radius = CalcPosDistance_(o_start_pos, o_circle_pos);
            float f_dist = CalcPosDistance_(o_robot_pos, o_circle_pos);

            auto start_node = find(vec_all_node_.begin(), vec_all_node_.end(), vec_edge_data_.at(i).GetStartNodeNumber());
            auto end_node = find(vec_all_node_.begin(), vec_all_node_.end(), vec_edge_data_.at(i).GetEndNodeNumber());

            // -180 ~ 180 리턴
            float f_start_angle =
                atan2((*start_node).GetYm() - o_circle_pos.GetYm(), (*start_node).GetXm() - o_circle_pos.GetXm()) * RADtoDEG;
            float f_end_angle = f_start_angle + vec_edge_data_.at(i).GetFromDriveInfo().f_circle_angle;

            float f_robot_angle = atan2(o_robot_pos.GetYm() - o_circle_pos.GetYm(), o_robot_pos.GetXm() - o_circle_pos.GetXm()) * RADtoDEG;

            float f_small, f_large;
            if (vec_edge_data_.at(i).GetFromDriveInfo().f_circle_angle >= 0) {
                f_small = f_start_angle;
                f_large = f_end_angle;
            }
            else {
                f_small = f_end_angle;
                f_large = f_start_angle;
            }

            if ((f_small <= f_robot_angle && f_robot_angle <= f_large) ||
                (f_small <= f_robot_angle + 360 && f_robot_angle + 360 <= f_large) ||
                (f_small <= f_robot_angle - 360 && f_robot_angle - 360 <= f_large)) {
                vec_edge_sort.push_back({fabs(f_radius - f_dist), i});
            }
            else {
                float f_min_dist = std::min(CalcPosDistance_(o_start_pos, o_robot_pos), CalcPosDistance_(o_end_pos, o_robot_pos));
                vec_edge_sort.push_back({f_min_dist, i});
            }
        }
    }
    std::sort(vec_edge_sort.begin(), vec_edge_sort.end());
    for (int i = 0; i < vec_edge_sort.size(); i++) {
        vec_edge_data_.at(vec_edge_sort.at(i).second)
            .SetDistRobot2EdgeM(vec_edge_sort.at(i).first);  // dist from robot and edge dist for choice path starting point
        result.emplace_back(vec_edge_data_.at(vec_edge_sort.at(i).second));

        if (result.size() >= n_find_num)
            break;
    }

    return result;
}

std::vector<NaviFra::NaviNode> WaypointPlanner::FindNeighboringNodeVec(const NaviFra::Pos& o_robot_pos, int n_find_num)
{
    std::vector<NaviNode> vec_all_node;
    for (int j = 0; j < vec_all_node_.size(); j++) {
        if (vec_all_node_.at(j).GetAllEdge().size() > 0 && vec_all_node_.at(j).GetName().substr(0, 3) != "ARC") {
            vec_all_node_.at(j).SetDistM(
                hypot(vec_all_node_.at(j).GetXm() - o_robot_pos.GetXm(), vec_all_node_.at(j).GetYm() - o_robot_pos.GetYm()));
            vec_all_node.push_back(vec_all_node_.at(j));
        }
    }
    sort(vec_all_node.begin(), vec_all_node.end());
    std::vector<NaviFra::NaviNode> vec_sorted_nodes;

    if (vec_all_node.empty())
        return vec_sorted_nodes;
    for (int i = 0; i < vec_all_node.size(); i++) {
        vec_sorted_nodes.emplace_back(vec_all_node.at(i));
        if (vec_sorted_nodes.size() >= n_find_num)
            return vec_sorted_nodes;
    }
    return vec_sorted_nodes;
}

void WaypointPlanner::MakeGraph(const NaviFra::NaviNode& o_navi_node)
{
    std::vector<NaviFra::NaviEdge> edges = o_navi_node.GetAllEdge();

    for (int i = 0; i < edges.size(); ++i) {
        auto end_node = find(vec_all_node_.begin(), vec_all_node_.end(), edges.at(i).GetEndNodeNumber());
        if (end_node == vec_all_node_.end()) {
            LOG_ERROR("%d, y : %d find end node FAILED", o_navi_node.GetNumber(), edges.at(i).GetEndNodeNumber());
            return;
        }
        if (end_node->GetID() == "") {
            LOG_ERROR("%d, y : %d no named end node", o_navi_node.GetNumber(), edges.at(i).GetEndNodeNumber());
            return;
        }
        if (end_node->GetID() == o_navi_node.GetID()) {
            continue;
        }
        LOG_DEBUG("%s -> %s, weight : %lf", o_navi_node.GetName().c_str(), end_node->GetName().c_str(), edges.at(i).GetDistM());
        o_dijkstra_algorithm_.AddEdge(o_navi_node.GetNumber(), end_node->GetNumber(), edges.at(i).GetDistM());
        o_dijkstra_algorithm_.SetName(o_navi_node.GetNumber(), o_navi_node.GetName());
        o_dijkstra_algorithm_.SetName(end_node->GetNumber(), end_node->GetName());
    }
}

float WaypointPlanner::CalcAngleDifferenceRad_(const NaviFra::Pos& o_pos, const NaviFra::Pos& o_pos2, const NaviFra::Pos& o_pos3)
{
    NaviFra::Pos o_vector21 = o_pos - o_pos2;
    float f_dist1 = hypot(o_vector21.GetXm(), o_vector21.GetYm());
    if (f_dist1 == 0)
        return 0;
    float f_rad1 = atan2(o_vector21.GetYm(), o_vector21.GetXm());

    NaviFra::Pos o_vector23 = o_pos3 - o_pos2;
    float f_dist2 = hypot(o_vector23.GetXm(), o_vector23.GetYm());
    if (f_dist2 == 0)
        return 0;
    float f_rad2 = atan2(o_vector23.GetYm(), o_vector23.GetXm());

    float f_angle_rad = f_rad1 - f_rad2;

    int n_sign = (f_angle_rad > 0) - (f_angle_rad < 0);

    float f_rad = fmod((f_angle_rad + n_sign * M_PI), 2.0 * M_PI);
    return f_rad - n_sign * M_PI;
}

float WaypointPlanner::CalcPosDistance_(const NaviFra::Pos& o_pos1, const NaviFra::Pos& o_pos2)
{
    return hypot(o_pos1.GetXm() - o_pos2.GetXm(), o_pos1.GetYm() - o_pos2.GetYm());
}

int WaypointPlanner::sign(float val)
{
    return (0 < val) - (val < 0);
}

float WaypointPlanner::CalcAngleDomainDeg_(float f_angle_deg)
{
    return fmod((f_angle_deg + sign(f_angle_deg) * 180.0), 360.0) - sign(f_angle_deg) * 180.0;
}

Pos WaypointPlanner::TransformRotationRad_(const Pos& o_pos, float f_rad)
{
    float f_rot_x = o_pos.GetXm() * cos(f_rad) - o_pos.GetYm() * sin(f_rad);
    float f_rot_y = o_pos.GetXm() * sin(f_rad) + o_pos.GetYm() * cos(f_rad);
    return Pos(f_rot_x, f_rot_y);
}

Pos WaypointPlanner::TransformRotationDeg_(const Pos& o_pos, float f_deg)
{
    return TransformRotationRad_(o_pos, f_deg * DEGtoRAD);
}

float WaypointPlanner::CalcDistanceFromDotToLine_(const Pos& o_line_s, const Pos& o_line_e, const Pos& o_target_pos)
{
    Pos o_line_vec = o_line_e - o_line_s;
    Pos o_sub_vec = o_target_pos - o_line_s;
    float o_line_dist = hypot(o_line_vec.GetXm(), o_line_vec.GetYm());
    if (o_line_dist > 0)
        return fabs((o_line_vec.GetXm() * o_sub_vec.GetYm() - o_line_vec.GetYm() * o_sub_vec.GetXm()) / o_line_dist);
    else
        return 0;
}

Pos WaypointPlanner::CalcPosDotOnLine_(const Pos& o_line_s, const Pos& o_line_e, const Pos& o_target_pos)
{
    float f_distance_m = CalcPosDistance_(o_line_e, o_line_s);
    if (f_distance_m < FLT_EPSILON)
        return Pos(0, 0, 0);

    Pos o_unit_vector = (o_line_e - o_line_s) / f_distance_m;
    Pos o_diff_pos = o_target_pos - o_line_s;
    Pos o_add_pos = o_unit_vector * (o_unit_vector.GetXm() * o_diff_pos.GetXm() + o_unit_vector.GetYm() * o_diff_pos.GetYm());
    Pos o_result = o_line_s + o_add_pos;
    return o_result;
}

}  // namespace NaviFra
