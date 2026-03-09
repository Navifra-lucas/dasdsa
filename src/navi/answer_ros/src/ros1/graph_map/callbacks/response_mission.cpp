#include "ros1/graph_map/graph_map_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

bool GraphMapRos1::ResponseMissionGraphNode(
    answer_msgs::MissionGraphNode::Request  &req,
    answer_msgs::MissionGraphNode::Response &res)
{
    LOG_INFO("uuid: {}, running: {}, start_id: {}, goal_id: {}",
        req.uuid.c_str(),
        req.running ? "true" : "false",
        req.start_id.c_str(),
        req.goal_id.c_str());

    std::vector<GraphEdge> vec_edges;
    if (req.running)
    {
        vec_edges = o_graph_map_ptr_->PlanPath(req.start_id, "", req.goal_id, "");
    }
    else
    {
        vec_edges = o_graph_map_ptr_->PlanPath(req.goal_id, "");
    }

    // exceptions
    if (vec_edges.empty())
    {
        answer_msgs::MissionResponse msg;
        msg.uuid = req.uuid;
        msg.b_received = false;
        publishers_[KEY::TOPICS::NAVI_MISSION_RESPONSE]->publish(msg);

        res.success = false;
        res.reason = "fail to make edges";
        LOG_ERROR("Fail to make edges");
    }
    else
    {
        answer_msgs::MissionNodePath msg;
        // uuid
        msg.uuid = req.uuid;
        // arrive align
        auto o_goal_node = o_graph_map_ptr_->GetNode(req.goal_id, "");
        msg.b_arrive_align = o_goal_node.GetArriveAlign();
        msg.o_arrive_align.n_align_direction = answer_msgs::MissionAlign::AUTO;
        msg.o_arrive_align.d_align_target_deg = o_goal_node.GetConstPose().GetDeg();
        // edge array
        msg.edge_array.reserve(vec_edges.size());
        for (const auto& edge : vec_edges)
        {
            answer_msgs::EdgeInfo edge_info;
            edge_info.o_drive_info = MsgConverter::Convert<
                DriveInfo, answer_msgs::DriveInfo>(
                    edge.GetDriveInfo());
            edge_info.o_from_node = MsgConverter::Convert<
                NodeInfo, answer_msgs::NodeInfo>(
                    edge.GetConstFromNode());
            edge_info.o_to_node = MsgConverter::Convert<
                NodeInfo, answer_msgs::NodeInfo>(
                    edge.GetConstToNode());
            edge_info.n_path_type = static_cast<int>(edge.GetPathType());
            edge_info.o_from_node_control = MsgConverter::Convert<
                Pose, geometry_msgs::Pose2D>(
                    edge.GetFromNodeControl());
            edge_info.o_to_node_control = MsgConverter::Convert<
                Pose, geometry_msgs::Pose2D>(
                    edge.GetToNodeControl());
            edge_info.d_time_control = edge.GetTimeControl();
            edge_info.d_target_speed = edge.GetTargetSpd();
            edge_info.d_robot_yaw = edge.GetRobotYaw();
            msg.edge_array.emplace_back(edge_info);
        }
        publishers_[KEY::TOPICS::NAVI_MISSION_NODE_PATH]->publish(msg);

        res.success = true;
        res.reason = "";
        LOG_INFO("Success to make edges, send mission");
    }

    return res.success;
}

bool GraphMapRos1::ResponseMissionGraphNodeList(
    answer_msgs::MissionGraphNodeList::Request  &req,
    answer_msgs::MissionGraphNodeList::Response &res)
{
    LOG_INFO("uuid: {}, size of node_list: {}",
        req.uuid.c_str(),
        req.node_list.size());

    int n_size = static_cast<int>(req.node_list.size());

    // exceptions
    if (n_size < 2)
    {
        res.success = false;
        res.reason = "size of node_list is small";
        return false;
    }

    std::string s_node_list = req.node_list[0];
    for (int i=0; i < n_size; ++i)
    {
        s_node_list += " -> " + req.node_list[i];
    }
    LOG_INFO("node_list : {}", s_node_list.c_str());

    std::vector<GraphEdge> vec_edges;
    vec_edges.reserve(n_size);
    for (int i=1; i < n_size; ++i)
    {
        std::vector<GraphEdge> vec_edge_segement =
            o_graph_map_ptr_->PlanPath(
                req.node_list[i-1], "",
                req.node_list[i], "");
        vec_edges.insert(vec_edges.end(),
            vec_edge_segement.begin(),
            vec_edge_segement.end());
    }

    // exceptions
    if (vec_edges.empty())
    {
        answer_msgs::MissionResponse msg;
        msg.uuid = req.uuid;
        msg.b_received = false;
        publishers_[KEY::TOPICS::NAVI_MISSION_RESPONSE]->publish(msg);

        res.success = false;
        res.reason = "fail to make edges";
        LOG_ERROR("Fail to make edges");
    }
    else
    {
        answer_msgs::MissionNodePath msg;
        // uuid
        msg.uuid = req.uuid;
        // arrive align
        auto o_goal_node = o_graph_map_ptr_->GetNode(req.node_list.back(), "");
        msg.b_arrive_align = o_goal_node.GetArriveAlign();
        msg.o_arrive_align.n_align_direction = answer_msgs::MissionAlign::AUTO;
        msg.o_arrive_align.d_align_target_deg = o_goal_node.GetConstPose().GetDeg();
        // edge array
        msg.edge_array.reserve(vec_edges.size());
        for (const auto& edge : vec_edges)
        {
            answer_msgs::EdgeInfo edge_info;
            edge_info.o_drive_info = MsgConverter::Convert<
                DriveInfo, answer_msgs::DriveInfo>(
                    edge.GetDriveInfo());
            edge_info.o_from_node = MsgConverter::Convert<
                NodeInfo, answer_msgs::NodeInfo>(
                    edge.GetConstFromNode());
            edge_info.o_to_node = MsgConverter::Convert<
                NodeInfo, answer_msgs::NodeInfo>(
                    edge.GetConstToNode());
            edge_info.n_path_type = static_cast<int>(edge.GetPathType());
            edge_info.o_from_node_control = MsgConverter::Convert<
                Pose, geometry_msgs::Pose2D>(
                    edge.GetFromNodeControl());
            edge_info.o_to_node_control = MsgConverter::Convert<
                Pose, geometry_msgs::Pose2D>(
                    edge.GetToNodeControl());
            edge_info.d_time_control = edge.GetTimeControl();
            edge_info.d_target_speed = edge.GetTargetSpd();
            edge_info.d_robot_yaw = edge.GetRobotYaw();
            msg.edge_array.emplace_back(edge_info);
        }
        publishers_[KEY::TOPICS::NAVI_MISSION_NODE_PATH]->publish(msg);

        res.success = true;
        res.reason = "";
        LOG_INFO("Success to make edges, send mission");
    }

    return res.success;
}

} // namespace NVFR
