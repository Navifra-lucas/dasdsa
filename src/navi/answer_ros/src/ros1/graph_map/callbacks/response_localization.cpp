#include "ros1/graph_map/graph_map_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

bool GraphMapRos1::ResponseLocalization(
    answer_msgs::GraphLocalization::Request  &req,
    answer_msgs::GraphLocalization::Response &res)
{
    LOG_INFO("[ResponseLocalization] data: {}", req.data.c_str());

    auto closest_node = o_graph_map_ptr_->FindClosestNode();

    if (closest_node.b_find)
    {
        res.success = true;
        res.reason = "";
        res.on_edge = closest_node.b_on_edge;
        res.node1 = MsgConverter::Convert<
            NodeInfo, answer_msgs::NodeInfo>(
                closest_node.o_node1);
        res.node2 = MsgConverter::Convert<
            NodeInfo, answer_msgs::NodeInfo>(
                closest_node.o_node2);
        res.edge_id = closest_node.s_edge_id;
        LOG_INFO("[ResponseLocalization] Success to closest node\n  on_edge: {}\n  node1: {}\n  node2: {}",
            closest_node.b_on_edge ? "true" : "false",
            closest_node.o_node1.toStr().c_str(),
            closest_node.o_node2.toStr().c_str(),
            closest_node.s_edge_id.c_str());
    }
    else
    {
        res.success = false;
        res.reason = "fail to find closest node";
        LOG_ERROR("[ResponseLocalization] Fail to find closest node");
    }

    return res.success;

}

} // namespace NVFR
