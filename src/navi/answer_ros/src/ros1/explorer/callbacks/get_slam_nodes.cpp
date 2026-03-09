#include "ros1/explorer/explorer_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void ExplorerRos1::GetSlamNodes(
    const geometry_msgs::Polygon::ConstPtr msg)
{
    // quat to euler
    Polygon slam_nodes =
        MsgConverter::Convert<std::vector<geometry_msgs::Point32>, Polygon>(
            msg->points);

    o_explorer_ptr_->SetSlamNodes(slam_nodes);
}

} // namespace NVFR
