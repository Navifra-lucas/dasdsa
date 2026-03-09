#include "ros1/navi/navigator_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void NavigatorRos1::GetGlobalMap(
    const nav_msgs::OccupancyGrid& msg)
{
    if (!o_navigator_ptr_->UpdateGlobalMap(msg.data,
            MsgConverter::Convert<nav_msgs::MapMetaData, MapInfo_t>(
                msg.info)
        )) return;
    LOG_INFO("[GlobalMap] (width,height,resolution,size,origin_x,origin_y) : ({},{},{:.3f},{},{:.3f},{:.3f})",
        msg.info.width, msg.info.height, msg.info.resolution, msg.data.size(),
        msg.info.origin.position.x, msg.info.origin.position.y);
}

//void NavigatorRos1::GetGlobalMap(
//    const nav_msgs::OccupancyGrid::ConstPtr msg)
//{
//    if (!o_navigator_ptr_->UpdateGlobalMapInfo(
//        MsgConverter::Convert<nav_msgs::MapMetaData, MapInfo_t>(
//            msg->info))) return;
//    if (!o_navigator_ptr_->UpdateGlobalMap(
//        msg->data)) return;
//    LOG_INFO("[GlobalMap] (width,height,resolution,size,origin_x,origin_y) : ({},{},{:.3f},{},{:.3f},{:.3f})",
//        msg->info.width, msg->info.height, msg->info.resolution, msg->data.size(),
//        msg->info.origin.position.x, msg->info.origin.position.y);
//}

} // namespace NVFR
