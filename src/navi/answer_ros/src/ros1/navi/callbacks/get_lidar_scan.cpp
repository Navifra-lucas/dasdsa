#include "ros1/navi/navigator_ros1.hpp"

#include "logger/logger.h"

namespace NVFR {

void NavigatorRos1::GetLidarScan(const sensor_msgs::LaserScan::ConstPtr msg)
{
    auto cloud = MsgConverter::Convert<
        sensor_msgs::LaserScan,
        pcl::PointCloud<pcl::PointXYZ>::Ptr>(
            *msg);

    if (o_navigator_ptr_->SetLocalSensorPoints(cloud)) {
        NVFR::GridMap::Data_t st_local_map =
            o_navigator_ptr_->GetLocalMap();
        DebugVisualizer::GetInstance()->PublishLocalMap(
            "local_map",
            *st_local_map.grid_map_ptr,
            st_local_map.st_map_info);
    }
}

void NavigatorRos1::GetPointCloud2D(const sensor_msgs::PointCloud::ConstPtr msg)
{
    auto cloud = MsgConverter::Convert<
        sensor_msgs::PointCloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr>(
            *msg);

    if (o_navigator_ptr_->SetLocalSensorPoints(cloud)) {
        NVFR::GridMap::Data_t st_local_map =
            o_navigator_ptr_->GetLocalMap();
        DebugVisualizer::GetInstance()->PublishLocalMap(
            "local_map",
            *st_local_map.grid_map_ptr,
            st_local_map.st_map_info);
    }
}

} // namespace NVFR
