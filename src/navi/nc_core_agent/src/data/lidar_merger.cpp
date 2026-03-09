#include "core_agent/core_agent.h"

#include <core_agent/data/lidar_merger.h>

using namespace NaviFra;

const std::string LidarMerger::KEY = "LidarMerger";

LidarMerger::LidarMerger()
{
}

LidarMerger::~LidarMerger()
{
    std::lock_guard<std::mutex> lock(mutex_);
    lidar_.clear();
}

void LidarMerger::update(size_t index, const sensor_msgs::PointCloud::ConstPtr& data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lidar_[index] = data;
}

std::string LidarMerger::toString()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::string result = "[";
    bool first_cloud = true;

    for (const auto& entry : lidar_) {
        if (!first_cloud) {
            result.append(",");
        }
        result.append(pointCloudToString(entry.second));
        first_cloud = false;
    }

    result.append("]");
    return result;
}

std::string LidarMerger::pointCloudToString(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    std::string val = "[";
    bool first_point = true;
    size_t count = 0;

    for (const auto& point : msg->points) {
        if (!first_point) {
            val.append(",");
        }
        std::string point_str;
        Poco::format(point_str, "[%.3hf,%.3hf,0]", point.x, point.y);
        val.append(point_str);
        first_point = false;
        count++;
    }

    val.append("]");
    return val;
}