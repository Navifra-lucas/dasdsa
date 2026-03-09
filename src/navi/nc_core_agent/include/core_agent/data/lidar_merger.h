#ifndef NAVIFRA_LIDAR_MERGER_H
#define NAVIFRA_LIDAR_MERGER_H

#include <sensor_msgs/PointCloud.h>

#include <map>
#include <mutex>

namespace NaviFra {
enum LIDAR
{
    LIDAR_FRONT = 0,
    LIDAR_REAR,
    LIDAR_LEFT,
    LIDAR_RIGHT,
    LIDAR_V2V,
    LIDAR_CAMERA,
    LIDAR_OBS
};

class LidarMerger {
public:
    const static std::string KEY;

    LidarMerger();
    ~LidarMerger();

    void update(size_t index, const sensor_msgs::PointCloud::ConstPtr& data);
    std::string toString();

private:
    std::string pointCloudToString(const sensor_msgs::PointCloud::ConstPtr& msg);

private:
    std::map<size_t, sensor_msgs::PointCloud::ConstPtr> lidar_;
    std::mutex mutex_;
};

}  // namespace NaviFra
#endif  // NAVIFRA_LIDAR_MERGER_H