#ifndef LIDAR_STATIC_CHECK_HPP
#define LIDAR_STATIC_CHECK_HPP

#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PolygonStamped.h"
#include "polygon/polygon.hpp"
#include "pos/pos.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "util/logger.hpp"

#include <memory>
#include <mutex>
#include <vector>

namespace NaviFra {

class LidarStaticCheck {
public:
    LidarStaticCheck();
    virtual ~LidarStaticCheck();

    void SetPolygon(const std::vector<NaviFra::Polygon>& vec_polygon);
    bool CheckObstacle(const sensor_msgs::PointCloud::ConstPtr& cloud_msg);
    geometry_msgs::PolygonStamped GetPolygonMsg(int index);

private:
    std::vector<NaviFra::Polygon> vec_static_polygons_;
    std::mutex mtx_polygon_;

    geometry_msgs::PolygonStamped DrawPolygon(const std::vector<NaviFra::Pos>& vec_pos);
};

}  // namespace NaviFra
#endif
