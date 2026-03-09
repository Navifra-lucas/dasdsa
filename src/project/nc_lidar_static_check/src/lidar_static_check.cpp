#include "nc_lidar_static_check/lidar_static_check.hpp"

#include <cmath>

namespace NaviFra {

LidarStaticCheck::LidarStaticCheck()
{
}

LidarStaticCheck::~LidarStaticCheck()
{
}

void LidarStaticCheck::SetPolygon(const std::vector<NaviFra::Polygon>& vec_polygon)
{
    std::lock_guard<std::mutex> lock(mtx_polygon_);
    vec_static_polygons_ = vec_polygon;
}

bool LidarStaticCheck::CheckObstacle(const sensor_msgs::PointCloud::ConstPtr& cloud_msg)
{
    std::lock_guard<std::mutex> lock(mtx_polygon_);
    if (vec_static_polygons_.empty()) {
        //오류
        return false;
    }

    if (vec_static_polygons_.size() < 2) {
        //오류....????????
        return false;
    }

    // index 0 = outline
    // index 1 = collision
    auto& outline_poly = vec_static_polygons_[0];
    auto& collision_poly = vec_static_polygons_[1];

    for (const auto& p : cloud_msg->points) {
        NaviFra::Pos pt(p.x, p.y, 0);

        bool inside_collision = collision_poly.GetPointInPolygon(pt);
        if (!inside_collision)
            continue;

        bool inside_outline = outline_poly.GetPointInPolygon(pt);

        if (inside_collision && !inside_outline) {
            NLOG(info) << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
            return true;
        }
    }
    return false;
}

geometry_msgs::PolygonStamped LidarStaticCheck::GetPolygonMsg(int index)
{
    std::lock_guard<std::mutex> lock(mtx_polygon_);
    if (index < 0 || index >= vec_static_polygons_.size()) {
        return geometry_msgs::PolygonStamped();
    }
    return DrawPolygon(vec_static_polygons_[index].o_polygon_vertexs_);
}

geometry_msgs::PolygonStamped LidarStaticCheck::DrawPolygon(const std::vector<NaviFra::Pos>& vec_pos)
{
    geometry_msgs::PolygonStamped poly_msg;
    geometry_msgs::Point32 p;

    poly_msg.header.frame_id = "base_link";  // Assuming base_link for now
    poly_msg.header.stamp = ros::Time::now();

    int n_vec_size = vec_pos.size();
    for (int i = 0; i < n_vec_size; i++) {
        p.x = vec_pos[i].GetXm();
        p.y = vec_pos[i].GetYm();
        poly_msg.polygon.points.push_back(p);
    }
    return poly_msg;
}

}  // namespace NaviFra
