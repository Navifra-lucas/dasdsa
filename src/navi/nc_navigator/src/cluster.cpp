#include "nc_navigator/cluster.hpp"

namespace LidarClustering {

void Cluster::SetCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr, const std::vector<int>& in_cluster_indices,
    std_msgs::Header in_ros_header, int in_id, RGB color, std::string in_label)
{
    label_ = in_label;
    id_ = in_id;
    color_ = color;

    // extract pointcloud using the indices
    // calculate min and max points
    pointcloud_->points.clear();
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    float average_x = 0, average_y = 0, average_z = 0;

    for (auto it = in_cluster_indices.begin(); it != in_cluster_indices.end(); ++it) {
        // fill new colored cluster point by point
        pcl::PointXYZRGB p;
        p.x = in_origin_cloud_ptr->points[*it].x;
        p.y = in_origin_cloud_ptr->points[*it].y;
        p.z = in_origin_cloud_ptr->points[*it].z;

        p.r = 255;
        p.g = 255;
        p.b = 0;

        average_x += p.x;
        average_y += p.y;
        average_z += p.z;
        centroid_.x += p.x;
        centroid_.y += p.y;
        centroid_.z += p.z;
        pointcloud_->points.push_back(p);

        if (p.x < min_x)
            min_x = p.x;
        if (p.y < min_y)
            min_y = p.y;
        if (p.z < min_z)
            min_z = p.z;
        if (p.x > max_x)
            max_x = p.x;
        if (p.y > max_y)
            max_y = p.y;
        if (p.z > max_z)
            max_z = p.z;
    }
    // min, max points
    min_point_.x = min_x;
    min_point_.y = min_y;
    min_point_.z = min_z;
    max_point_.x = max_x;
    max_point_.y = max_y;
    max_point_.z = max_z;

    // calculate centroid, average
    if (in_cluster_indices.size() > 0) {
        centroid_.x /= in_cluster_indices.size();
        centroid_.y /= in_cluster_indices.size();
        centroid_.z /= in_cluster_indices.size();

        average_x /= in_cluster_indices.size();
        average_y /= in_cluster_indices.size();
        average_z /= in_cluster_indices.size();
    }

    average_point_.x = average_x;
    average_point_.y = average_y;
    average_point_.z = average_z;

    // calculate bounding box
    length_ = max_point_.x - min_point_.x;
    width_ = max_point_.y - min_point_.y;
    height_ = max_point_.z - min_point_.z;

    std::vector<cv::Point2f> points;
    for (unsigned int i = 0; i < pointcloud_->points.size(); i++) {
        cv::Point2f pt;
        pt.x = pointcloud_->points[i].x;
        pt.y = pointcloud_->points[i].y;
        points.push_back(pt);
    }

    std::vector<cv::Point2f> hull;
    cv::convexHull(points, hull);

    // polygon_.header = in_ros_header;
    // polygon_.polygon.points.clear();
    // for (size_t i = 0; i < hull.size() + 1; i++) {
    //     geometry_msgs::Point32 point;
    //     point.x = hull[i % hull.size()].x;
    //     point.y = hull[i % hull.size()].y;
    //     point.z = min_point_.z;
    //     polygon_.polygon.points.push_back(point);
    // }

    pointcloud_->width = pointcloud_->points.size();
    pointcloud_->height = 1;
    pointcloud_->is_dense = true;

    b_valid_cluster_ = true;
}
}  // namespace LidarClustering