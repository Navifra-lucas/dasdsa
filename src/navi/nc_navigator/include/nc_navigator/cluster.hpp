#ifndef __CLUSTER_HPP__
#define __CLUSTER_HPP__

// #include "obstacle_detector_pch.hpp"

#include <geometry_msgs/PolygonStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
// #include "brain_msgs/VehicleState.h"
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using pPointCloudXYZ = PointCloudXYZ::Ptr;
using cPointCloudXYZ = const PointCloudXYZ;
using cpPointCloudXYZ = const pPointCloudXYZ;

using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using pPointCloudXYZRGB = PointCloudXYZRGB::Ptr;
using cPointCloudXYZRGB = const PointCloudXYZRGB;
using cpPointCloudXYZRGB = const pPointCloudXYZRGB;

namespace LidarClustering {

struct RGB {
    int r;
    int g;
    int b;
};

class Cluster {
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_;
    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;
    pcl::PointXYZ average_point_;
    pcl::PointXYZ centroid_;
    double orientation_angle_;
    float length_, width_, height_;

    //   jsk_recognition_msgs::BoundingBox bounding_box_;
    geometry_msgs::PolygonStamped polygon_;

    std::string label_;
    int id_;
    RGB color_;

    Eigen::Matrix3f eigen_vectors_;
    Eigen::Vector3f eigen_values_;

    bool b_valid_cluster_ = true;

public:
    /* \brief Constructor. Creates a Cluster object using the specified points in a PointCloud
     * \param[in] in_origin_cloud_ptr   Origin PointCloud
     * \param[in] in_cluster_indices   Indices of the Origin Pointcloud to create the Cluster
     * \param[in] in_id         ID of the cluster
     * \param[in] in_r           Amount of Red [0-255]
     * \param[in] in_g           Amount of Green [0-255]
     * \param[in] in_b           Amount of Blue [0-255]
     * \param[in] in_label         Label to identify this cluster (optional)
     * \param[in] in_estimate_pose    Flag to enable Pose Estimation of the Bounding Box
     * */
    void SetCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr, const std::vector<int>& in_cluster_indices,
        std_msgs::Header in_ros_header, int in_id, RGB color, std::string in_label);

    /* \brief Returns the autoware_msgs::CloudCluster message associated to this Cluster */
    //   void ToROSMessage(std_msgs::Header in_ros_header, uav_msgs::CloudCluster& out_cluster_message);

    Cluster() { pointcloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>); }
    virtual ~Cluster() = default;

    /* \brief Returns the pointer to the PointCloud containing the points in this Cluster */
    inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud() { return pointcloud_; }
    /* \brief Returns the minimum point in the cluster */
    inline pcl::PointXYZ GetMinPoint() { return min_point_; }
    /* \brief Returns the maximum point in the cluster*/
    inline pcl::PointXYZ GetMaxPoint() { return max_point_; }
    /* \brief Returns the average point in the cluster*/
    inline pcl::PointXYZ GetAveragePoint() { return average_point_; }
    /* \brief Returns the centroid point in the cluster */
    inline pcl::PointXYZ GetCentroid() { return centroid_; }
    //   /* \brief Returns the calculated BoundingBox of the object */
    //   inline jsk_recognition_msgs::BoundingBox GetBoundingBox() { return bounding_box_; }
    /* \brief Returns the calculated PolygonArray of the object */
    // inline geometry_msgs::PolygonStamped GetPolygon() { return polygon_; }
    /* \brief Returns the angle in radians of the BoundingBox. 0 if pose estimation was not enabled. */
    inline double GetOrientationAngle() { return orientation_angle_; }
    /* \brief Returns the Id of the Cluster */
    inline int GetId() { return id_; }
    /* \brief Returns the Eigen Vectors of the cluster */
    inline Eigen::Matrix3f GetEigenVectors() { return eigen_vectors_; }
    /* \brief Returns the Eigen Values of the Cluster */
    inline Eigen::Vector3f GetEigenValues() { return eigen_values_; }
    /* \brief Returns if the Cluster is marked as valid or not*/
    inline bool IsValid() { return b_valid_cluster_; }
    /* \brief Sets whether the Cluster is valid or not*/
    inline void SetValidity(bool in_valid) { b_valid_cluster_ = in_valid; }
    /* \brief Retunrs color of the Cluster */
    inline RGB GetColor() { return color_; }
};

typedef boost::shared_ptr<Cluster> ClusterPtr;

}  // namespace LidarClustering
#endif