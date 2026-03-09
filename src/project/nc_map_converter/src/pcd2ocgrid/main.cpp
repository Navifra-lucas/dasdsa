#include "nc_map_converter/pcd_2_ocgrid.h"
#include "core/util/logger.hpp"

using namespace NaviFra;
using namespace std;

Pcd2Ocgrid::Pcd2Ocgrid(ros::NodeHandle& nh ,ros::NodeHandle& nhp ) : nh_(nh), nhp_(nhp)
{
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occocc", 1000, true);
    pcd_to_png_sub_ = nh_.subscribe<std_msgs::Bool>("/map_convert/pcd_to_png", 10, &Pcd2Ocgrid::ConvertCallback, this); 
}

Pcd2Ocgrid::~Pcd2Ocgrid()
{
}

void Pcd2Ocgrid::ConvertCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ros::param::get("/resolution", image_resolution_);
  ros::param::get("/pcd_path", pcd_path_);
  ros::param::get("/image_width", image_width_);
  ros::param::get("/image_height", image_height_);

  PcdConvert();
}

void Pcd2Ocgrid::PcdConvert()
{
    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcd_path_, *cloud);

    // Voxel filtering
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*filtered_cloud);

    float min_x, max_x, min_y, max_y;
    min_x = max_x = filtered_cloud->points[0].x;
    min_y = max_y = filtered_cloud->points[0].y;
    for (size_t i = 0; i < filtered_cloud->size(); i++) {
        float x = filtered_cloud->points[i].x;
        float y = filtered_cloud->points[i].y;
        if (x<min_x) min_x = x;
        if (x>max_x) max_x = x;
        if (y<min_y) min_y = y;
        if (y>max_y) max_y = y;
    }

    float ratio = (max_x-min_x) / (max_y - min_y);
    LOG_INFO("X %.2f Y %.2f", (max_x-min_x), (max_y-min_y));
    // Convert to OccupancyGrid
    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = "map";
    grid_msg.header.stamp = ros::Time::now();
    grid_msg.info.resolution = image_resolution_;
    //grid_msg.info.width = (int)((max_x-min_x)/grid_msg.info.resolution);
    //grid_msg.info.height = (int)((max_y-min_y)/grid_msg.info.resolution);
    grid_msg.info.width  = image_width_;
    grid_msg.info.height = image_height_;
    grid_msg.info.origin.position.x = 0.0;
    grid_msg.info.origin.position.y = 0.0;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;
    grid_msg.info.origin.orientation.x = 0.0;
    grid_msg.info.origin.orientation.y = 0.0;
    grid_msg.info.origin.orientation.z = 0.0;
    grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);

    LOG_INFO("Resolution %.2f Width %d Height %d", grid_msg.info.resolution, grid_msg.info.width, grid_msg.info.height);
    LOG_INFO("min  %.2f %.2f max %.2f %.2f", min_x, min_y, max_x, max_y);
    LOG_INFO("Origin  %.2f %.2f", grid_msg.info.origin.position.x, grid_msg.info.origin.position.y);

    for (size_t i = 0; i < filtered_cloud->size(); i++) {
        // int x = (filtered_cloud->points[i].x- min_x) / grid_msg.info.resolution ;
        // int y = (filtered_cloud->points[i].y- min_y) / grid_msg.info.resolution ;

        int x = (filtered_cloud->points[i].x ) / grid_msg.info.resolution + int(grid_msg.info.width/2);
        int y = (filtered_cloud->points[i].y ) / grid_msg.info.resolution + int(grid_msg.info.height/2);

        if (x < 0 || x >= grid_msg.info.width || y < 0 || y >= grid_msg.info.height) {
          continue;
        }

        int index = y * grid_msg.info.width + x;
        grid_msg.data[index] = 100;
    }

    // Publish OccupancyGrid message
    grid_pub_.publish(grid_msg);
}

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "pcd_to_occupancygrid");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    NaviFra::Pcd2Ocgrid po(nh,nhp);

    try
    {
        ros::spin();
    }
    catch(std::runtime_error& e)
    {
        ROS_ERROR("pcd_to_occupancygrid exception: %s", e.what());
        return -1;
    }

    return 0;
}