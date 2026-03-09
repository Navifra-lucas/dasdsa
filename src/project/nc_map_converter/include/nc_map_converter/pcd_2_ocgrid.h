#ifndef PCD_2_OCGRID_H
#define PCD_2_OCGRID_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/OccupancyGrid.h>

#include <stdlib.h>

namespace NaviFra{
    class Pcd2Ocgrid
    {
    public:
        ~Pcd2Ocgrid();
        Pcd2Ocgrid(ros::NodeHandle& nh ,ros::NodeHandle& nhp ) ;
    
    private:
        ros::NodeHandle& nh_;
        ros::NodeHandle& nhp_;

        ros::Publisher grid_pub_;
        ros::Subscriber pcd_to_png_sub_;

        std::string pcd_path_;
        float image_resolution_;
        float image_width_;
        float image_height_;

        void PcdConvert();
        void ConvertCallback(const std_msgs::Bool::ConstPtr& msg);

    };
};

#endif