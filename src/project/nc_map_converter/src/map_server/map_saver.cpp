#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <std_msgs/Bool.h>

#include "nc_map_converter/pcd_2_ocgrid.h"

using namespace NaviFra;
using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator
{
  public:

    MapGenerator(int threshold_occupied, int threshold_free)
      : saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
      ros::NodeHandle n;
      map_update_pub_ = n.advertise<std_msgs::Bool>("/map_update_acs", 1);
      pcd_to_png_sub_ = n.subscribe<std_msgs::Bool>("/map_convert/pcd_to_png", 10, &MapGenerator::ConvertCallback, this); 
      // map_sub_ = n.subscribe("/occocc", 1, &MapGenerator::mapCallback, this);
      n.getParam("/map_saver/map_path", s_map_path_); 
    }

    void ConvertCallback(const std_msgs::Bool::ConstPtr& msg)
    {
      ros::param::get("/resolution", image_resolution_);
      ros::param::get("/pcd_path", pcd_path_);
      ros::param::get("/image_width", image_width_);
      ros::param::get("/image_height", image_height_);

      PcdConvert();
    }

    void PcdConvert()
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

        std::string mapdatafile = s_map_path_ + "/map.pgm";
        FILE* out = fopen(mapdatafile.c_str(), "w");
        if (!out)
        {
          return;
        }

        fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
                grid_msg.info.resolution, grid_msg.info.width, grid_msg.info.height);
        for(unsigned int y = 0; y < grid_msg.info.height; y++) {
          for(unsigned int x = 0; x < grid_msg.info.width; x++) {
            unsigned int i = x + (grid_msg.info.height - y - 1) * grid_msg.info.width;
            if (grid_msg.data[i] >= 0 && grid_msg.data[i] <= threshold_free_) { // [0,free)
              fputc(254, out);
            } else if (grid_msg.data[i] >= threshold_occupied_) { // (occ,255]
              fputc(000, out);
            } else { //occ [0.25,0.65]
              fputc(205, out);
            }
          }
        }

        fclose(out);

        saved_map_ = true;

        std::string s_command = "convert " + mapdatafile + " " + s_map_path_ + "/map.png";

        int result = std::system(s_command.c_str());
        std_msgs::Bool msg;
        msg.data = true;
        map_update_pub_.publish(msg);
    }

    ros::Subscriber map_sub_;
    ros::Publisher map_update_pub_;
    ros::Subscriber pcd_to_png_sub_;
    std::string pcd_path_;
    float image_resolution_;
    float image_width_;
    float image_height_;
    bool saved_map_;
    int threshold_occupied_;
    int threshold_free_;
    std::string s_map_path_;

};

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";
  int threshold_occupied = 65;
  int threshold_free = 25;

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else if (!strcmp(argv[i], "--occ"))
    {
      if (++i < argc)
      {
        threshold_occupied = std::atoi(argv[i]);
        if (threshold_occupied < 1 || threshold_occupied > 100)
        {
          return 1;
        }
      }
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else if (!strcmp(argv[i], "--free"))
    {
      if (++i < argc)
      {
        threshold_free = std::atoi(argv[i]);
        if (threshold_free < 0 || threshold_free > 100)
        {
          return 1;
        }
      }
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  if (threshold_occupied <= threshold_free)
  {
    return 1;
  }
  MapGenerator mg(threshold_occupied, threshold_free);

  ros::Rate rate(10);
  while (ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}