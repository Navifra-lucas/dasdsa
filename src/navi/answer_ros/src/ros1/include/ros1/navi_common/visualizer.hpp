#ifndef VISUALIZER_ROS1_HPP_
#define VISUALIZER_ROS1_HPP_

#include <vector>
#include <string>
#include <map>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "utils/debug_visualizer.hpp"
#include "utils/pose.hpp"
#include "utils/grid_map/map_info.hpp"

using namespace NVFR;

class Visualizer
{
public:
  Visualizer(const ros::NodeHandle& nh, const std::string& s_namespace="");
  ~Visualizer() { map_pub_.clear(); };

  void SetNameSpace(const std::string& s_namespace);
  void SetFrameId(const std::string& s_fixed, const std::string& s_local);

private:
  // ros
  ros::NodeHandle nh_;
  std::map< std::string, std::shared_ptr<ros::Publisher> > map_pub_;

  // frame_id
  std::string TOPIC_NAMESPACE;
  std::string FIXED_FRAME_ID;
  std::string LOCAL_FRAME_ID;

  // Receive data
  void RecvPose(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvLocalPose(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvMultiPose(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvLocalMultiPose(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvPath(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvLocalPath(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvPolygon(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvLocalPolygon(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvMultiPolygon(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvLocalMultiPolygon(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvGlobalMap(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);
  void RecvLocalMap(const boost::any& any_type_var1, const boost::any& any_type_var2, const boost::any& any_type_var3);

  // Draw data
  visualization_msgs::Marker DrawPose(const std::string& s_frame_id, const Pose& pose, const DrawInfo& st_draw_info) const;
  visualization_msgs::MarkerArray DrawMultiPose(const std::string& s_frame_id, const std::string& s_ns, const std::vector<Pose>& vec_pose, const DrawInfo& st_draw_info) const;
  visualization_msgs::Marker DrawPath(const std::string& s_frame_id, const Path& path, const DrawInfo& st_draw_info) const;
  visualization_msgs::Marker DrawPolygon(const std::string& s_frame_id, const std::vector<Pose>& polygon, const DrawInfo& st_draw_info) const;
  visualization_msgs::MarkerArray DrawMultiPolygon(const std::string& s_frame_id, const std::string& s_ns, const std::vector< std::vector<Pose> >& vec_polygon, const DrawInfo& st_draw_info) const;
  nav_msgs::OccupancyGrid DrawMap(const std::string& s_frame_id, const std::vector<int8_t>& grid_map, const MapInfo_t& st_map_info, double d_z_m=-1.0);

};

#endif
