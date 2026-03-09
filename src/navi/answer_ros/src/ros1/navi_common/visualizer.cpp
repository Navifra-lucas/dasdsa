#include "ros1/navi_common/visualizer.hpp"

#include "logger/logger.h"
#include "utils/common_math.hpp"

Visualizer::Visualizer(const ros::NodeHandle& nh, const std::string& s_namespace)
    : nh_(nh)
    , TOPIC_NAMESPACE(s_namespace.empty() ? "" : s_namespace + "/")
    , FIXED_FRAME_ID("map")
    , LOCAL_FRAME_ID("base_link")
{
    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::POSE,
        std::bind(
            &Visualizer::RecvPose, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::LOCAL_POSE,
        std::bind(
            &Visualizer::RecvLocalPose, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::MULTI_POSE,
        std::bind(
            &Visualizer::RecvMultiPose, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::LOCAL_MULTI_POSE,
        std::bind(
            &Visualizer::RecvLocalMultiPose, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::PATH,
        std::bind(
            &Visualizer::RecvPath, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::LOCAL_PATH,
        std::bind(
            &Visualizer::RecvLocalPath, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::POLYGON,
        std::bind(
            &Visualizer::RecvPolygon, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::LOCAL_POLYGON,
        std::bind(
            &Visualizer::RecvLocalPolygon, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::MULTI_POLYGON,
        std::bind(
            &Visualizer::RecvMultiPolygon, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::LOCAL_MULTI_POLYGON,
        std::bind(
            &Visualizer::RecvLocalMultiPolygon, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::GLOBAL_MAP,
        std::bind(
            &Visualizer::RecvGlobalMap, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));

    NVFR::DebugVisualizer::GetInstance()->GetCbRegister()->RegistCbFunc(
        NVFR::DebugVisualizer::LOCAL_MAP,
        std::bind(
            &Visualizer::RecvLocalMap, this, std::placeholders::_1,
            std::placeholders::_2, std::placeholders::_3));
}

// namespace
void Visualizer::SetNameSpace(const std::string& s_namespace)
{
    TOPIC_NAMESPACE = s_namespace.empty() ? "" : s_namespace + "/";
}

// frame_id
void Visualizer::SetFrameId(
    const std::string& s_fixed,
    const std::string& s_local)
{
    FIXED_FRAME_ID = s_fixed;
    LOCAL_FRAME_ID = s_local;
}

// Receive data
void Visualizer::RecvPose(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    Pose pose = boost::any_cast<Pose>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::Marker marker =
        DrawPose(FIXED_FRAME_ID, pose, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::Marker>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist pose msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker);
}

void Visualizer::RecvLocalPose(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    Pose pose = boost::any_cast<Pose>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::Marker marker =
        DrawPose(LOCAL_FRAME_ID, pose, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::Marker>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist pose msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker);
}

void Visualizer::RecvMultiPose(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    std::vector<Pose> vec_pose =
        boost::any_cast<std::vector<Pose>>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::MarkerArray marker_array =
        DrawMultiPose(FIXED_FRAME_ID, s_topic_name, vec_pose, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::MarkerArray>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist multi pose msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker_array);
}

void Visualizer::RecvLocalMultiPose(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    std::vector<Pose> vec_pose =
        boost::any_cast<std::vector<Pose>>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::MarkerArray marker_array =
        DrawMultiPose(LOCAL_FRAME_ID, s_topic_name, vec_pose, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::MarkerArray>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist multi pose msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker_array);
}

void Visualizer::RecvPath(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    Path path = boost::any_cast<Path>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::Marker marker =
        DrawPath(FIXED_FRAME_ID, path, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::Marker>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist path msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker);
}

void Visualizer::RecvLocalPath(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    Path path = boost::any_cast<Path>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::Marker marker =
        DrawPath(LOCAL_FRAME_ID, path, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::Marker>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist path msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker);
}

void Visualizer::RecvPolygon(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    std::vector<Pose> polygon =
        boost::any_cast<std::vector<Pose>>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::Marker marker =
        DrawPolygon(FIXED_FRAME_ID, polygon, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::Marker>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist polygon msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker);
}

void Visualizer::RecvLocalPolygon(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    std::vector<Pose> polygon =
        boost::any_cast<std::vector<Pose>>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::Marker marker =
        DrawPolygon(LOCAL_FRAME_ID, polygon, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::Marker>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist polygon msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker);
}

void Visualizer::RecvMultiPolygon(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    std::vector<std::vector<Pose>> vec_polygon =
        boost::any_cast<std::vector<std::vector<Pose>>>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::MarkerArray marker_array =
        DrawMultiPolygon(FIXED_FRAME_ID, s_topic_name, vec_polygon, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::MarkerArray>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist multi polygon msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker_array);
}

void Visualizer::RecvLocalMultiPolygon(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    std::vector<std::vector<Pose>> vec_polygon =
        boost::any_cast<std::vector<std::vector<Pose>>>(any_type_var2);
    DrawInfo st_draw_info = boost::any_cast<DrawInfo>(any_type_var3);

    visualization_msgs::MarkerArray marker_array =
        DrawMultiPolygon(LOCAL_FRAME_ID, s_topic_name, vec_polygon, st_draw_info);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<visualization_msgs::MarkerArray>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist multi polygon msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(marker_array);
}

void Visualizer::RecvGlobalMap(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    std::vector<int8_t> grid_map =
        boost::any_cast<std::vector<int8_t>>(any_type_var2);
    MapInfo_t st_map_info = boost::any_cast<MapInfo_t>(any_type_var3);

    nav_msgs::OccupancyGrid og =
        DrawMap(FIXED_FRAME_ID, grid_map, st_map_info, -1.0);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<nav_msgs::OccupancyGrid>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist grid map msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(og);
}

void Visualizer::RecvLocalMap(
    const boost::any &any_type_var1, const boost::any &any_type_var2,
    const boost::any &any_type_var3)
{
    std::string s_topic_name = boost::any_cast<std::string>(any_type_var1);
    std::vector<int8_t> grid_map =
        boost::any_cast<std::vector<int8_t>>(any_type_var2);
    MapInfo_t st_map_info = boost::any_cast<MapInfo_t>(any_type_var3);

    nav_msgs::OccupancyGrid og =
        DrawMap(LOCAL_FRAME_ID, grid_map, st_map_info, -0.5);

    if (map_pub_.find(s_topic_name) == map_pub_.end()) {
        map_pub_[s_topic_name] =
            std::make_shared<ros::Publisher>(
                nh_.advertise<nav_msgs::OccupancyGrid>(
                    TOPIC_NAMESPACE + s_topic_name, 10));
        LOG_DEBUG("[Visualizer] regist grid map msg : /{}",
            s_topic_name.c_str());
    }

    map_pub_[s_topic_name]->publish(og);
}

// Draw data
visualization_msgs::Marker Visualizer::DrawPose(
    const std::string &s_frame_id, const Pose &pose,
    const DrawInfo &st_draw_info) const
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = s_frame_id;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = st_draw_info.st_scale.f_scale_x_m;
    marker.scale.y = st_draw_info.st_scale.f_scale_y_m;
    marker.scale.z = 0.5;
    marker.color.a = st_draw_info.st_color.f_a;
    marker.color.r = st_draw_info.st_color.f_r;
    marker.color.g = st_draw_info.st_color.f_g;
    marker.color.b = st_draw_info.st_color.f_b;

    marker.pose.position.x = pose.GetXm();
    marker.pose.position.y = pose.GetYm();
    marker.pose.position.z = 0.7;
    marker.pose.orientation.w = 1.0;

    return marker;
}

visualization_msgs::MarkerArray Visualizer::DrawMultiPose(
    const std::string &s_frame_id, const std::string& s_ns,
    const std::vector<Pose> &vec_pose, const DrawInfo &st_draw_info) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = s_frame_id;
    marker.ns = s_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = st_draw_info.st_scale.f_scale_x_m;
    marker.scale.y = st_draw_info.st_scale.f_scale_y_m;
    marker.scale.z = 0.5;
    marker.color.a = st_draw_info.st_color.f_a;
    marker.color.r = st_draw_info.st_color.f_r;
    marker.color.g = st_draw_info.st_color.f_g;
    marker.color.b = st_draw_info.st_color.f_b;

    marker.pose.position.z = 0.7;
    marker.pose.orientation.w = 1.0;

    size_t un_vec_size = vec_pose.size();
    marker_array.markers.resize(vec_pose.size(), marker);
    for (size_t i = 0; i < un_vec_size; ++i) {
        marker_array.markers[i].id = i;
        marker_array.markers[i].pose.position.x = vec_pose[i].GetXm();
        marker_array.markers[i].pose.position.y = vec_pose[i].GetYm();
        auto quat = CM::Yaw2Quaternion(vec_pose[i].GetRad());
        marker_array.markers[i].pose.orientation.z = quat[2];
        marker_array.markers[i].pose.orientation.w = quat[3];
    }

    return marker_array;
}

visualization_msgs::Marker Visualizer::DrawPath(
    const std::string &s_frame_id, const Path &path,
    const DrawInfo &st_draw_info) const
{
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = s_frame_id;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = st_draw_info.st_scale.f_scale_x_m;
    marker.scale.y = st_draw_info.st_scale.f_scale_y_m;
    marker.scale.z = 0.5;
    marker.color.a = st_draw_info.st_color.f_a;
    marker.color.r = st_draw_info.st_color.f_r;
    marker.color.g = st_draw_info.st_color.f_g;
    marker.color.b = st_draw_info.st_color.f_b;
    marker.pose.orientation.w = 1.0;

    size_t un_vec_size = path.size();
    marker.points.resize(un_vec_size);
    for (size_t i = 0; i < un_vec_size; ++i) {
        marker.points[i].x = path[i].GetXm();
        marker.points[i].y = path[i].GetYm();
        marker.points[i].z = 0.2;
    }

    return marker;
}

visualization_msgs::Marker Visualizer::DrawPolygon(
    const std::string &s_frame_id, const std::vector<Pose> &polygon,
    const DrawInfo &st_draw_info) const
{
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = s_frame_id;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = st_draw_info.st_scale.f_scale_x_m;
    marker.scale.y = st_draw_info.st_scale.f_scale_y_m;
    marker.scale.z = 0.5;
    marker.color.a = st_draw_info.st_color.f_a;
    marker.color.r = st_draw_info.st_color.f_r;
    marker.color.g = st_draw_info.st_color.f_g;
    marker.color.b = st_draw_info.st_color.f_b;
    marker.pose.orientation.w = 1.0;

    size_t un_vec_size = polygon.size();
    marker.points.resize(un_vec_size + 1UL);
    for (size_t i = 0; i < un_vec_size; ++i) {
        marker.points[i].x = polygon[i].GetXm();
        marker.points[i].y = polygon[i].GetYm();
        marker.points[i].z = 0.3;
    }
    marker.points.back().x = polygon[0].GetXm();
    marker.points.back().y = polygon[0].GetYm();
    marker.points.back().z = 0.3;

    return marker;
}

visualization_msgs::MarkerArray Visualizer::DrawMultiPolygon(
    const std::string &s_frame_id, const std::string& s_ns,
    const std::vector<std::vector<Pose>> &vec_polygon,
    const DrawInfo &st_draw_info) const
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = s_frame_id;
    marker.ns = s_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = st_draw_info.st_scale.f_scale_x_m;
    marker.scale.y = st_draw_info.st_scale.f_scale_y_m;
    marker.scale.z = 0.5;
    marker.color.a = st_draw_info.st_color.f_a;
    marker.color.r = st_draw_info.st_color.f_r;
    marker.color.g = st_draw_info.st_color.f_g;
    marker.color.b = st_draw_info.st_color.f_b;
    marker.pose.orientation.w = 1.0;

    size_t un_vec_size = vec_polygon.size();
    marker_array.markers.resize(un_vec_size, marker);
    for (size_t k = 0; k < un_vec_size; ++k) {
        marker_array.markers[k].id = k;
        size_t un_polygon_size = vec_polygon[k].size();
        marker_array.markers[k].points.resize(un_polygon_size + 1UL);
        for (size_t i = 0; i < un_polygon_size; ++i) {
            marker_array.markers[k].points[i].x = vec_polygon[k][i].GetXm();
            marker_array.markers[k].points[i].y = vec_polygon[k][i].GetYm();
            marker_array.markers[k].points[i].z = 0.3;
        }
        marker_array.markers[k].points.back().x = vec_polygon[k][0].GetXm();
        marker_array.markers[k].points.back().y = vec_polygon[k][0].GetYm();
        marker_array.markers[k].points.back().z = 0.3;
    }

    return marker_array;
}

nav_msgs::OccupancyGrid Visualizer::DrawMap(
    const std::string &s_frame_id, const std::vector<int8_t> &grid_map,
    const MapInfo_t &st_map_info, double d_z_m)
{
    nav_msgs::OccupancyGrid og;
    og.header.stamp = ros::Time::now();
    og.header.frame_id = s_frame_id;
    og.info.height = st_map_info.n_size_y_px;
    og.info.width = st_map_info.n_size_x_px;
    og.info.resolution = st_map_info.d_resolution_m;
    og.info.origin.position.x = -st_map_info.d_origin_x_m;
    og.info.origin.position.y = -st_map_info.d_origin_y_m;
    og.info.origin.position.z = d_z_m;
    og.info.origin.orientation.w = 1.0;

    og.data = grid_map;

    return og;
}
