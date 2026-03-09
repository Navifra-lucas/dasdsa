#include "nc_v2v_safer.hpp"

using namespace NaviFra;

V2V_SAFER::V2V_SAFER(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    pub_vehicle_point_ = nh_.advertise<sensor_msgs::PointCloud>("/v2v_cloud", 1);
    pub_vehicle_point_global_ = nh_.advertise<sensor_msgs::PointCloud>("/v2v_cloud_global", 1);
    sub_v2v_info_ = nh_.subscribe("/v2v_info", 5, &V2V_SAFER::ReceivedV2Vinfo, this);
    sub_rsr_info_ = nh_.subscribe("/rsr_info", 5, &V2V_SAFER::ReceivedRSRinfo, this);
    navi_info_ = nh_.subscribe("/navifra/info", 5, &V2V_SAFER::ReceivedNaviinfo, this);

    if (nh_.getParam("nc_v2v_safer/f_detect_sec", f_detect_sec_));
    if (nh_.getParam("nc_v2v_safer/f_detect_range_m", f_detect_range_m_));
    if (nh_.getParam("nc_v2v_safer/f_predict_sec", f_predict_sec_));
    if (nh_.getParam("nc_v2v_safer/f_period_sec", f_period_sec_));
    th_ = boost::thread(boost::bind(&V2V_SAFER::MainLoop, this));
}

V2V_SAFER::~V2V_SAFER()
{
    b_thread_run_ = false;
    if(th_.joinable()) {
        th_.interrupt();
        th_.join();
    }
}

void V2V_SAFER::MainLoop()
{
    while (b_thread_run_) {
        core_msgs::NavicoreStatus o_robot;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            o_robot = o_navi_msg_;
        }
        sensor_msgs::PointCloud msg_point;
        {
            std::lock_guard<std::mutex> lock(mtx_map_);
            for (auto it = map_vehicle_.begin(); it != map_vehicle_.end(); it++) {
                std::chrono::duration<double> sec = std::chrono::system_clock::now() - it->second.update_time;
                float f_dist = hypot(it->second.x_m - o_robot.f_robot_pos_x_m, it->second.y_m - o_robot.f_robot_pos_y_m);
                // cout<<"it-> "<<it->second.id<<" sec.count() "<<sec.count()<<" f_dist "<<f_dist<<endl;
                if (sec.count() <= f_detect_sec_ && f_dist <= f_detect_range_m_) {
                    // cout<<"make point"<<endl;
                    MakePoint(msg_point, it->second, o_robot);
                }
            }
        }
        pub_vehicle_point_.publish(msg_point);

        pub_vehicle_point_global_.publish(ToGlobal(msg_point));
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
}
sensor_msgs::PointCloud V2V_SAFER::ToGlobal(sensor_msgs::PointCloud& vec_cloud)
{
    sensor_msgs::PointCloud msg_result = vec_cloud;

    Pos current_pose;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        current_pose.SetXm(o_navi_msg_.f_robot_pos_x_m);
        current_pose.SetYm(o_navi_msg_.f_robot_pos_y_m);
        current_pose.SetDeg(o_navi_msg_.f_robot_pos_deg);
    }

    for (int i = 0; i < vec_cloud.points.size(); i++) {
        float x = msg_result.points[i].x * cos(current_pose.GetRad()) - msg_result.points[i].y * sin(current_pose.GetRad()) +
            current_pose.GetXm();
        float y = msg_result.points[i].x * sin(current_pose.GetRad()) + msg_result.points[i].y * cos(current_pose.GetRad()) +
            current_pose.GetYm();
        msg_result.points[i].x = x;
        msg_result.points[i].y = y;
    }
    return msg_result;
}

void V2V_SAFER::MakePoint(sensor_msgs::PointCloud& msg_point, const VehicleInfo& o_vehicle, const core_msgs::NavicoreStatus& o_robot)
{
    vector<Pos> vec_point;
    Pos fl(o_vehicle.f_robot_size_front_m, o_vehicle.f_robot_size_left_m);
    Pos fr(o_vehicle.f_robot_size_front_m, -fabs(o_vehicle.f_robot_size_right_m));
    Pos rr(-fabs(o_vehicle.f_robot_size_rear_m), -fabs(o_vehicle.f_robot_size_right_m));
    Pos rl(-fabs(o_vehicle.f_robot_size_rear_m), o_vehicle.f_robot_size_left_m);
    vec_point.emplace_back(fl);
    vec_point.emplace_back(fr);
    vec_point.emplace_back(rr);
    vec_point.emplace_back(rl);

    for (int i = 0; i < 4; i++) {
        vec_point[i] = CoreCalculator::TransformRotationDeg_(vec_point[i], o_vehicle.angle_deg);
        float f_x = (vec_point[i].GetXm() + o_vehicle.x_m - o_robot.f_robot_pos_x_m);
        float f_y = (vec_point[i].GetYm() + o_vehicle.y_m - o_robot.f_robot_pos_y_m);
        float f_rad = -o_robot.f_robot_pos_deg * DEGtoRAD;
        // cout<<"fx "<<f_x<<" fy "<<f_y<<" // "<<f_rad<<endl;
        // cout<<"o_robot.f_robot_pos_x_m "<<o_robot.f_robot_pos_x_m<<" y "<<o_robot.f_robot_pos_y_m<<endl;
        vec_point[i].SetXm(f_x * cos(f_rad) - f_y * sin(f_rad));
        vec_point[i].SetYm(f_x * sin(f_rad) + f_y * cos(f_rad));

        // cout<<i<<" x "<<vec_point[i].GetXm()<<" / "<<vec_point[i].GetYm()<<endl;
    }

    Pos o_expand(o_vehicle.f_linear_speed_x_ms * f_predict_sec_, o_vehicle.f_linear_speed_y_ms * f_predict_sec_, 0);
    vector<Pos> vec_predict;
    vec_predict.emplace_back(fl);
    vec_predict.emplace_back(fr);
    vec_predict.emplace_back(rr);
    vec_predict.emplace_back(rl);
    for (int i = 0; i < 4; i++) {
        vec_predict[i] = CoreCalculator::TransformRotationDeg_(
            vec_predict[i] + o_expand, o_vehicle.angle_deg + o_vehicle.f_angular_speed_z_degs * f_predict_sec_ / 2);
        float f_x = (vec_predict[i].GetXm() + o_vehicle.x_m - o_robot.f_robot_pos_x_m);
        float f_y = (vec_predict[i].GetYm() + o_vehicle.y_m - o_robot.f_robot_pos_y_m);
        float f_rad = -o_robot.f_robot_pos_deg * DEGtoRAD;
        // cout<<"fx "<<f_x<<" fy "<<f_y<<" // "<<f_rad<<endl;
        // cout<<"o_robot.f_robot_pos_x_m "<<o_robot.f_robot_pos_x_m<<" y "<<o_robot.f_robot_pos_y_m<<endl;
        vec_predict[i].SetXm(f_x * cos(f_rad) - f_y * sin(f_rad));
        vec_predict[i].SetYm(f_x * sin(f_rad) + f_y * cos(f_rad));

        // cout<<i<<" x "<<vec_point[i].GetXm()<<" / "<<vec_point[i].GetYm()<<endl;
        vec_point.emplace_back(vec_predict[i]);
    }

    for (int i = 0; i < vec_point.size(); i++) {
        std::vector<Pos> vec_temp_path;
        if (i == 3)
            vec_temp_path = CoreCalculator::GetRayPosToTarget_(vec_point[i], vec_point[0], 0.05);
        else if (i == 7)
            vec_temp_path = CoreCalculator::GetRayPosToTarget_(vec_point[i], vec_point[4], 0.05);
        else
            vec_temp_path = CoreCalculator::GetRayPosToTarget_(vec_point[i], vec_point[i + 1], 0.05);

        for (int j = 0; j < vec_temp_path.size(); j++) {
            geometry_msgs::Point32 o_point;
            o_point.x = vec_temp_path[j].GetXm();
            o_point.y = vec_temp_path[j].GetYm();
            msg_point.points.emplace_back(o_point);
        }
        if (i < 4) {
            std::vector<Pos> vec_temp_path;
            vec_temp_path = CoreCalculator::GetRayPosToTarget_(vec_point[i], vec_point[i + 4], 0.05);
            for (int k = 0; k < vec_temp_path.size(); k++) {
                geometry_msgs::Point32 o_point;
                o_point.x = vec_temp_path[k].GetXm();
                o_point.y = vec_temp_path[k].GetYm();
                msg_point.points.emplace_back(o_point);
            }
        }
    }
}

void V2V_SAFER::ReceivedNaviinfo(const core_msgs::NavicoreStatus::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    o_navi_msg_ = *msg;
}

void V2V_SAFER::ReceivedRSRinfo(const core_msgs::Vehicle::ConstPtr& msg)
{
    if (msg->x_m == 0 && msg->y_m == 0 && msg->angle_deg == 0) return;

    std::lock_guard<std::mutex> lock(mtx_map_);
    if (map_vehicle_.find(msg->id) == map_vehicle_.end()) {
        std::cout << msg->id << " Key not found" << std::endl;
        VehicleInfo o_tmp;
        map_vehicle_.insert(make_pair(msg->id, o_tmp));
    }

    map_vehicle_[msg->id].id = msg->id;
    map_vehicle_[msg->id].update_time = std::chrono::system_clock::now();  // update time
    map_vehicle_[msg->id].x_m = msg->x_m;
    map_vehicle_[msg->id].y_m = msg->y_m;
    map_vehicle_[msg->id].angle_deg = msg->angle_deg;
    map_vehicle_[msg->id].f_linear_speed_x_ms = msg->f_linear_speed_x_ms;
    map_vehicle_[msg->id].f_linear_speed_y_ms = msg->f_linear_speed_y_ms;
    map_vehicle_[msg->id].f_angular_speed_z_degs = msg->f_angular_speed_z_degs;
    map_vehicle_[msg->id].f_robot_size_front_m = msg->f_robot_size_front_m;
    map_vehicle_[msg->id].f_robot_size_rear_m = msg->f_robot_size_rear_m;
    map_vehicle_[msg->id].f_robot_size_left_m = msg->f_robot_size_left_m;
    map_vehicle_[msg->id].f_robot_size_right_m = msg->f_robot_size_right_m;
}

void V2V_SAFER::ReceivedV2Vinfo(const core_msgs::VehicleList::ConstPtr& msg)
{
    // cout<<"ReceivedV2Vinfo"<<endl;
    // copy data
    for (int i = 0; i < msg->data.size(); i++) {
        if (msg->data[i].x_m == 0 && msg->data[i].y_m == 0 && msg->data[i].angle_deg == 0)
            continue;

        std::lock_guard<std::mutex> lock(mtx_map_);
        if (map_vehicle_.find(msg->data[i].id) == map_vehicle_.end()) {
            std::cout << msg->data[i].id << " Key not found" << std::endl;
            VehicleInfo o_tmp;
            map_vehicle_.insert(make_pair(msg->data[i].id, o_tmp));
        }

        map_vehicle_[msg->data[i].id].id = msg->data[i].id;
        map_vehicle_[msg->data[i].id].update_time = std::chrono::system_clock::now();  // update time
        map_vehicle_[msg->data[i].id].x_m = msg->data[i].x_m;
        map_vehicle_[msg->data[i].id].y_m = msg->data[i].y_m;
        map_vehicle_[msg->data[i].id].angle_deg = msg->data[i].angle_deg;
        map_vehicle_[msg->data[i].id].f_linear_speed_x_ms = msg->data[i].f_linear_speed_x_ms;
        map_vehicle_[msg->data[i].id].f_linear_speed_y_ms = msg->data[i].f_linear_speed_y_ms;
        map_vehicle_[msg->data[i].id].f_angular_speed_z_degs = msg->data[i].f_angular_speed_z_degs;
        map_vehicle_[msg->data[i].id].f_robot_size_front_m = msg->data[i].f_robot_size_front_m;
        map_vehicle_[msg->data[i].id].f_robot_size_rear_m = msg->data[i].f_robot_size_rear_m;
        map_vehicle_[msg->data[i].id].f_robot_size_left_m = msg->data[i].f_robot_size_left_m;
        map_vehicle_[msg->data[i].id].f_robot_size_right_m = msg->data[i].f_robot_size_right_m;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_v2v_safer");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    V2V_SAFER nc_v2v_safer(nh, nhp);
    ros::spin();

    return 0;
}
