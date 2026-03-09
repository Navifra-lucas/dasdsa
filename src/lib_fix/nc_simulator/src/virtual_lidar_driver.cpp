#include "virtual_lidar_driver.hpp"

#include "core/util/logger.hpp"
using namespace std;
namespace NaviFra {
VirtualLidarDriver::VirtualLidarDriver(ros::NodeHandle* nh)
    : nh_(*nh)
{
    b_map_loaded_ = false;
    boost::this_thread::sleep(boost::posix_time::millisec(1000));
    resetDefault();
    readLastPos();

    map_db_sub_ = nh_.subscribe("map/db", 1, &VirtualLidarDriver::RecvDB, this);

    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 50);
    odom_time_ = std::chrono::steady_clock::now();
    ros_robot_pos_sub_ = nh_.subscribe("/odom", 1000, &VirtualLidarDriver::RecvRobotoPos, this, ros::TransportHints().tcpNoDelay(true));
    initial_pose_sub_ = nh_.subscribe("initialpose", 2, &VirtualLidarDriver::RecvInitialPos, this);
    initial_pose_sub2_ = nh_.subscribe("initialpose_sim", 2, &VirtualLidarDriver::RecvInitialPos_sim, this);
    obstacle_pose_sub_ = nh_.subscribe("/obstacle_info", 2, &VirtualLidarDriver::RecvObsPos, this);
    sub_param_ = nh_.subscribe("navifra/param_update", 10, &VirtualLidarDriver::ParamUpdateCallback, this);

    // tf2_ros::Buffer buffer;
    // tf2_ros::TransformListener tfl(buffer);
    // ros::Duration timeout(1.0);
    // try {
    //     tf_geom_ = buffer.lookupTransform("laser_link", header_frame_, ros::Time(0), timeout);

    //     st_scan_.st_extrinsic_parameter.f_x_m = tf_geom_.transform.translation.x;  // 센서 부착위치 X offset
    //     st_scan_.st_extrinsic_parameter.f_y_m = tf_geom_.transform.translation.y;  // 센서 부착위치 Y offset
    //     st_scan_.st_extrinsic_parameter.f_z_m = tf_geom_.transform.translation.z;  // 센서 부착위치 Z offset

    //     // ros rf quaternion을 변경해서 센서데이터에 입력
    //     NaviFra::Pos oPos;
    //     oPos.SetQuaternion(
    //         tf_geom_.transform.rotation.w, tf_geom_.transform.rotation.x, tf_geom_.transform.rotation.y, tf_geom_.transform.rotation.z);

    //     st_scan_.st_extrinsic_parameter.f_roll_deg = oPos.GetRollDeg();  // 센서 부착위치 ROLL
    //     st_scan_.st_extrinsic_parameter.f_pitch_deg = oPos.GetPitchDeg();  // 센서 부착위치 PITCH
    //     st_scan_.st_extrinsic_parameter.f_yaw_deg = oPos.GetDeg();  // 센서 부착위치 YAW
    // }
    // catch (tf2::TransformException& e) {
    // }
    
    st_scan_.st_extrinsic_parameter.f_z_m = 0.0;  // 센서 부착위치 Z offset
    st_scan_.st_extrinsic_parameter.f_roll_deg = 0.0;  // 센서 부착위치 ROLL
    st_scan_.st_extrinsic_parameter.f_pitch_deg = 0.0;  // 센서 부착위치 PITCH
    
    if(header_frame_ == "front_laser")
    {
        NLOG(info) << "front_laser";
        ros::param::param<float>("lidar/front_x_m", st_scan_.st_extrinsic_parameter.f_x_m, 0);
        ros::param::param<float>("lidar/front_y_m", st_scan_.st_extrinsic_parameter.f_y_m, 0);
        ros::param::param<float>("lidar/front_yaw_deg", st_scan_.st_extrinsic_parameter.f_yaw_deg, 0);
    }
    else if(header_frame_ == "rear_laser")
    {
        NLOG(info) << "rear_laser";
        ros::param::param<float>("lidar/rear_x_m", st_scan_.st_extrinsic_parameter.f_x_m, 0);
        ros::param::param<float>("lidar/rear_y_m", st_scan_.st_extrinsic_parameter.f_y_m, 0);
        ros::param::param<float>("lidar/rear_yaw_deg", st_scan_.st_extrinsic_parameter.f_yaw_deg, 0);
    }
    else if(header_frame_ == "left_laser")
    {
        NLOG(info) << "left_laser";
        ros::param::param<float>("lidar/left_x_m", st_scan_.st_extrinsic_parameter.f_x_m, 0);
        ros::param::param<float>("lidar/left_y_m", st_scan_.st_extrinsic_parameter.f_y_m, 0);
        ros::param::param<float>("lidar/left_yaw_deg", st_scan_.st_extrinsic_parameter.f_yaw_deg, 0);
    }
    else if(header_frame_ == "right_laser")
    {
        NLOG(info) << "right_laser";
        ros::param::param<float>("lidar/right_x_m", st_scan_.st_extrinsic_parameter.f_x_m, 0);
        ros::param::param<float>("lidar/right_y_m", st_scan_.st_extrinsic_parameter.f_y_m, 0);
        ros::param::param<float>("lidar/right_yaw_deg", st_scan_.st_extrinsic_parameter.f_yaw_deg, 0);
    }
    else
    {
        st_scan_.st_extrinsic_parameter.f_x_m = 0.0;  // 센서 부착위치 X offset
        st_scan_.st_extrinsic_parameter.f_y_m = 0.0;  // 센서 부착위치 Y offset
        st_scan_.st_extrinsic_parameter.f_z_m = 0.0;  // 센서 부착위치 Z offset

        st_scan_.st_extrinsic_parameter.f_roll_deg = 0.0;  // 센서 부착위치 ROLL
        st_scan_.st_extrinsic_parameter.f_pitch_deg = 0.0;  // 센서 부착위치 PITCH
        st_scan_.st_extrinsic_parameter.f_yaw_deg = 0.0;  // 센서 부착위치 YAW
    }
    laser_timer_ = nh_.createTimer(ros::Duration(laser_scan_.scan_time), boost::bind(&VirtualLidarDriver::Read, this, _1));
}

VirtualLidarDriver::~VirtualLidarDriver()
{
}

void VirtualLidarDriver::readLastPos()
{
    try {
        std::vector<float> vec_pos;
        std::string file_name = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/pose.txt";
        std::ifstream in(file_name);

        if (in.is_open()) {
            string line;
            while (getline(in, line)) {
                // NLOG(info) << "lastpos : " << line;
                vec_pos.emplace_back(stof(line));
            }
        }
        else {
            NLOG(severity_level::error) << "파일을 찾을 수 없습니다.";
        }
        // NLOG(info) << "vec_pos.size() " << vec_pos.size();
        if (vec_pos.size() == 3) {
            NaviFra::Pos o_robot(vec_pos[0], vec_pos[1], vec_pos[2]);
            o_robot_pos_ = o_robot;
            // NLOG(info) << "SetRobotPose init success " << vec_pos.size();
        }
        else {
            NLOG(severity_level::error) << "pos가 3개가 아닙니다.";
            NaviFra::Pos o_robot(0, 0, 0);
            o_robot_pos_ = o_robot;
        }
    }
    catch (...) {
        NLOG(severity_level::error) << "POS read error";
    }
}

void VirtualLidarDriver::ParamUpdateCallback(const std_msgs::String::ConstPtr& msg)
{
    if(header_frame_ == "front_laser")
    {
        NLOG(info) << "front_laser";
        ros::param::param<float>("lidar/front_x_m", st_scan_.st_extrinsic_parameter.f_x_m, 0);
        ros::param::param<float>("lidar/front_y_m", st_scan_.st_extrinsic_parameter.f_y_m, 0);
        ros::param::param<float>("lidar/front_yaw_deg", st_scan_.st_extrinsic_parameter.f_yaw_deg, 0);
    }
    else if(header_frame_ == "rear_laser")
    {
        NLOG(info) << "rear_laser";
        ros::param::param<float>("lidar/rear_x_m", st_scan_.st_extrinsic_parameter.f_x_m, 0);
        ros::param::param<float>("lidar/rear_y_m", st_scan_.st_extrinsic_parameter.f_y_m, 0);
        ros::param::param<float>("lidar/rear_yaw_deg", st_scan_.st_extrinsic_parameter.f_yaw_deg, 0);
    }
    else if(header_frame_ == "left_laser")
    {
        NLOG(info) << "left_laser";
        ros::param::param<float>("lidar/left_x_m", st_scan_.st_extrinsic_parameter.f_x_m, 0);
        ros::param::param<float>("lidar/left_y_m", st_scan_.st_extrinsic_parameter.f_y_m, 0);
        ros::param::param<float>("lidar/left_yaw_deg", st_scan_.st_extrinsic_parameter.f_yaw_deg, 0);
    }
    else if(header_frame_ == "right_laser")
    {
        NLOG(info) << "right_laser";
        ros::param::param<float>("lidar/right_x_m", st_scan_.st_extrinsic_parameter.f_x_m, 0);
        ros::param::param<float>("lidar/right_y_m", st_scan_.st_extrinsic_parameter.f_y_m, 0);
        ros::param::param<float>("lidar/right_yaw_deg", st_scan_.st_extrinsic_parameter.f_yaw_deg, 0);
    }
    else
    {
        st_scan_.st_extrinsic_parameter.f_x_m = 0.0;  // 센서 부착위치 X offset
        st_scan_.st_extrinsic_parameter.f_y_m = 0.0;  // 센서 부착위치 Y offset
        st_scan_.st_extrinsic_parameter.f_z_m = 0.0;  // 센서 부착위치 Z offset

        st_scan_.st_extrinsic_parameter.f_roll_deg = 0.0;  // 센서 부착위치 ROLL
        st_scan_.st_extrinsic_parameter.f_pitch_deg = 0.0;  // 센서 부착위치 PITCH
        st_scan_.st_extrinsic_parameter.f_yaw_deg = 0.0;  // 센서 부착위치 YAW
    }
}

void VirtualLidarDriver::RecvObsPos(const std_msgs::String msg)
{
    // NLOG(info) << msg.data;
    string line = msg.data;
    string delim = "/";
    vector<string> words{};

    size_t pos = 0;
    while ((pos = line.find(delim)) != string::npos) {
        words.push_back(line.substr(0, pos));
        line.erase(0, pos + delim.length());
    }
    if (words.size() == 5) {
        o_obs_info_.b_active = true;
        o_obs_info_.f_pos_x = stof(words[0]);
        o_obs_info_.f_pos_y = stof(words[1]);
        o_obs_info_.f_height = stof(words[2]);
        o_obs_info_.f_width = stof(words[3]);
        o_obs_info_.f_angle_deg = stof(words[4]);
    }
    // for (const auto& w : words) {
    //     NLOG(info) << w << endl;
    // }
    // LOG_INFO("RecvObsPos");
}

void VirtualLidarDriver::RecvInitialPos(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    geometry_msgs::PoseStamped ps_robot_pose;
    ps_robot_pose.header.frame_id = "map";
    ps_robot_pose.pose = msg.pose.pose;
    o_initial_pos_ = PosConvert(ps_robot_pose);
    o_robot_pos_ = o_initial_pos_;

    // LOG_INFO("Recveived initial robot pos");
}

void VirtualLidarDriver::RecvInitialPos_sim(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    geometry_msgs::PoseStamped ps_robot_pose;
    ps_robot_pose.header.frame_id = "map";
    ps_robot_pose.pose = msg.pose.pose;
    NaviFra::Pos tmp = PosConvert(ps_robot_pose);
    o_robot_pos_.SetXm(o_robot_pos_.GetXm() + tmp.GetXm());
    o_robot_pos_.SetYm(o_robot_pos_.GetYm() + tmp.GetYm());
    o_robot_pos_.SetDeg(o_robot_pos_.GetDeg() + tmp.GetDeg());

    // LOG_INFO("Recveived initial sim robot pos");
}

void VirtualLidarDriver::RecvDB(const core_msgs::MapDB::ConstPtr& msg)
{
    ros::param::param<bool>("localization/b_use_localization", b_use_localization_, true);
    // NLOG(info) << "b_use_localization_ " << b_use_localization_;
    if (b_use_localization_ == false)
        return;
    // std::lock_guard<std::mutex> lock(mtx_map_lock_);
    b_map_loaded_ = false;

    f_map_min_x_ = msg->map_min_x;
    f_map_min_y_ = msg->map_min_y;
    f_map_max_x_ = msg->map_max_x;
    f_map_max_y_ = msg->map_max_y;

    if (abs(f_map_max_x_ - f_map_min_x_) > 200 || abs(f_map_max_y_ - f_map_min_y_) > 200) {
        f_map_resolution_ = 0.04;
    }
    else if (abs(f_map_max_x_ - f_map_min_x_) > 60 || abs(f_map_max_y_ - f_map_min_y_) > 60) {
        f_map_resolution_ = 0.02;
    }
    else {
        f_map_resolution_ = 0.01;
    }

    o_raymarch_.Initialize(msg, 30);
    b_map_loaded_ = true;

    LOG_INFO("RecvDB Received all map from map server!!");
}

void VirtualLidarDriver::resetDefault()
{
    private_nh_ = ros::NodeHandle("~");
    private_nh_.param<int>("scan_size", scan_size_, 2700);  // Default min value
    private_nh_.param<string>("scan_frame", header_frame_, "world");
    laser_scan_.header.frame_id = header_frame_;
    private_nh_.param<float>("angle_min", laser_scan_.angle_min, -2.35619449);  // Default min value
    private_nh_.param<float>("angle_max", laser_scan_.angle_max, 2.35619449);  // Default min value
    laser_scan_.angle_increment = (laser_scan_.angle_max - laser_scan_.angle_min) / (float)scan_size_;  // default max resolution
    private_nh_.param<float>("scan_time", laser_scan_.scan_time, 0.04);  // Default
    private_nh_.param<float>("range_min", laser_scan_.range_min, 0.001);  // default
    private_nh_.param<float>("range_max", laser_scan_.range_max, 65.0);  // Max range 65m
    laser_scan_.ranges.resize(0);
    laser_scan_.ranges.resize(scan_size_);
    laser_scan_.intensities.resize(0);
    laser_scan_.intensities.resize(scan_size_);
    // LOG_INFO("Header frame : %s", header_frame_.c_str());
    // ROS_INFO_STREAM("[Laser Scanner] Reset data");
}

void VirtualLidarDriver::RecvRobotoPos(const nav_msgs::Odometry::ConstPtr& msg)
{
    NaviFra::Pos o_odom_pos = PosConvert(msg);

    float f_cos = cosf(o_prev_odom_pos_.GetRad());
    float f_sin = sinf(o_prev_odom_pos_.GetRad());
    float f_dx = o_odom_pos.GetXm() - o_prev_odom_pos_.GetXm();
    float f_dy = o_odom_pos.GetYm() - o_prev_odom_pos_.GetYm();
    NaviFra::Pos o_del_odom_pos(f_cos * f_dx + f_sin * f_dy, -f_sin * f_dx + f_cos * f_dy, o_odom_pos.GetDeg() - o_prev_odom_pos_.GetDeg());

    {
        std::lock_guard<std::mutex> lock(mtx_lock_);
        odom_time_ = std::chrono::steady_clock::now();
        o_robot_pos_ = ConvertToGlobalPosition_(o_del_odom_pos, o_robot_pos_);
        vec_pos_history_.emplace_back(o_robot_pos_);
        if (vec_pos_history_.size() > 2) {
            vec_pos_history_.erase(vec_pos_history_.begin());
        }
    }
    o_prev_odom_pos_ = o_odom_pos;
}

NaviFra::Pos VirtualLidarDriver::ConvertToGlobalPosition_(const NaviFra::Pos& o_local_pos, const NaviFra::Pos& o_origin_pos)
{
    NaviFra::Pos o_result_pos;
    o_result_pos.SetDeg(o_origin_pos.GetDeg() + o_local_pos.GetDeg());

    float f_x_m =
        o_origin_pos.GetXm() + o_local_pos.GetXm() * cos(o_origin_pos.GetRad()) - o_local_pos.GetYm() * sin(o_origin_pos.GetRad());
    float f_y_m =
        o_origin_pos.GetYm() + o_local_pos.GetXm() * sin(o_origin_pos.GetRad()) + o_local_pos.GetYm() * cos(o_origin_pos.GetRad());
    float f_z_m = o_local_pos.GetZm();

    o_result_pos.SetXm(f_x_m);
    o_result_pos.SetYm(f_y_m);
    o_result_pos.SetZm(f_z_m);
    return o_result_pos;
}
float VirtualLidarDriver::WrapAnglePiToPiDeg_(float f_angle_deg)
{
    float f_deg = fmod((f_angle_deg + ToSign_(f_angle_deg) * 180.0), 360.0);
    return f_deg - ToSign_(f_angle_deg) * 180.0;
}
void VirtualLidarDriver::Read(const ros::TimerEvent& event)
{
    static float f_length = 0.f;

    NaviFra::Pos o_start_pos_;
    {
        const std::lock_guard<std::mutex> lock(mtx_lock_);
        std::chrono::duration<double> sec = std::chrono::steady_clock::now() - odom_time_;
        if (vec_pos_history_.size() > 1) {
            o_start_pos_ = o_robot_pos_;
            NaviFra::Pos o_d_pos = vec_pos_history_[1] - vec_pos_history_[0];
            float f_time_ratio = sec.count() / 0.02;
            o_start_pos_.SetXm(o_start_pos_.GetXm() + o_d_pos.GetXm() * f_time_ratio);
            o_start_pos_.SetYm(o_start_pos_.GetYm() + o_d_pos.GetYm() * f_time_ratio);
            o_start_pos_.SetDeg(o_start_pos_.GetDeg() + WrapAnglePiToPiDeg_(o_d_pos.GetDeg()) * f_time_ratio);
            // NLOG(info)<<"ERROR f_time_ratio "<<f_time_ratio<<" "<<o_d_pos.GetXm()*f_time_ratio;
        }
        else {
            return;
        }
    }

    if (b_map_loaded_) {
        // NLOG(info)<<"Read";
        // std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        laser_scan_.header.stamp = ros::Time::now();
        laser_scan_.time_increment = f_length;
        float fAngleRad = 0.f;
        float f_robot_angle = o_start_pos_.GetRad();

        float f_point_x_m = o_start_pos_.GetXm() + st_scan_.st_extrinsic_parameter.f_x_m * cos(f_robot_angle) -
            st_scan_.st_extrinsic_parameter.f_y_m * sin(f_robot_angle);
        float f_point_y_m = o_start_pos_.GetYm() + st_scan_.st_extrinsic_parameter.f_x_m * sin(f_robot_angle) +
            st_scan_.st_extrinsic_parameter.f_y_m * cos(f_robot_angle);
        for (size_t i = 0; i < scan_size_; i++) {
            if (st_scan_.st_extrinsic_parameter.f_roll_deg > 175 || st_scan_.st_extrinsic_parameter.f_roll_deg < -175) {
                fAngleRad =
                    (st_scan_.st_extrinsic_parameter.f_yaw_deg * DEGtoRAD + (laser_scan_.angle_max - i * laser_scan_.angle_increment));
            }
            else {
                fAngleRad =
                    (st_scan_.st_extrinsic_parameter.f_yaw_deg * DEGtoRAD + (laser_scan_.angle_min + i * laser_scan_.angle_increment));
            }
            int n_x = (int)((f_point_x_m - f_map_min_x_) / f_map_resolution_);
            int n_y = (int)((f_point_y_m - f_map_min_y_) / f_map_resolution_);

            {
                // std::lock_guard<std::mutex> lock(mtx_map_lock_);
                laser_scan_.ranges[i] = o_raymarch_.CalcPointLength(n_x, n_y, f_robot_angle + fAngleRad);
            }
        }
        pub_scan_.publish(laser_scan_);
        // std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start;
        // printf("%f ms\n",sec.count()*1000);
    }
    else if (b_use_localization_ == false) {
        laser_scan_.header.stamp = ros::Time::now();
        pub_scan_.publish(laser_scan_);
    }
}

NaviFra::Pos VirtualLidarDriver::PosConvert(const geometry_msgs::PoseStamped& input)
{
    NaviFra::Pos o_pos;
    o_pos.SetXm(input.pose.position.x);
    o_pos.SetYm(input.pose.position.y);
    o_pos.SetQuaternion(input.pose.orientation.w, input.pose.orientation.x, input.pose.orientation.y, input.pose.orientation.z);
    return o_pos;
}

NaviFra::Pos VirtualLidarDriver::PosConvert(const nav_msgs::Odometry::ConstPtr& input)
{
    NaviFra::Pos o_pos;
    o_pos.SetXm(input->pose.pose.position.x);
    o_pos.SetYm(input->pose.pose.position.y);
    o_pos.SetQuaternion(
        input->pose.pose.orientation.w, input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z);
    return o_pos;
}

}  // namespace NaviFra

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_lidar");
    ros::NodeHandle n;
    NaviFra::VirtualLidarDriver lidardriver(&n);
    ros::spin();
    return 0;
}
