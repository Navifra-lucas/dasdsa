#include "nc_lidar_merger.hpp"

#include "core/util/logger.hpp"
#include "tf2/utils.h"

using namespace std;
namespace NaviFra {
LidarMergerFilter::LidarMergerFilter(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    tf_.reset(new tf2_ros::Buffer());
    tfl_.reset(new tf2_ros::TransformListener(*tf_));

    quality_service_ = nh.advertiseService("scan_quality_service", &LidarMergerFilter::CQcallback, this);

    sub_param_ = nh_.subscribe("navifra/param_update", 10, &LidarMergerFilter::ParamUpdateCallback, this);

    pub_scan1_ = nh_.advertise<sensor_msgs::PointCloud>("front_cloud", 5);
    pub_scan2_ = nh_.advertise<sensor_msgs::PointCloud>("rear_cloud", 5);
    pub_scan3_ = nh_.advertise<sensor_msgs::PointCloud>("left_cloud", 5);
    pub_scan4_ = nh_.advertise<sensor_msgs::PointCloud>("right_cloud", 5);

    pub_scan1_global_ = nh_.advertise<sensor_msgs::PointCloud>("front_cloud_global", 5);
    pub_scan2_global_ = nh_.advertise<sensor_msgs::PointCloud>("rear_cloud_global", 5);
    pub_scan3_global_ = nh_.advertise<sensor_msgs::PointCloud>("left_cloud_global", 5);
    pub_scan4_global_ = nh_.advertise<sensor_msgs::PointCloud>("right_cloud_global", 5);

    pub_merge_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan_merge", 5);

    sub_speed_ = nh_.subscribe("odom", 10, &LidarMergerFilter::SpeedCallback, this, ros::TransportHints().tcpNoDelay(true));
    sub_pose_ = nh_.subscribe("localization/robot_pos", 10, &LidarMergerFilter::PosCallback, this, ros::TransportHints().tcpNoDelay(true));
    sub_obstacle_ = nh_.subscribe("virtual_obs", 10, &LidarMergerFilter::VirtualObsCallback, this);
    sub_loaded_ = nh_.subscribe("navifra/loaded", 10, &LidarMergerFilter::LoadedCallback, this);
    sub_fork_position_ = nh_.subscribe("fork_position_state", 10, &LidarMergerFilter::ForkPositionCallback, this);

    vec_input_topics.clear();
    Initialize();
}

LidarMergerFilter::~LidarMergerFilter()
{
    LOG_INFO("~LidarMergerFilter start1");

    if (synchronizer2_ != nullptr) {
        delete synchronizer2_;
    }
    LOG_INFO("~LidarMergerFilter start2");

    if (synchronizer3_ != nullptr) {
        delete synchronizer3_;
    }
    LOG_INFO("~LidarMergerFilter start3");

    if (synchronizer4_ != nullptr) {
        delete synchronizer4_;
    }
    LOG_INFO("~LidarMergerFilter start4");

    int n_size = message_filter_subscribers_.size();
    for (int i = 0; i < n_size; i++) {
        if (message_filter_subscribers_[i] != nullptr) {
            delete message_filter_subscribers_[i];
        }
    }
    LOG_INFO("~LidarMergerFilter finish");
}

void LidarMergerFilter::Initialize()
{
    ros::param::param<string>("lidarmerger/scan_destination_topic", s_target_scan_topic_, std::string(""));
    ros::param::param<string>("lidarmerger/destination_frame", s_target_frame_id, std::string("/base_link"));
    if (nh_.getNamespace() != "/") {
        s_target_frame_id = nh_.getNamespace() + "/" + s_target_frame_id;

        LOG_INFO("namespace is %s", nh_.getNamespace().c_str());
    }
    else {
        LOG_INFO("namespace is empty");
    }
    ros::param::param<float>("lidarmerger/dist_min", f_dist_min_m_, 0.1);
    ros::param::param<float>("lidarmerger/dist_max", f_dist_max_m_, 28);

    ros::param::param<float>("lidarmerger/x_front_m", f_x_front_m_, 1.0);
    ros::param::param<float>("lidarmerger/x_back_m", f_x_back_m_, -1.0);
    ros::param::param<float>("lidarmerger/y_left_m", f_y_left_m_, 1.0);
    ros::param::param<float>("lidarmerger/y_right_m", f_y_right_m_, -1.0);
    ros::param::param<bool>("lidarmerger/b_load_diff_range_use", b_load_diff_range_use_, false);
    ros::param::param<float>("lidarmerger/load_x_front_m", f_x_load_front_m_, 1.0);
    ros::param::param<float>("lidarmerger/load_x_back_m", f_x_load_back_m_, -1.0);
    ros::param::param<float>("lidarmerger/load_y_left_m", f_y_load_left_m_, 1.0);
    ros::param::param<float>("lidarmerger/load_y_right_m", f_y_load_right_m_, -1.0);

    ros::param::param<float>("lidarmerger/lidar_merge_range_diff", f_lidar_merge_range_diff_, 0.05);
    ros::param::param<float>("lidarmerger/lidar_max_gap_msec", f_max_msec_, 30);
    ros::param::param<float>("lidarmerger/lidar_merger_time_bound", f_time_bound_, 0.04);

    ros::param::param<float>("lidarmerger/lidar_timeout_threshold_", f_lidar_timeout_threshold_, 1.0);
    ros::param::param<int>("lidarmerger/max_lidar_num", n_max_lidar_num_, 4);
    ros::param::param<int>("lidarmerger/lidar_ground_filter", n_lidar_temp_, 0);

    ros::param::param<int>("lidarmerger/n_full_lidar_m", n_full_lidar_m_, 2.5);
    ros::param::param<int>("lidarmerger/n_skip_lidar_std", n_skip_lidar_std_, 10);

    ros::param::param<bool>("lidarmerger/b_cal_on", b_cal_on_, true);

    ros::param::param<float>("lidar/front_x_m", st_offset_.front_x_m_, 0);
    ros::param::param<float>("lidar/front_y_m", st_offset_.front_y_m_, 0);
    ros::param::param<float>("lidar/front_z_m", st_offset_.front_z_m_, 0);
    ros::param::param<float>("lidar/front_roll_deg", st_offset_.front_roll_deg_, 0);
    ros::param::param<float>("lidar/front_pitch_deg", st_offset_.front_pitch_deg_, 0);
    ros::param::param<float>("lidar/front_yaw_deg", st_offset_.front_yaw_deg_, 0);
    ros::param::param<float>("lidar/front_angle_increment", st_offset_.front_angle_increment_, 0);
    ros::param::param<float>("lidar/front_angle_min", st_offset_.front_angle_min_, 0);
    ros::param::param<float>("lidar/front_range_offset_m", st_offset_.front_range_offset_m_, 0);
    ros::param::param<float>("lidar/rear_x_m", st_offset_.rear_x_m_, 0);
    ros::param::param<float>("lidar/rear_y_m", st_offset_.rear_y_m_, 0);
    ros::param::param<float>("lidar/rear_z_m", st_offset_.rear_z_m_, 0);
    ros::param::param<float>("lidar/rear_roll_deg", st_offset_.rear_roll_deg_, 0);
    ros::param::param<float>("lidar/rear_pitch_deg", st_offset_.rear_pitch_deg_, 0);
    ros::param::param<float>("lidar/rear_yaw_deg", st_offset_.rear_yaw_deg_, 0);
    ros::param::param<float>("lidar/rear_angle_increment", st_offset_.rear_angle_increment_, 0);
    ros::param::param<float>("lidar/rear_angle_min", st_offset_.rear_angle_min_, 0);
    ros::param::param<float>("lidar/rear_range_offset_m", st_offset_.rear_range_offset_m_, 0);
    ros::param::param<float>("lidar/left_x_m", st_offset_.left_x_m_, 0);
    ros::param::param<float>("lidar/left_y_m", st_offset_.left_y_m_, 0);
    ros::param::param<float>("lidar/left_z_m", st_offset_.left_z_m_, 0);
    ros::param::param<float>("lidar/left_roll_deg", st_offset_.left_roll_deg_, 0);
    ros::param::param<float>("lidar/left_pitch_deg", st_offset_.left_pitch_deg_, 0);
    ros::param::param<float>("lidar/left_yaw_deg", st_offset_.left_yaw_deg_, 0);
    ros::param::param<float>("lidar/left_angle_increment", st_offset_.left_angle_increment_, 0);
    ros::param::param<float>("lidar/left_angle_min", st_offset_.left_angle_min_, 0);
    ros::param::param<float>("lidar/left_range_offset_m", st_offset_.left_range_offset_m_, 0);
    ros::param::param<float>("lidar/right_x_m", st_offset_.right_x_m_, 0);
    ros::param::param<float>("lidar/right_y_m", st_offset_.right_y_m_, 0);
    ros::param::param<float>("lidar/right_z_m", st_offset_.right_z_m_, 0);
    ros::param::param<float>("lidar/right_roll_deg", st_offset_.right_roll_deg_, 0);
    ros::param::param<float>("lidar/right_pitch_deg", st_offset_.right_pitch_deg_, 0);
    ros::param::param<float>("lidar/right_yaw_deg", st_offset_.right_yaw_deg_, 0);
    ros::param::param<float>("lidar/right_angle_increment", st_offset_.right_angle_increment_, 0);
    ros::param::param<float>("lidar/right_angle_min", st_offset_.right_angle_min_, 0);
    ros::param::param<float>("lidar/right_range_offset_m", st_offset_.right_range_offset_m_, 0);

    ros::param::get("lidarmerger/input_scans", vec_input_topics);
    ros::param::get("lidarmerger/lidar_timeout_threshold", f_lidar_timeout_threshold_);
    ros::param::get("lidarmerger/max_lidar_num", n_max_lidar_num_);

    if (b_param_init_) {
        b_offset_init_ = false;
        return;
    }
    NLOG(info) << f_lidar_timeout_threshold_;
    vec_lidar_info_.resize(n_max_lidar_num_);
    vec_tp_lidar_timestamp_.resize(n_max_lidar_num_);
    vec_f_dist_.resize(vec_input_topics.size(), vector<float>(9, 0));

    NLOG(info) << "s_target_scan_topic_ " << s_target_scan_topic_;
    pub_scan_ = nh_.advertise<sensor_msgs::PointCloud>(s_target_scan_topic_, 1);

    d_scan_front_time_diff_ = ros::Time::now().toSec();
    d_scan_rear_time_diff_ = ros::Time::now().toSec();
    target_frame_front_id_ = "";
    b_lidar_direction_ = true;
    DefineCallbackFunction(vec_input_topics);
    vec_target_frame_id_.resize(vec_input_topics.size());
    vec_f_increment_.resize(vec_input_topics.size(), 0);
    vec_f_angle_min_.resize(vec_input_topics.size(), 0);
    vec_range_offset_m_.resize(vec_input_topics.size(), 0);

    b_offset_init_ = false;
    b_param_init_ = true;
    b_lidar_update_ = true;
}

void LidarMergerFilter::DefineCallbackFunction(const vector<string>& vec_input_topics)
{
    message_filter_subscribers_.clear();
    // CHECK(static_cast<int>(vec_input_topics.size()) <= 4) << "NOT SUPPORT!!";
    vec_check_transform_.resize(vec_input_topics.size(), false);
    for (int i = 0; i < vec_input_topics.size(); i++) {
        vec_tf_listener_.push_back(make_shared<tf::TransformListener>());
    }
    // vec_tf_listener_.resize(vec_input_topics.size());
    for (int i = 0; i < vec_input_topics.size(); i++) {
        LOG_INFO("scan topic name : %s\n", vec_input_topics[i].c_str());
        vec_lidar_info_[i].b_lidar_use = true;
        message_filter_subscribers_.push_back(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, vec_input_topics[i], 1));
    }
    switch (vec_input_topics.size()) {
        case 1:
        {
            sub_scan1_ = nh_.subscribe(
                vec_input_topics[0], 2, &LidarMergerFilter::messageFilterCallback, this, ros::TransportHints().tcpNoDelay(true));
            break;
        }
        case 2:
        {
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;

            synchronizer2_ = new message_filters::Synchronizer<SyncPolicy>(
                SyncPolicy(2), *message_filter_subscribers_[0], *message_filter_subscribers_[1]);

            synchronizer2_->setInterMessageLowerBound(0, ros::Duration(f_time_bound_));
            synchronizer2_->setInterMessageLowerBound(1, ros::Duration(f_time_bound_));
            synchronizer2_->registerCallback(boost::bind(&LidarMergerFilter::messageFilterCallback, this, _1, _2));
            break;
        }
        case 3:
        {
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan>
                SyncPolicy;

            synchronizer3_ = new message_filters::Synchronizer<SyncPolicy>(
                SyncPolicy(3), *message_filter_subscribers_[0], *message_filter_subscribers_[1], *message_filter_subscribers_[2]);

            synchronizer3_->setInterMessageLowerBound(0, ros::Duration(f_time_bound_));
            synchronizer3_->setInterMessageLowerBound(1, ros::Duration(f_time_bound_));
            synchronizer3_->setInterMessageLowerBound(2, ros::Duration(f_time_bound_));
            synchronizer3_->registerCallback(boost::bind(&LidarMergerFilter::messageFilterCallback, this, _1, _2, _3));
            break;
        }
        case 4:
        {
            typedef message_filters::sync_policies::ApproximateTime<
                sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan>
                SyncPolicy;

            synchronizer4_ = new message_filters::Synchronizer<SyncPolicy>(
                SyncPolicy(4), *message_filter_subscribers_[0], *message_filter_subscribers_[1], *message_filter_subscribers_[2],
                *message_filter_subscribers_[3]);

            synchronizer4_->setInterMessageLowerBound(0, ros::Duration(f_time_bound_));
            synchronizer4_->setInterMessageLowerBound(1, ros::Duration(f_time_bound_));
            synchronizer4_->setInterMessageLowerBound(2, ros::Duration(f_time_bound_));
            synchronizer4_->setInterMessageLowerBound(3, ros::Duration(f_time_bound_));
            synchronizer4_->registerCallback(boost::bind(&LidarMergerFilter::messageFilterCallback, this, _1, _2, _3, _4));
            break;
        }
        default:
            break;
    }
    LOG_INFO("%d lidar sensors callback registered\n", vec_input_topics.size());
}
void LidarMergerFilter::SetTransformListener(
    tf::StampedTransform& req_to_trans, float fx, float fy, float fz, float fRx, float fRy, float fRz)
{
    auto xv = req_to_trans.getOrigin();
    auto q = req_to_trans.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    NLOG(info) << "req_to_trans1:  " << xv.x() << ", " << xv.y() << ", " << xv.z() << " ,R: " << roll << ", " << pitch << ", " << yaw;
    m.setRPY(fRx * 3.141592 / 180, fRy * 3.141592 / 180, fRz * 3.141592 / 180);
    m.getRotation(q);
    req_to_trans.setRotation(q);
    tf::Vector3 xz(fx, fy, fz);
    req_to_trans.setOrigin(xz);
    xv = req_to_trans.getOrigin();
    q = req_to_trans.getRotation();
    m.getRPY(roll, pitch, yaw);
    NLOG(info) << "req_to_trans2:  " << xv.x() << ", " << xv.y() << ", " << xv.z() << " ,R: " << roll << ", " << pitch << ", " << yaw;
}

void LidarMergerFilter::TransformListener(tf::StampedTransform& req_to_trans, float fx, float fy, float fz, float fRx, float fRy, float fRz)
{
    auto xv = req_to_trans.getOrigin();
    auto q = req_to_trans.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    NLOG(info) << "req_to_trans1:  " << xv.x() << ", " << xv.y() << ", " << xv.z() << " ,R: " << roll << ", " << pitch << ", " << yaw;
    m.setRPY(roll + fRx, pitch + fRy, yaw + fRz);
    m.getRotation(q);
    req_to_trans.setRotation(q);
    tf::Vector3 xz(xv.x() + fx, xv.y() + fy, xv.z() + fz);
    req_to_trans.setOrigin(xz);
    xv = req_to_trans.getOrigin();
    q = req_to_trans.getRotation();
    m.getRPY(roll, pitch, yaw);
    NLOG(info) << "req_to_trans2:  " << xv.x() << ", " << xv.y() << ", " << xv.z() << " ,R: " << roll << ", " << pitch << ", " << yaw;
}

void LidarMergerFilter::UpdateOffset(const std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scans)
{
    NLOG(info) << "UpdateOffset";
    for (int k = 0; k < vec_tf_listener_.size(); k++) {
        tf::StampedTransform req_to_trans;
        vec_tf_listener_[k]->lookupTransform(s_target_frame_id, current_scans[k]->header.frame_id, ros::Time(0), req_to_trans);

        if (k == 0 && vec_tf_listener_.size() != 1) {  // front lidar
            SetTransformListener(
                req_to_trans, st_offset_.front_x_m_, st_offset_.front_y_m_, st_offset_.front_z_m_, st_offset_.front_roll_deg_,
                st_offset_.front_pitch_deg_, st_offset_.front_yaw_deg_);
            vec_f_increment_[k] = st_offset_.front_angle_increment_;
            vec_f_angle_min_[k] = st_offset_.front_angle_min_;
            vec_range_offset_m_[k] = st_offset_.front_range_offset_m_;
        }
        else if (k == 1) {  // rear lidar
            SetTransformListener(
                req_to_trans, st_offset_.rear_x_m_, st_offset_.rear_y_m_, st_offset_.rear_z_m_, st_offset_.rear_roll_deg_,
                st_offset_.rear_pitch_deg_, st_offset_.rear_yaw_deg_);
            vec_f_increment_[k] = st_offset_.rear_angle_increment_;
            vec_f_angle_min_[k] = st_offset_.rear_angle_min_;
            vec_range_offset_m_[k] = st_offset_.rear_range_offset_m_;
        }
        else if (k == 2) {  // left lidar
            SetTransformListener(
                req_to_trans, st_offset_.left_x_m_, st_offset_.left_y_m_, st_offset_.left_z_m_, st_offset_.left_roll_deg_,
                st_offset_.left_pitch_deg_, st_offset_.left_yaw_deg_);
            vec_f_increment_[k] = st_offset_.left_angle_increment_;
            vec_f_angle_min_[k] = st_offset_.left_angle_min_;
            vec_range_offset_m_[k] = st_offset_.left_range_offset_m_;
        }
        else if (k == 3) {  // right lidar
            SetTransformListener(
                req_to_trans, st_offset_.right_x_m_, st_offset_.right_y_m_, st_offset_.right_z_m_, st_offset_.right_roll_deg_,
                st_offset_.right_pitch_deg_, st_offset_.right_yaw_deg_);
            vec_f_increment_[k] = st_offset_.right_angle_increment_;
            vec_f_angle_min_[k] = st_offset_.right_angle_min_;
            vec_range_offset_m_[k] = st_offset_.right_range_offset_m_;
        }
        vec_tf_listener_[k]->setTransform(req_to_trans);
    }
}

void LidarMergerFilter::VirtualObsCallback(const std_msgs::String::ConstPtr& msg)
{
    istringstream iss(msg->data);  // istringstream에 str을 담는다.
    string buffer;  // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼
    vector<string> result;
    while (getline(iss, buffer, '/')) {
        result.push_back(buffer);  // 절삭된 문자열을 vector에 저장
    }
    if (result.size() == 3) {
        o_obs_info_.o_pos.SetXm(stof(result[0]));
        o_obs_info_.o_pos.SetYm(stof(result[1]));
        o_obs_info_.f_size = stof(result[2]);
    }
}

bool LidarMergerFilter::CQcallback(core_msgs::CommonString::Request& request, core_msgs::CommonString::Response& response)
{
    NLOG(info) << "request.data " << request.data;
    if (request.data == "high") {
        b_full_lidar_ = true;
    }
    else {
        b_full_lidar_ = false;
    }
    response.success = true;
    return true;
}

void LidarMergerFilter::ForkPositionCallback(const std_msgs::Int16::ConstPtr& msg)
{
    int n_fork_position = msg->data;
    n_fork_position_ = n_fork_position;
}

void LidarMergerFilter::LoadedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    b_loaded_ = msg->data;
}

void LidarMergerFilter::ParamUpdateCallback(const std_msgs::String::ConstPtr& msg)
{
    Initialize();
}

void LidarMergerFilter::SpeedCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    checktime_odom_ = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lock(mtx_lock_correct_);
        o_correct_info_.SetXm(msg->twist.twist.linear.x);
        o_correct_info_.SetYm(msg->twist.twist.linear.y);
        o_correct_info_.SetRad(msg->twist.twist.angular.z);
        d_odometry_time_ = msg->header.stamp.toSec();
        nav_odom_ = *msg;
    }
}

void LidarMergerFilter::PosCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - checktime_odom_;
    if (sec.count() > 0.5) {
        std::lock_guard<std::mutex> lock(mtx_lock_correct_);
        o_correct_info_.SetXm(0);
        o_correct_info_.SetYm(0);
        o_correct_info_.SetRad(0);
        d_odometry_time_ = msg->header.stamp.toSec();
    }

    {
        std::lock_guard<std::mutex> lock(mtx_lock_pos_);
        geometry_msgs::PoseStamped ps_robot_pose;
        ps_robot_pose.header.frame_id = "map";
        ps_robot_pose.pose = msg->pose.pose;
        o_pf_pos_ = PosConvert(ps_robot_pose);
    }
}

bool LidarMergerFilter::LidarMerge(const std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scans)
{
    std::vector<sensor_msgs::PointCloud> vec_cloud;
    std::vector<sensor_msgs::PointCloud> vec_cloud_ui;
    sensor_msgs::PointCloud scan_cloud;
    vec_cloud.resize(current_scans.size());
    vec_cloud_ui.resize(current_scans.size());

    if (b_offset_init_ == false && !current_scans.empty()) {
        b_offset_init_ = true;
        UpdateOffset(current_scans);
    }
    else if (!current_scans.empty()) {
        for (int i = 0; i < current_scans.size(); i++) {
            {
                std::lock_guard<std::mutex> lock(mtx_lock_correct_);
                vec_cloud[i].header.stamp = nav_odom_.header.stamp;
                vec_cloud_ui[i].header.stamp = nav_odom_.header.stamp;
            }

            vec_cloud[i].header.frame_id = s_target_frame_id;
            vec_cloud_ui[i].header.frame_id = s_target_frame_id;
            try {
                if (vec_check_transform_[i] == false) {
                    if (!vec_tf_listener_[i]->waitForTransform(
                            s_target_frame_id, current_scans[i]->header.frame_id, current_scans[i]->header.stamp, ros::Duration(1))) {
                        return false;
                    }
                    vec_target_frame_id_[i] = current_scans[i]->header.frame_id;
                    vec_check_transform_[i] = true;
                }
                int channel = laser_geometry::channel_option::Distance + laser_geometry::channel_option::Index +
                    laser_geometry::channel_option::Intensity;
                sensor_msgs::LaserScan o_laser = *current_scans[i];

                if (vec_f_increment_[i] > 0) {
                    o_laser.angle_increment = vec_f_increment_[i];
                }
                if (vec_f_angle_min_[i] != 0) {
                    o_laser.angle_min = vec_f_angle_min_[i];
                }
                if (vec_range_offset_m_[i] != 0) {
                    for (int idx = 0; idx < o_laser.ranges.size(); idx++) {
                        o_laser.ranges[idx] += vec_range_offset_m_[i];
                    }
                }

                projector_.transformLaserScanToPointCloud(s_target_frame_id, o_laser, vec_cloud[i], *vec_tf_listener_[i], channel);
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
        }

        // 가상 라이다 정보 추가 by nate
        if (o_obs_info_.f_size > 0) {
            for (int id = 0; id < vec_cloud.size(); id++) {
                Pos o_center;
                o_center.SetXm(o_obs_info_.o_pos.GetXm() - o_pf_pos_.GetXm());
                o_center.SetYm(o_obs_info_.o_pos.GetYm() - o_pf_pos_.GetYm());
                o_center = TransformRotationDeg_(o_center, -o_pf_pos_.GetDeg());

                for (int deg = 0; deg < 360; deg++) {
                    geometry_msgs::Point32 point;
                    point.x = o_center.GetXm() + cos(deg * DEGtoRAD) * o_obs_info_.f_size;
                    point.y = o_center.GetYm() + sin(deg * DEGtoRAD) * o_obs_info_.f_size;
                    // NLOG(info)<<"id "<<id<<" :: "<<o_center.GetXm()<<" / "<<o_center.GetYm()<<"  point deg "<<deg<<" x "<<point.y
                    // "<<point.y<<endl;
                    vec_cloud[id].points.emplace_back(point);
                    // vec_cloud_ui[id].points.emplace_back(point);
                }
            }
        }
        // 로봇 속도와 라이다 시간 차에 대한 포인트 위치 보상 by nate
        // d_odometry_time_ // 가장 최근의 오도메트리 시간
        std::vector<Interpolation> vec_inter_data = CalcInterPolation(current_scans);  // 라이다 포인트 오도메트리 기준 보상값 계산
        // ------------------------------------------------------

        std::vector<float> vec_degcheck(3600, 0.f);

        geometry_msgs::Point32 point;
        for (int scan_id = 0; scan_id < vec_cloud.size(); scan_id++) {
            vec_lidar_info_[scan_id].n_lidar_point_size = vec_cloud[scan_id].points.size();
            for (auto point : vec_cloud[scan_id].points) {
                point = CalcIPpoint(point, vec_inter_data[scan_id]);  // 포인트 보정 계산 적용

                float f_dist = hypot(point.x, point.y);

                if (CheckData(point.x, point.y, f_dist) == false || (n_lidar_temp_ == 1 && point.z < 0.0))
                    continue;
                if (scan_id == 0)  // front
                {
                    int n_deg_idx = int((atan2(point.y, point.x) + M_PI) / M_PI * 1800);
                    if (n_deg_idx >= 0 && n_deg_idx < 3600) {
                        vec_degcheck[n_deg_idx] = f_dist;
                        scan_cloud.points.emplace_back(point);  // 라이다 포인트 추가
                        AppendUIData(f_dist, vec_cloud_ui[scan_id], point);
                    }
                }
                else if (scan_id == 1)  // rear
                {
                    int n_deg_idx = int((atan2(point.y, point.x) + M_PI) / M_PI * 1800);
                    if (n_deg_idx >= 0 && n_deg_idx < 3600) {
                        if (vec_degcheck[n_deg_idx] <= FLT_EPSILON || fabs(f_dist - vec_degcheck[n_deg_idx]) > f_lidar_merge_range_diff_) {
                            vec_degcheck[n_deg_idx] = f_dist;
                            scan_cloud.points.emplace_back(point);
                            AppendUIData(f_dist, vec_cloud_ui[scan_id], point);
                        }
                    }
                }
                else if (scan_id == 2)  // left
                {
                    int n_deg_idx = int((atan2(point.y, point.x) + M_PI) / M_PI * 1800);
                    if (n_deg_idx >= 0 && n_deg_idx < 3600) {
                        if (vec_degcheck[n_deg_idx] <= FLT_EPSILON || fabs(f_dist - vec_degcheck[n_deg_idx]) > f_lidar_merge_range_diff_) {
                            vec_degcheck[n_deg_idx] = f_dist;
                            scan_cloud.points.emplace_back(point);
                            AppendUIData(f_dist, vec_cloud_ui[scan_id], point);
                        }
                    }
                }
                else if (scan_id == 3)  // right
                {
                    int n_deg_idx = int((atan2(point.y, point.x) + M_PI) / M_PI * 1800);
                    if (n_deg_idx >= 0 && n_deg_idx < 3600) {
                        if (vec_degcheck[n_deg_idx] <= FLT_EPSILON || fabs(f_dist - vec_degcheck[n_deg_idx]) > f_lidar_merge_range_diff_) {
                            vec_degcheck[n_deg_idx] = f_dist;
                            scan_cloud.points.emplace_back(point);
                            AppendUIData(f_dist, vec_cloud_ui[scan_id], point);
                        }
                    }
                }
            }
        }

        // LOG_INFO("computation : %.3f",(ros::Time::now() - start).toSec());
        static long long int scan_count;
        {
            std::lock_guard<std::mutex> lock(mtx_lock_correct_);
            scan_cloud.header = nav_odom_.header;
        }
        scan_cloud.header.frame_id = s_target_frame_id;
        scan_cloud.header.seq = scan_count++;
        sensor_msgs::ChannelFloat32 channel;
        channel.values.emplace_back(0);
        scan_cloud.channels.emplace_back(channel);
        pub_scan_.publish(scan_cloud);
        MergeTransform(scan_cloud);

        pub_scan1_.publish(vec_cloud[0]);
        pub_scan1_global_.publish(ToGlobal(vec_cloud_ui[0]));
        if (vec_cloud.size() >= 2) {
            pub_scan2_.publish(vec_cloud[1]);
            pub_scan2_global_.publish(ToGlobal(vec_cloud_ui[1]));
        }
        if (vec_cloud.size() >= 3) {
            pub_scan3_.publish(vec_cloud[2]);
            pub_scan3_global_.publish(ToGlobal(vec_cloud_ui[2]));
        }
        if (vec_cloud.size() >= 4) {
            pub_scan4_.publish(vec_cloud[3]);
            pub_scan4_global_.publish(ToGlobal(vec_cloud_ui[3]));
        }
    }
    else {
        NLOG(error) << "scan list empty";
    }
    return true;
    // NLOG(info)<<"front gap "<<(scan_cloud.header.stamp - o_laser1.header.stamp)*1000<<" rear gap "<<(scan_cloud.header-
    // o_laser2.header.stamp)*1000<<endl;
}
Pos LidarMergerFilter::TransformRotationRad_(const Pos& o_pos, float f_rad)
{
    float f_rot_x = o_pos.GetXm() * cos(f_rad) - o_pos.GetYm() * sin(f_rad);
    float f_rot_y = o_pos.GetXm() * sin(f_rad) + o_pos.GetYm() * cos(f_rad);
    return Pos(f_rot_x, f_rot_y);
}

Pos LidarMergerFilter::TransformRotationDeg_(const Pos& o_pos, float f_deg)
{
    return TransformRotationRad_(o_pos, f_deg * DEGtoRAD);
}
void LidarMergerFilter::AppendUIData(float f_dist, sensor_msgs::PointCloud& vec_cloud, geometry_msgs::Point32 point)
{
    static int n_ui_pub_std = 0;
    if (!b_full_lidar_) {
        if (f_dist < n_full_lidar_m_) {  // n_full_lidar_m_ m 이내면 전체 보여주기
            vec_cloud.points.emplace_back(point);
        }
        else  // 15m 이내면 n_skip_lidar_std_개 건너뛰기
        {
            n_ui_pub_std++;
            if (n_ui_pub_std > n_skip_lidar_std_) {
                vec_cloud.points.emplace_back(point);
                n_ui_pub_std = 0;
            }
        }
    }
    else {
        vec_cloud.points.emplace_back(point);
    }
}  // namespace NaviFra

sensor_msgs::PointCloud LidarMergerFilter::ToGlobal(sensor_msgs::PointCloud& vec_cloud)
{
    // pose_update_mtx_.lock();
    geometry_msgs::PoseWithCovarianceStamped robot_pose;
    sensor_msgs::PointCloud msg_result = vec_cloud;

    Pos current_pose;

    try {
        auto robot_pose_in_tf = tf_->lookupTransform("map", "base_link", ros::Time(0));
        current_pose.SetXm(robot_pose_in_tf.transform.translation.x);
        current_pose.SetYm(robot_pose_in_tf.transform.translation.y);
        current_pose.SetRad(tf2::getYaw(robot_pose_in_tf.transform.rotation));
    }
    catch (tf2::TransformException) {
        ROS_DEBUG("fail to transform map to base_link");

        std::lock_guard<std::mutex> lock(mtx_lock_pos_);
        current_pose = o_pf_pos_;
    }

    // NLOG(info) << "robot pose in map : " << current_pose.GetXm() << ", " << current_pose.GetYm() << ", " << current_pose.GetDeg();
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

void LidarMergerFilter::MergeTransform(const sensor_msgs::PointCloud& scan_cloud)
{
    sensor_msgs::LaserScan scan_msg;
    scan_msg.header.frame_id = "laser_link";
    scan_msg.header.stamp = ros::Time::now();
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = 0.00174533;
    scan_msg.scan_time = 1. / 25.;
    scan_msg.range_min = f_dist_min_m_;
    scan_msg.range_max = f_dist_max_m_;
    uint32_t ranges_size = std::ceil((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
    scan_msg.time_increment = (1. / 25.) / ranges_size;
    scan_msg.ranges.assign(ranges_size, 0);

    for (int i = 0; i < scan_cloud.points.size(); i++) {
        double angle = atan2(scan_cloud.points[i].y, scan_cloud.points[i].x);
        int index = (angle - scan_msg.angle_min) / scan_msg.angle_increment;

        float f_merge_dist = hypot(scan_cloud.points[i].y, scan_cloud.points[i].x);
        if (f_merge_dist >= scan_msg.range_min && f_merge_dist <= scan_msg.range_max) {
            scan_msg.ranges[index] = f_merge_dist;
        }
    }
    pub_merge_scan_.publish(scan_msg);
}

bool LidarMergerFilter::CheckData(const float& x, const float& y, const float& f_dist)
{
    if(b_load_diff_range_use_ && b_loaded_) // Fork position 0 : Down
    {
        if (f_dist > f_dist_min_m_ && f_dist < f_dist_max_m_ && (x > f_x_load_front_m_ || x < f_x_load_back_m_ || y > f_y_load_left_m_ || y < f_y_load_right_m_)) {
            return true;
        }
    }
    else{
        if (f_dist > f_dist_min_m_ && f_dist < f_dist_max_m_ && (x > f_x_front_m_ || x < f_x_back_m_ || y > f_y_left_m_ || y < f_y_right_m_)) {
            return true;
        }
    }
    return false;
}

void LidarMergerFilter::messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans;
    current_scans.emplace_back(scan);

    LidarMerge(current_scans);
}

void LidarMergerFilter::messageFilterCallback(const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2)
{
    std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans;

    current_scans.emplace_back(scan1);
    current_scans.emplace_back(scan2);
    MeasureTimeGap(current_scans);
    // merge_gap << d_scan_gap << endl;
    LidarMerge(current_scans);
}

void LidarMergerFilter::messageFilterCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2,
    const sensor_msgs::LaserScan::ConstPtr& scan3)
{
    std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans;

    current_scans.emplace_back(scan1);
    current_scans.emplace_back(scan2);
    current_scans.emplace_back(scan3);

    MeasureTimeGap(current_scans);

    // NLOG(info)<<"scan gap : "<<d_scan_gap;
    // merge_gap << d_scan_gap << endl;
    LidarMerge(current_scans);
}

void LidarMergerFilter::messageFilterCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan1, const sensor_msgs::LaserScan::ConstPtr& scan2,
    const sensor_msgs::LaserScan::ConstPtr& scan3, const sensor_msgs::LaserScan::ConstPtr& scan4)
{
    std::vector<sensor_msgs::LaserScan::ConstPtr> current_scans;

    current_scans.emplace_back(scan1);
    current_scans.emplace_back(scan2);
    current_scans.emplace_back(scan3);
    current_scans.emplace_back(scan4);

    MeasureTimeGap(current_scans);
    // NLOG(info)<<"scan gap : "<<d_scan_gap;
    // merge_gap << d_scan_gap << endl;
    LidarMerge(current_scans);
}

void LidarMergerFilter::MeasureTimeGap(std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scan)
{
    for (int i = 0; i < current_scan.size() - 1; i++) {
        for (int j = 1; j < current_scan.size(); j++) {
            if (i == j)
                continue;
            double d_scan_gap = current_scan[i]->header.stamp.toSec() - current_scan[j]->header.stamp.toSec();
            if (d_scan_gap > 0.04) {
                LOG_WARNING("%d and %d time gap %.3f\n", i, j, d_scan_gap);
            }
        }
    }
}

vector<Interpolation> LidarMergerFilter::CalcInterPolation(const std::vector<sensor_msgs::LaserScan::ConstPtr>& current_scan)
{
    std::lock_guard<std::mutex> lock(mtx_lock_correct_);

    vector<Interpolation> vec_data;
    Interpolation o_data;

    for (int i = 0; i < current_scan.size(); i++) {
        double d_time_gap = d_odometry_time_ - current_scan[i]->header.stamp.toSec();

        o_data.f_correct_x = -o_correct_info_.GetXm() * fabs(d_time_gap);
        o_data.f_correct_y = -o_correct_info_.GetYm() * fabs(d_time_gap);
        o_data.f_theta_deg_sec = -o_correct_info_.GetRad() * fabs(d_time_gap);
        o_data.f_sin = sin(o_data.f_theta_deg_sec);
        o_data.f_cos = cos(o_data.f_theta_deg_sec);
        vec_data.emplace_back(o_data);
        // NLOG(info)<<i<<" d_time_gap "<<d_time_gap<<" x "<<o_data.f_correct_x<<" y "<<o_data.f_correct_y<<" deg "<<o_data.f_theta_deg_sec;
    }

    return vec_data;
}

geometry_msgs::Point32 LidarMergerFilter::CalcIPpoint(const geometry_msgs::Point32& point, const Interpolation& o_IP)
{
    geometry_msgs::Point32 result;
    float f_x = point.x;
    float f_y = point.y;
    result.x = f_x * o_IP.f_cos - f_y * o_IP.f_sin + o_IP.f_correct_x;
    result.y = f_x * o_IP.f_sin + f_y * o_IP.f_cos + o_IP.f_correct_y;
    result.z = point.z;
    return result;
}

float LidarMergerFilter::GetHz(std::vector<std::chrono::steady_clock::time_point>& vec_time)
{
    float f_sec_sum = 0;
    int n_sec_cnt = 0;
    for (int i = 1; i < vec_time.size(); i++) {
        std::chrono::duration<double> sec = vec_time[i] - vec_time[i - 1];
        f_sec_sum += sec.count();
        n_sec_cnt++;
    }
    return float(1 / (f_sec_sum / n_sec_cnt));
}

NaviFra::Pos LidarMergerFilter::PosConvert(const geometry_msgs::PoseStamped& input)
{
    NaviFra::Pos o_pos;
    o_pos.SetXm(input.pose.position.x);
    o_pos.SetYm(input.pose.position.y);
    o_pos.SetQuaternion(input.pose.orientation.w, input.pose.orientation.x, input.pose.orientation.y, input.pose.orientation.z);
    return o_pos;
}

}  // namespace NaviFra

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_merger");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

#ifdef NAVIFRA_LICENSE_ON

    License o_license_checker;
    if (o_license_checker.CheckLicence() == 0) {
        LOG_ERROR("************************************************");
        LOG_ERROR("MERGER Your License Not Resiter. Terminate the process.");
        LOG_ERROR("************************************************");
        return 0;
    }
    else {
        LOG_INFO("MERGER License is Resitered.");
    }

#else  // license 옵션 안건 경우
    LOG_WARNING("************************************************");
    LOG_WARNING("MERGER License Not Checked!  Please Check your License.");
    LOG_WARNING("************************************************");
#endif

    NaviFra::LidarMergerFilter lidardriver(nh, nhp);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    // ros::spin();
    ros::waitForShutdown();
    return 0;
}
