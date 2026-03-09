

#include "nc_calibrator.hpp"

#include "core/util/logger.hpp"
#include "std_msgs/Float64MultiArray.h"

using namespace cv;
using namespace std;
Calibrator::Calibrator()
{
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    nh_private.param<string>("front_laser", str_front_laser_link_, std::string("front_laser"));
    nh_private.param<string>("rear_laser", str_rear_laser_link_, std::string("rear_laser"));

    nh_private.param<float>("rotation_num_", n_rotation_num_, 1);
    nh_private.param<float>("go_dist_", f_go_dist_, 2);

    nh_private.param("d_base_length_", d_base_length_, 1.2);

    sub_ = n_.subscribe("odom", 2, &Calibrator::odomCallback, this, ros::TransportHints().tcpNoDelay(true));
    sub2_ = n_.subscribe("cali_cmd", 10, &Calibrator::cmdCallback, this);

    sub_scan_cloud_ = n_.subscribe("scan_cloud", 10, &Calibrator::scanCloudCallback, this);
    sub_cloud_front_ = n_.subscribe("front_cloud", 10, &Calibrator::scanFrontCallback, this);
    sub_cloud_rear_ = n_.subscribe("rear_cloud", 10, &Calibrator::scanRearCallback, this);

    pub_error_x = n_.advertise<std_msgs::Float64>("cali/error_x", 10);
    pub_error_xa = n_.advertise<std_msgs::Float64>("cali/error_xa", 10);
    pub_error_a = n_.advertise<std_msgs::Float64>("cali/error_a", 10);
    pub_dist = n_.advertise<std_msgs::String>("cali/info", 10);
    pub_result = n_.advertise<std_msgs::String>("cali/result", 10);

    pub_CaliErrorProgress = n_.advertise<std_msgs::Float64MultiArray>("cali/error_progress", 10);  // for "Brain UI"
    pub_CaliResult = n_.advertise<std_msgs::Float64MultiArray>("cali/calresult", 10);  // for "Brain UI"
}

float Calibrator::CalCostIcp(vector<sensor_msgs::LaserScan>& vec_data, float f_angle_inc)
{
    NaviFra::Pos o_result(0, 0, 0);
    int n_cnt = 0;
    float f_score_sum = 0;

    // first step
    for (int i = 180; i < vec_data.size(); i += 7) {
        sensor_msgs::LaserScan scan_pre = vec_data[i - 180];
        sensor_msgs::LaserScan scan_now = vec_data[i];

        NaviFra::PointToPlaneICP o_icp;
        o_icp.SetParameter(10, 0.0001, 0.2f, 0.4f, 0.05f);

        std::vector<NaviFra::SimplePos> vec_start;
        std::vector<NaviFra::SimplePos> vec_end;

        sensor_msgs::PointCloud start_cloud1, start_cloud2;
        sensor_msgs::PointCloud end_cloud1, end_cloud2;

        scan_pre.angle_increment = f_angle_inc;
        scan_now.angle_increment = f_angle_inc;

        projector_.transformLaserScanToPointCloud(
            scan_pre.header.frame_id, scan_pre, start_cloud1, tfListener1_, laser_geometry::channel_option::Distance);
        tfListener1_.transformPointCloud("base_link", start_cloud1, start_cloud2);

        projector_.transformLaserScanToPointCloud(
            scan_now.header.frame_id, scan_now, end_cloud1, tfListener1_, laser_geometry::channel_option::Distance);
        tfListener1_.transformPointCloud("base_link", end_cloud1, end_cloud2);

        for (int j = 0; j < start_cloud2.points.size(); j++) {
            NaviFra::SimplePos o_pos;
            o_pos.SetXm(start_cloud2.points[j].x);
            o_pos.SetYm(start_cloud2.points[j].y);
            vec_start.emplace_back(o_pos);
        }

        for (int j = 0; j < end_cloud2.points.size(); j++) {
            NaviFra::SimplePos o_pos;
            o_pos.SetXm(end_cloud2.points[j].x);
            o_pos.SetYm(end_cloud2.points[j].y);
            vec_end.emplace_back(o_pos);
        }
        NaviFra::SimplePos o_start_pos(0, 0, 180);
        NaviFra::SimplePos o_icp_result = o_icp.CalcICPpos(vec_start, vec_end, o_start_pos, 2);
        f_score_sum += o_icp.GetRMSE();
        n_cnt++;
        NLOG(info) << "o_icp_rear_result " << o_icp_result.GetXm() << " " << o_icp_result.GetYm() << " " << o_icp_result.GetDeg() << " / "
                   << o_icp.GetRMSE();
    }
    NLOG(info) << "result " << o_result.GetXm() << " " << o_result.GetYm() << " " << o_result.GetRad();
    return f_score_sum / n_cnt;
}

void Calibrator::scanCloudCallback(const sensor_msgs::PointCloudConstPtr& point_cloud)
{
    // if (str_.size() == 0)
    //     return;
    {
        std::lock_guard<std::mutex> lock(mtx_lock_lidar_);
        cloud_ = *point_cloud;
    }
}

void Calibrator::scanFrontCallback(const sensor_msgs::PointCloudConstPtr& point_cloud)
{
    // if (str_.size() == 0)
    //     return;
    {
        std::lock_guard<std::mutex> lock(mtx_lock_lidar_);
        cloud_front_ = *point_cloud;
    }
}

void Calibrator::scanRearCallback(const sensor_msgs::PointCloudConstPtr& point_cloud)
{
    // if (str_.size() == 0)
    //     return;
    {
        std::lock_guard<std::mutex> lock(mtx_lock_lidar_);
        cloud_rear_ = *point_cloud;
    }
}

void Calibrator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    start = std::chrono::steady_clock::now();

    if (str_.size() == 0)
        return;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::PoseStamped robot_pose;

    robot_pose.header.frame_id = "base_link";
    robot_pose.header.stamp = ros::Time(0);
    robot_pose.pose.position.x = 0.0;
    robot_pose.pose.position.y = 0.0;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = 0.0;
    robot_pose.pose.orientation.y = 0.0;
    robot_pose.pose.orientation.z = 0.0;
    robot_pose.pose.orientation.w = 1.0;

    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("base_link", "odom", ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(robot_pose, robot_pose, transformStamped);
    }
    catch (tf::TransformException const& ex) {
        ROS_WARN_STREAM_THROTTLE(1.0, "4 Transform error: " << ex.what());
    }
    robot_pos_ = PosConvert(robot_pose);
    o_odom_pos_ = PosConvert(msg);
    {
        const std::lock_guard<std::mutex> lock(mtx_lock_);
        d_yaw_pre_ = d_yaw_;
        d_yaw_ = o_odom_pos_.GetRad();
    }

    if (str_ == "v_f" || str_ == "v_b")
        GoCmd();
    if (str_ == "w_p" || str_ == "w_m")
        RotateCmd();
    if (str_ == "lidar")
        CalLidar();
    if (str_ == "yaw")
        CalYaw();
}

void Calibrator::Init()
{
    NLOG(info) << "INIT";
    b_cmd_ = false;
    d_yaw_sum_ = 0;
    odom_start_ = odom_;
    d_yaw_start_ = d_yaw_;
    robot_pos_start_ = robot_pos_;
    f_matching_rmse_ = 0;
    {
        std::lock_guard<std::mutex> lock(mtx_lock_lidar_);
        NLOG(info) << "cloud_.points.size() " << cloud_.points.size();
        vec_lidar_start_.clear();
        vec_lidar_start_.resize(cloud_.points.size());
        for (int i = 0; i < vec_lidar_start_.size(); i++) {
            vec_lidar_start_[i].SetXm(cloud_.points[i].x);
            vec_lidar_start_[i].SetYm(cloud_.points[i].y);
        }

        vec_lidar_front_start_.clear();
        vec_lidar_front_start_.resize(cloud_front_.points.size());
        for (int i = 0; i < vec_lidar_front_start_.size(); i++) {
            vec_lidar_front_start_[i].SetXm(cloud_front_.points[i].x);
            vec_lidar_front_start_[i].SetYm(cloud_front_.points[i].y);
        }

        vec_lidar_rear_start_.clear();
        vec_lidar_rear_start_.resize(cloud_rear_.points.size());
        for (int i = 0; i < vec_lidar_rear_start_.size(); i++) {
            vec_lidar_rear_start_[i].SetXm(cloud_rear_.points[i].x);
            vec_lidar_rear_start_[i].SetYm(cloud_rear_.points[i].y);
        }
    }
    o_odom_pos_start_ = o_odom_pos_;
    NLOG(info) << "Init done ";
}

void Calibrator::cmdCallback(const std_msgs::String::ConstPtr& msg)
{
    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - start;
    if (sec.count() > 1) {
        NLOG(info) << "odom is not callbacked!";
        return;
    }

    str_ = msg->data;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    Init();

    if (str_ == "clear") {
        str_ = "";
    }
}

void Calibrator::CalLidar()
{
    int n_target_size = 400;
    int n_iter = 50;
    float f_max = 0.00174533;
    float f_min = 0.00174000;
    float f_min_score = 100;
    float f_min_result = 0;
    int n_stop_cnt = 0;

    if (vec_front_scan_.size() > n_target_size && vec_rear_scan_.size() > n_target_size) {
        // stop
        geometry_msgs::Twist cmd;
        cmd.angular.z = 0.00;
        pub_.publish(cmd);

        std_msgs::String msg;
        string s_front_msg = "";
        string s_rear_msg = "";
        msg.data = s_front_msg + "/" + s_rear_msg;
        pub_dist.publish(msg);

        // front
        f_max = 0.001746;
        f_min = 0.001738;
        f_min_score = 100;
        n_stop_cnt = 0;

        for (int i = 0; i < n_iter; i++) {
            std::lock_guard<std::mutex> lock(mtx_lock_front_);
            float f_angle_increment = f_max - (f_max - f_min) * i / n_iter;
            float rmse = CalCostIcp(vec_front_scan_, f_angle_increment);
            NLOG(info) << "nate front i " << i << " rmse " << rmse << " / inc " << f_angle_increment;
            std::stringstream stream;
            stream << std::fixed << std::setprecision(8) << f_angle_increment;
            s_front_msg = "front i " + to_string(i) + " rmse " + to_string(rmse) + " / inc " + stream.str();
            if (rmse < f_min_score) {
                f_min_score = rmse;
                f_min_result = f_angle_increment;
                n_stop_cnt = 0;
            }
            else {
                n_stop_cnt++;
            }
            if (n_stop_cnt > 3) {
                break;
            }
            msg.data = s_front_msg + "/" + s_rear_msg;
            pub_dist.publish(msg);
        }
        NLOG(info) << "nate front f_min_result " << f_min_result;
        vec_front_scan_.clear();

        std::stringstream stream1;
        stream1 << std::fixed << std::setprecision(8) << f_min_result;
        s_front_msg = stream1.str();
        msg.data = s_front_msg + "/" + s_rear_msg;
        pub_dist.publish(msg);

        // rear
        f_min_score = 100;
        n_stop_cnt = 0;

        for (int i = 0; i < n_iter; i++) {
            std::lock_guard<std::mutex> lock(mtx_lock_rear_);
            float f_angle_increment = f_max - (f_max - f_min) * i / n_iter;
            float rmse = CalCostIcp(vec_rear_scan_, f_angle_increment);
            NLOG(info) << "nate rear i " << i << " rmse " << rmse << " / inc " << f_angle_increment;

            std::stringstream stream;
            stream << std::fixed << std::setprecision(8) << f_angle_increment;
            s_rear_msg = "rear i " + to_string(i) + " rmse " + to_string(rmse) + " / inc " + stream.str();
            if (rmse < f_min_score) {
                f_min_score = rmse;
                f_min_result = f_angle_increment;
                n_stop_cnt = 0;
            }
            else {
                n_stop_cnt++;
            }
            if (n_stop_cnt > 3) {
                break;
            }
            msg.data = s_front_msg + "/" + s_rear_msg;
            pub_dist.publish(msg);
        }
        NLOG(info) << "nate rear f_min_result " << f_min_result;
        vec_rear_scan_.clear();

        std::stringstream stream2;
        stream2 << std::fixed << std::setprecision(8) << f_min_result;
        s_rear_msg = stream2.str();
        msg.data = s_front_msg + "/" + s_rear_msg;
        pub_dist.publish(msg);

        str_ = "";
    }
    else {
        geometry_msgs::Twist cmd;
        // cmd.angular.z    = 0.2;
        cmd.angular.z = 0.05;
        pub_.publish(cmd);

        std_msgs::String msg;
        msg.data = to_string(n_target_size) + "/" + to_string(vec_front_scan_.size()) + "," + to_string(vec_rear_scan_.size());
        pub_dist.publish(msg);
    }
}

void Calibrator::CalYaw()
{
    int n_target_size = 200;
    int n_iter = 50;
    float f_max = 2;
    float f_min = -2;
    float f_min_score = 100;
    float f_min_result = 0;
    int n_stop_cnt = 0;
    float f_start_deg = 0;

    if (vec_front_scan_.size() > n_target_size && vec_rear_scan_.size() > n_target_size) {
        ifstream in("lidar_sensor_info.txt");
        string stringBuffer;
        vector<string> vec_f_offset;

        while (getline(in, stringBuffer, ',')) {
            stringBuffer.erase(stringBuffer.find_last_not_of(" \n\r\t") + 1);
            vec_f_offset.emplace_back(stringBuffer);
            NLOG(info) << "lidar_sensor_info " << vec_f_offset.size();
        }

        f_front_increment_ = 0.00174532925;
        f_rear_increment_ = 0.00174532925;

        if (vec_f_offset.size() < 4)
            return;
        vector<float> vec_offset;
        for (int i = 0; i < vec_f_offset.size(); i++)
            vec_offset.emplace_back(std::stof(vec_f_offset[i]));

        f_front_increment_ = vec_offset[0];
        f_rear_increment_ = vec_offset[1];

        // stop
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.00;
        pub_.publish(cmd);

        std_msgs::String msg;
        string s_front_msg = "";
        string s_rear_msg = "";
        msg.data = s_front_msg + "/" + s_rear_msg;
        pub_dist.publish(msg);

        // front
        f_min_score = 100;
        n_stop_cnt = 0;
        f_start_deg = -45;
        for (int i = 0; i < n_iter; i++) {
            std::lock_guard<std::mutex> lock(mtx_lock_front_);
            float f_angle_rad = (f_start_deg + f_max - (f_max - f_min) * i / n_iter) * DEGtoRAD;
            float rmse = CalCostIcp2(vec_front_scan_, f_angle_rad, true);
            NLOG(info) << "nate front i " << i << " rmse " << rmse << " / inc " << f_angle_rad;
            std::stringstream stream;
            stream << std::fixed << std::setprecision(8) << f_angle_rad;
            s_front_msg = "front i " + to_string(i) + " rmse " + to_string(rmse) + " / inc " + stream.str();
            if (rmse < f_min_score) {
                f_min_score = rmse;
                f_min_result = f_angle_rad;
                n_stop_cnt = 0;
            }
            else {
                n_stop_cnt++;
            }
            if (n_stop_cnt > 2) {
                break;
            }
            msg.data = s_front_msg + "/" + s_rear_msg;
            pub_dist.publish(msg);
        }
        NLOG(info) << "nate front f_min_result " << f_min_result;
        vec_front_scan_.clear();

        std::stringstream stream1;
        stream1 << std::fixed << std::setprecision(8) << f_min_result;
        s_front_msg = stream1.str();
        msg.data = s_front_msg + "/" + s_rear_msg;
        pub_dist.publish(msg);

        // rear
        f_min_score = 100;
        n_stop_cnt = 0;
        f_start_deg = 135;
        for (int i = 0; i < n_iter; i++) {
            std::lock_guard<std::mutex> lock(mtx_lock_rear_);
            float f_angle_rad = (f_start_deg + f_max - (f_max - f_min) * i / n_iter) * DEGtoRAD;
            float rmse = CalCostIcp2(vec_rear_scan_, f_angle_rad, false);
            NLOG(info) << "nate rear i " << i << " rmse " << rmse << " / inc " << f_angle_rad;

            std::stringstream stream;
            stream << std::fixed << std::setprecision(8) << f_angle_rad;
            s_rear_msg = "rear i " + to_string(i) + " rmse " + to_string(rmse) + " / inc " + stream.str();
            if (rmse < f_min_score) {
                f_min_score = rmse;
                f_min_result = f_angle_rad;
                n_stop_cnt = 0;
            }
            else {
                n_stop_cnt++;
            }
            if (n_stop_cnt > 2) {
                break;
            }
            msg.data = s_front_msg + "/" + s_rear_msg;
            pub_dist.publish(msg);
        }
        NLOG(info) << "nate rear f_min_result " << f_min_result;
        vec_rear_scan_.clear();

        std::stringstream stream2;
        stream2 << std::fixed << std::setprecision(8) << f_min_result;
        s_rear_msg = stream2.str();
        msg.data = s_front_msg + "/" + s_rear_msg;
        pub_dist.publish(msg);

        str_ = "";
    }
    else {
        geometry_msgs::Twist cmd;
        // cmd.linear.x    = 0.2;
        cmd.linear.x = 0.05;
        pub_.publish(cmd);

        std_msgs::String msg;
        msg.data = to_string(n_target_size) + "/" + to_string(vec_front_scan_.size()) + "," + to_string(vec_rear_scan_.size());
        pub_dist.publish(msg);
    }
}

float Calibrator::CalCostIcp2(vector<sensor_msgs::LaserScan>& vec_data, float f_angle_rad, bool b_front)
{
    int n_data_size = vec_data.size();

    int n_idx_start = 20;

    NaviFra::Pos o_result(0, 0, 0);
    int n_cnt = 0;

    // first step
    for (int i = n_idx_start; i < n_data_size; i += 5) {
        sensor_msgs::LaserScan scan_pre = vec_data[i - n_idx_start];
        sensor_msgs::LaserScan scan_now = vec_data[i];
        if (b_front) {
            scan_pre.angle_increment = f_front_increment_;
            scan_now.angle_increment = f_front_increment_;
        }
        else {
            scan_pre.angle_increment = f_rear_increment_;
            scan_now.angle_increment = f_rear_increment_;
        }
        NaviFra::SimplePos o_start_pos(0, 0, 0);
        NaviFra::PointToPlaneICP o_icp;
        o_icp.SetParameter(10, 0.0001, 0.1f, 0.3f, 0.05f);
        o_start_pos.SetXm(0.01 * n_idx_start);
        std::vector<NaviFra::SimplePos> front_start = LStoPS(scan_pre, f_angle_rad);
        std::vector<NaviFra::SimplePos> front_end = LStoPS(scan_now, f_angle_rad);

        NaviFra::SimplePos o_icp_result = o_icp.CalcICPpos(front_start, front_end, o_start_pos, 2);

        o_result.SetXm(o_result.GetXm() + fabs(o_icp_result.GetXm()));
        o_result.SetYm(o_result.GetYm() + fabs(o_icp_result.GetYm()));
        o_result.SetRad(o_result.GetRad() + fabs(o_icp_result.GetRad()));
        n_cnt++;
    }
    if (n_cnt != 0) {
        o_result.SetXm(o_result.GetXm() / (float)n_cnt);
        o_result.SetYm(o_result.GetYm() / (float)n_cnt);
        o_result.SetRad(o_result.GetRad() / (float)n_cnt);
    }
    NLOG(info) << "result " << o_result.GetXm() << " " << o_result.GetYm() << " " << o_result.GetRad();
    return fabs(o_result.GetYm());
}

std::vector<NaviFra::SimplePos> Calibrator::LStoPS(const sensor_msgs::LaserScan& laser_scan, float f_angle)
{
    float cx = cos(0 * DEGtoRAD);
    float sx = sin(0 * DEGtoRAD);
    float cy = cos(0 * DEGtoRAD);
    float sy = sin(0 * DEGtoRAD);
    float cz = cos(f_angle);
    float sz = sin(f_angle);

    std::vector<NaviFra::SimplePos> vec_result;
    vec_result.resize(laser_scan.ranges.size());
    float f_angle_rad = laser_scan.angle_min;
    for (int i = 0; i < laser_scan.ranges.size(); i++) {
        float f_dist_m = laser_scan.ranges[i];
        float px = f_dist_m * cos(f_angle_rad);
        float py = f_dist_m * sin(f_angle_rad);
        float pz = 0.0f;

        vec_result[i].SetXm(px * (cy * cz) + py * (-cx * sz + sx * sy * cz) + pz * (sx * sz + cx * sy * cz));
        vec_result[i].SetYm(px * (cy * sz) + py * (cx * cz + sx * sy * sz) + pz * (-sx * cz + cx * sy * sz));

        f_angle_rad += laser_scan.angle_increment;
    }
    return vec_result;
}

NaviFra::SimplePos Calibrator::CalIcpPoint(
    NaviFra::Pos& o_predict_robot_pos, vector<NaviFra::SimplePos>& start_lidar, vector<NaviFra::SimplePos>& new_lidar, int mode)
{
    NaviFra::PointToPlaneICP o_icp_;
    o_icp_.SetParameter(
        50, 0.001, 0.3f, 0.3f  //
        ,
        0.05f);  // 탐색 거리
    vector<NaviFra::SimplePos> vecTransformedNewData = new_lidar;

    vector<float> vec_deg(3);
    vec_deg[0] = 0;
    vec_deg[1] = -10;
    vec_deg[2] = +10;

    NLOG(info) << "vec_local_lidar_data " << vecTransformedNewData.size();
    float f_rmse_min = 9999;
    NaviFra::SimplePos o_best_icp = o_predict_robot_pos;
    for (int i = 0; i < 1; i++) {
        NaviFra::SimplePos o_init_pos = o_predict_robot_pos;
        o_init_pos.SetDeg(o_init_pos.GetDeg() + vec_deg[i]);

        NaviFra::SimplePos o_icp_tmp_pos = o_icp_.CalcICPpos(start_lidar, vecTransformedNewData, o_init_pos, 2);
        if (o_icp_.GetRMSE() < f_rmse_min) {
            NLOG(info) << i << " rmse o_icp_.GetRMSE() " << o_icp_.GetRMSE();
            f_rmse_min = o_icp_.GetRMSE();
            o_best_icp = o_icp_tmp_pos;
        }
    }

    NaviFra::SimplePos oCorrectedRobotPos = o_best_icp;
    f_matching_rmse_ = f_rmse_min;

    std_msgs::String msg;
    msg.data = to_string(f_matching_rmse_);
    pub_dist.publish(msg);
    return oCorrectedRobotPos;
}

void Calibrator::RotateCmd()
{
    if (abs(d_yaw_ - d_yaw_pre_) < 1) {
        d_yaw_sum_ += abs(d_yaw_ - d_yaw_pre_);
    }
    else {
        d_yaw_sum_ += abs(d_yaw_ + d_yaw_pre_);
    }
    geometry_msgs::Twist cmd;
    if (d_yaw_sum_ < 0.1) {
        cmd.angular.z = 0.1;
    }
    else if (d_yaw_sum_ < n_rotation_num_ * M_PI * 2 - 0.5) {
        cmd.angular.z = 0.15;
    }
    else {
        cmd.angular.z = 0.05;
    }

    if (str_ == "w_m") {
        cmd.angular.z = -cmd.angular.z;
    }
    NLOG(info) << d_yaw_sum_ << " " << abs(d_yaw_ - d_yaw_start_);
    std_msgs::String msg;
    msg.data = to_string(int(n_rotation_num_ * M_PI * 2 * RADtoDEG)) + "/" + to_string(int(d_yaw_sum_ * RADtoDEG));
    pub_dist.publish(msg);

    std_msgs::Float64MultiArray arrayMsg;
    if ((n_rotation_num_ * M_PI * 2 * RADtoDEG) > 0.0) {
        float fcaldist = (d_yaw_sum_ * RADtoDEG) / (n_rotation_num_ * M_PI * 2 * RADtoDEG);
        arrayMsg.data.push_back(0.0);  //// for "Brain UI" - x error
        arrayMsg.data.push_back(0.0);  //// for "Brain UI" - y error
        arrayMsg.data.push_back(0.0);  //// for "Brain UI" - deg error
        arrayMsg.data.push_back(round(fcaldist * 100) / 100);  //// for "Brain UI" - progress
        pub_CaliErrorProgress.publish(arrayMsg);
    }

    if (d_yaw_sum_ > n_rotation_num_ * M_PI * 2 - 0.01) {
        if (b_cmd_ == false) {
            wait_ = std::chrono::steady_clock::now();
            b_cmd_ = true;
        }
        cmd.angular.z = 0;
        cmd.linear.x = 0;
    }
    else {
        NLOG(info) << d_yaw_sum_ / M_PI / 2;
    }

    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - wait_;
    if (cmd.angular.z == 0 && sec.count() > 2) {
        NLOG(info) << "RotateCmd finish";

        NaviFra::Pos odom_temp;
        odom_temp.SetXm(0);
        odom_temp.SetYm(0);
        odom_temp.SetDeg(robot_pos_start_.GetDeg() - robot_pos_.GetDeg());
        {
            std::lock_guard<std::mutex> lock(mtx_lock_lidar_);

            vec_lidar_.clear();
            vec_lidar_.resize(cloud_.points.size());
            for (int i = 0; i < vec_lidar_.size(); i++) {
                vec_lidar_[i].SetXm(cloud_.points[i].x);
                vec_lidar_[i].SetYm(cloud_.points[i].y);
            }

            vec_lidar_front_.clear();
            vec_lidar_front_.resize(cloud_front_.points.size());
            for (int i = 0; i < vec_lidar_front_.size(); i++) {
                vec_lidar_front_[i].SetXm(cloud_front_.points[i].x);
                vec_lidar_front_[i].SetYm(cloud_front_.points[i].y);
            }

            vec_lidar_rear_.clear();
            vec_lidar_rear_.resize(cloud_rear_.points.size());
            for (int i = 0; i < vec_lidar_rear_.size(); i++) {
                vec_lidar_rear_[i].SetXm(cloud_rear_.points[i].x);
                vec_lidar_rear_[i].SetYm(cloud_rear_.points[i].y);
            }
        }
        NaviFra::SimplePos o_icp_pos = CalIcpPoint(odom_temp, vec_lidar_start_, vec_lidar_);

        NaviFra::SimplePos o_icp_pos_front = CalIcpPoint(odom_temp, vec_lidar_front_start_, vec_lidar_front_);
        NaviFra::SimplePos o_icp_pos_rear = CalIcpPoint(odom_temp, vec_lidar_rear_start_, vec_lidar_rear_);
        std_msgs::String msg_result;
        msg_result.data = "front x : " + to_string(o_icp_pos_front.GetXm() - odom_temp.GetXm()) +
            " y : " + to_string(o_icp_pos_front.GetYm() - odom_temp.GetYm()) + " deg : " + to_string(o_icp_pos_front.GetDeg()) +
            "\n rear x : " + to_string(o_icp_pos_rear.GetXm() - odom_temp.GetXm()) +
            " y : " + to_string(o_icp_pos_rear.GetYm() - odom_temp.GetYm()) + " deg : " + to_string(o_icp_pos_rear.GetDeg());

        pub_result.publish(msg_result);

        std_msgs::Float64MultiArray arrayProgressMsg, arrayResultMsg;
        arrayResultMsg.data.push_back(o_icp_pos_front.GetXm() - odom_temp.GetXm());  //// for "Brain UI" - front x error
        arrayResultMsg.data.push_back(o_icp_pos_front.GetYm() - odom_temp.GetYm());  //// for "Brain UI" - front y error
        arrayResultMsg.data.push_back(o_icp_pos_front.GetDeg());  //// for "Brain UI" - front deg error
        arrayResultMsg.data.push_back(o_icp_pos_rear.GetXm() - odom_temp.GetXm());  //// for "Brain UI" - rear x error
        arrayResultMsg.data.push_back(o_icp_pos_rear.GetYm() - odom_temp.GetYm());  //// for "Brain UI" - rear y error
        arrayResultMsg.data.push_back(o_icp_pos_rear.GetDeg());  //// for "Brain UI" - rear deg error
        arrayResultMsg.data.push_back(1.0);  //// for "Brain UI" - progress
        pub_CaliResult.publish(arrayResultMsg);

        NLOG(info) << "deg error : " << odom_temp.GetDeg() - o_icp_pos.GetDeg();
        NLOG(info) << "rot deg : " << d_yaw_sum_ / M_PI * 180;

        std_msgs::Float64 msg;
        msg.data = o_icp_pos.GetXm() - odom_temp.GetXm();
        pub_error_x.publish(msg);

        std_msgs::Float64 msg2;
        msg2.data = o_icp_pos.GetYm() - odom_temp.GetYm();
        pub_error_xa.publish(msg2);

        std_msgs::Float64 msg3;
        msg3.data = odom_temp.GetDeg() - o_icp_pos.GetDeg();
        pub_error_a.publish(msg3);

        arrayProgressMsg.data.push_back(msg.data);  //// for "Brain UI" - x error
        arrayProgressMsg.data.push_back(msg2.data);  //// for "Brain UI" - y error
        arrayProgressMsg.data.push_back(msg3.data);  //// for "Brain UI" - deg error
        arrayProgressMsg.data.push_back(100);  //// for "Brain UI" - progress
        pub_CaliErrorProgress.publish(arrayProgressMsg);

        NaviFra::Pos result;
        result.SetXm(d_yaw_sum_ / M_PI * 180);
        result.SetDeg(odom_temp.GetDeg() - o_icp_pos.GetDeg());

        str_ = "";
    }
    pub_.publish(cmd);
}

void Calibrator::GoCmd()
{
    NLOG(info) << "GoCmd";
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.1;
    if (std::hypot(o_odom_pos_.GetXm() - o_odom_pos_start_.GetXm(), o_odom_pos_.GetYm() - o_odom_pos_start_.GetYm()) > f_go_dist_ - 0.1) {
        cmd.linear.x = 0.05;
    }
    if (str_ == "v_b") {
        cmd.linear.x = -cmd.linear.x;
    }
    NLOG(info) << "f_go_dist_ " << f_go_dist_;
    if (std::hypot(o_odom_pos_.GetXm() - o_odom_pos_start_.GetXm(), o_odom_pos_.GetYm() - o_odom_pos_start_.GetYm()) >=
        f_go_dist_ - 0.001) {
        if (b_cmd_ == false) {
            wait_ = std::chrono::steady_clock::now();
            b_cmd_ = true;
        }
        cmd.linear.x = 0;
    }
    float f_dist = std::hypot(o_odom_pos_.GetXm() - o_odom_pos_start_.GetXm(), o_odom_pos_.GetYm() - o_odom_pos_start_.GetYm());
    NLOG(info) << f_dist;
    std_msgs::String msg;
    msg.data = to_string(f_go_dist_) + "/" + to_string(f_dist);
    pub_dist.publish(msg);

    if (f_dist > 0.0) {
        std_msgs::Float64MultiArray arrayMsg;
        float fcaldist = f_dist / f_go_dist_;
        arrayMsg.data.push_back(0.0);  //// for "Brain UI" - x error
        arrayMsg.data.push_back(0.0);  //// for "Brain UI" - y error
        arrayMsg.data.push_back(0.0);  //// for "Brain UI" - deg error
        arrayMsg.data.push_back(round(fcaldist * 100) / 100);  //// for "Brain UI" - progress
        pub_CaliErrorProgress.publish(arrayMsg);
    }

    std::chrono::duration<double> sec = std::chrono::steady_clock::now() - wait_;
    // NLOG(info)<<"sec : "<<sec.count();

    if (cmd.linear.x == 0 && sec.count() > 2) {
        NLOG(info) << "GoCmd finish";

        NaviFra::Pos odom_temp;
        odom_temp.SetXm(robot_pos_start_.GetXm() - robot_pos_.GetXm());
        odom_temp.SetYm(robot_pos_start_.GetYm() - robot_pos_.GetYm());
        odom_temp.SetDeg(0);
        {
            std::lock_guard<std::mutex> lock(mtx_lock_lidar_);

            vec_lidar_.clear();
            vec_lidar_.resize(cloud_.points.size());
            for (int i = 0; i < vec_lidar_.size(); i++) {
                vec_lidar_[i].SetXm(cloud_.points[i].x);
                vec_lidar_[i].SetYm(cloud_.points[i].y);
            }

            vec_lidar_front_.clear();
            vec_lidar_front_.resize(cloud_front_.points.size());
            for (int i = 0; i < vec_lidar_front_.size(); i++) {
                vec_lidar_front_[i].SetXm(cloud_front_.points[i].x);
                vec_lidar_front_[i].SetYm(cloud_front_.points[i].y);
            }

            vec_lidar_rear_.clear();
            vec_lidar_rear_.resize(cloud_rear_.points.size());
            for (int i = 0; i < vec_lidar_rear_.size(); i++) {
                vec_lidar_rear_[i].SetXm(cloud_rear_.points[i].x);
                vec_lidar_rear_[i].SetYm(cloud_rear_.points[i].y);
            }
        }
        NaviFra::SimplePos o_icp_pos = CalIcpPoint(odom_temp, vec_lidar_start_, vec_lidar_);

        NaviFra::SimplePos o_icp_pos_front = CalIcpPoint(odom_temp, vec_lidar_front_start_, vec_lidar_front_);
        NaviFra::SimplePos o_icp_pos_rear = CalIcpPoint(odom_temp, vec_lidar_rear_start_, vec_lidar_rear_);
        std_msgs::String msg_result;
        msg_result.data = "front x : " + to_string(o_icp_pos_front.GetXm() - odom_temp.GetXm()) +
            " y : " + to_string(o_icp_pos_front.GetYm() - odom_temp.GetYm()) + " deg : " + to_string(o_icp_pos_front.GetDeg()) +
            "\n rear x : " + to_string(o_icp_pos_rear.GetXm() - odom_temp.GetXm()) +
            " y : " + to_string(o_icp_pos_rear.GetYm() - odom_temp.GetYm()) + " deg : " + to_string(o_icp_pos_rear.GetDeg());

        pub_result.publish(msg_result);

        std_msgs::Float64MultiArray arrayProgressMsg, arrayResultMsg;
        arrayResultMsg.data.push_back(o_icp_pos_front.GetXm() - odom_temp.GetXm());  //// for "Brain UI" - front x error
        arrayResultMsg.data.push_back(o_icp_pos_front.GetYm() - odom_temp.GetYm());  //// for "Brain UI" - front y error
        arrayResultMsg.data.push_back(o_icp_pos_front.GetDeg());  //// for "Brain UI" - front deg error
        arrayResultMsg.data.push_back(o_icp_pos_rear.GetXm() - odom_temp.GetXm());  //// for "Brain UI" - rear x error
        arrayResultMsg.data.push_back(o_icp_pos_rear.GetYm() - odom_temp.GetYm());  //// for "Brain UI" - rear y error
        arrayResultMsg.data.push_back(o_icp_pos_rear.GetDeg());  //// for "Brain UI" - rear deg error
        arrayResultMsg.data.push_back(1.0);  //// for "Brain UI" - progress
        pub_CaliResult.publish(arrayResultMsg);

        NLOG(info) << "error : " << o_icp_pos.GetXm() - odom_temp.GetXm() << " " << o_icp_pos.GetYm() - odom_temp.GetYm() << " "
                   << o_icp_pos.GetDeg();

        std_msgs::Float64 msg;
        msg.data = o_icp_pos.GetXm() - odom_temp.GetXm();
        pub_error_x.publish(msg);

        std_msgs::Float64 msg2;
        msg2.data = o_icp_pos.GetYm() - odom_temp.GetYm();
        pub_error_xa.publish(msg2);

        std_msgs::Float64 msg3;
        msg3.data = o_icp_pos.GetDeg();
        pub_error_a.publish(msg3);

        std_msgs::Float64MultiArray arrayMsg;
        arrayProgressMsg.data.push_back(msg.data);  //// for "Brain UI" - x error
        arrayProgressMsg.data.push_back(msg2.data);  //// for "Brain UI" - y error
        arrayProgressMsg.data.push_back(msg3.data);  //// for "Brain UI" - deg error
        arrayProgressMsg.data.push_back(100);  //// for "Brain UI" - progress
        pub_CaliErrorProgress.publish(arrayProgressMsg);

        NaviFra::Pos result;
        result.SetXm(o_icp_pos.GetXm() - odom_temp.GetXm());
        result.SetYm(o_icp_pos.GetYm() - odom_temp.GetYm());
        result.SetDeg(o_icp_pos.GetDeg());

        str_ = "";
    }
    pub_.publish(cmd);
}

NaviFra::Pos Calibrator::PosConvert(const geometry_msgs::PoseStamped& input)
{
    NaviFra::Pos o_pos;
    o_pos.SetXm(input.pose.position.x);
    o_pos.SetYm(input.pose.position.y);
    o_pos.SetQuaternion(input.pose.orientation.w, input.pose.orientation.x, input.pose.orientation.y, input.pose.orientation.z);
    return o_pos;
}

NaviFra::Pos Calibrator::PosConvert(const nav_msgs::Odometry::ConstPtr& input)
{
    NaviFra::Pos o_pos;
    o_pos.SetXm(input->pose.pose.position.x);
    o_pos.SetYm(input->pose.pose.position.y);
    o_pos.SetQuaternion(
        input->pose.pose.orientation.w, input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z);
    return o_pos;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_cali");
    Calibrator optimizer;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    // ros::spin();
    return 0;
}