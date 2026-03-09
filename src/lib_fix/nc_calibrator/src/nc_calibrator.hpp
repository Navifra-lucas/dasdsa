#ifndef NAVIFRA_CALIBARTION_H_
#define NAVIFRA_CALIBARTION_H_

#include "core/util/logger.hpp"
#include "core_calculator/core_calculator.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "icp/point_to_plane_icp.hpp"
#include "nav_msgs/Odometry.h"
#include "opencv2/opencv.hpp"
#include "pos/pos.hpp"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/MarkerArray.h"

#include <laser_geometry/laser_geometry.h>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/tf.h>

#include <chrono>
#include <fstream>
#include <queue>
#include <string>
#include <vector>

using namespace std;
class Calibrator {
public:
    Calibrator();
    /**
     * @brief 앞쪽 라이다 정보 수신
     *
     * @param msg
     */

    /**
     * @brief 엔코더 기반 x,y 수신
     *
     * @param msg
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void calCallback(const geometry_msgs::Twist::ConstPtr& msg);

    /**
     * @brief 초기화
     *
     */
    void Init();

    /**
     * @brief ICP를 이용한 칼리브레이션 명령 수신
     *
     * @param msg
     */
    void cmdCallback(const std_msgs::String::ConstPtr& msg);

    /**
     * @brief ICP를 이용하여 초기 라이다 포인트와 매칭하여, 오돔값과 비교
     *
     * @param o_predict_robot_pos
     * @param start_lidar
     * @param new_lidar
     * @param mode
     * @return NaviFra::Pos
     */
    NaviFra::SimplePos CalIcpPoint(
        NaviFra::Pos& o_predict_robot_pos, vector<NaviFra::SimplePos>& start_lidar, vector<NaviFra::SimplePos>& new_lidar, int mode = 0);

    pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    /**
     * @brief 회전 칼리브레이션 실행
     *
     */
    void RotateCmd();

    /**
     * @brief 직진 칼리브레이션 실행
     *
     */
    void GoCmd();

    // 아래 부분은 칼리브레이션 결과를 바탕으로 로봇의 기구학에 따라서, 기구학 파라미터 보정값 계산 함수
    void CalDDType(float f_x, float f_y, float f_deg);
    void CalTriType(float f_x, float f_y, float f_deg);
    void CalQuadType(float f_x, float f_y, float f_deg);

    // 시각화
    void scanCloudCallback(const sensor_msgs::PointCloudConstPtr& point_cloud);
    void scanFrontCallback(const sensor_msgs::PointCloudConstPtr& point_cloud);
    void scanRearCallback(const sensor_msgs::PointCloudConstPtr& point_cloud);
    void CalLidar();
    float CalCostIcp(vector<sensor_msgs::LaserScan>& vec_data, float f_angle_inc);
    void CalYaw();
    float CalCostIcp2(vector<sensor_msgs::LaserScan>& vec_data, float f_angle_rad, bool b_front);
    std::vector<NaviFra::SimplePos> LStoPS(const sensor_msgs::LaserScan& laser_scan, float f_angle);

    NaviFra::Pos PosConvert(const nav_msgs::Odometry::ConstPtr& ros_pos);
    NaviFra::Pos PosConvert(const geometry_msgs::PoseStamped& ros_pos);

private:  // private으로 NodeHandle과 publisher, subscriber를 선언한다.
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher pub_scan;
    ros::Publisher filtered_obstacles_publisher1_;
    ros::Publisher filtered_obstacles_publisher0_;
    ros::Publisher filtered_obstacles_publisher2_;
    ros::Publisher filtered_obstacles_publisher2_2;

    ros::Publisher filtered_obstacles_publisher3_;
    ros::Publisher filtered_obstacles_publisher3_2;
    ros::Publisher pub_error_x;
    ros::Publisher pub_error_xa;
    ros::Publisher pub_error_a;
    ros::Publisher pub_dist;
    ros::Publisher pub_result;

    ros::Publisher pub_CaliErrorProgress;  // for "Brain UI"
    ros::Publisher pub_CaliResult;  // for "Brain UI"

    ros::Subscriber sub_;
    ros::Subscriber sub2_;
    ros::Subscriber sub_scan1_, sub_scan2_;
    ros::Subscriber sub_scan_cloud_;
    ros::Subscriber sub_cloud_front_;
    ros::Subscriber sub_cloud_rear_;
    ros::Subscriber sub_cal_;
    ros::Subscriber sub_scan_laser_;
    ros::Subscriber sub_scan_laser2_;

    nav_msgs::Odometry odom_start_;
    nav_msgs::Odometry odom_;
    double d_yaw_;
    double d_yaw_start_;
    double d_yaw_pre_;
    double d_yaw_sum_ = 0;
    bool b_cmd_ = false;
    bool b_filter_use_;
    std::vector<NaviFra::SimplePos> vec_lidar_;
    std::vector<NaviFra::SimplePos> vec_lidar_start_;

    std::vector<NaviFra::SimplePos> vec_lidar_front_;
    std::vector<NaviFra::SimplePos> vec_lidar_front_start_;

    std::vector<NaviFra::SimplePos> vec_lidar_rear_;
    std::vector<NaviFra::SimplePos> vec_lidar_rear_start_;

    std::vector<NaviFra::Pos> odom_trj;

    std::mutex mtx_lock_;
    std::vector<NaviFra::Pos> vec_icp_db_;
    geometry_msgs::TransformStamped tf_scan1_;
    geometry_msgs::TransformStamped tf_scan2_;
    geometry_msgs::TransformStamped tf_base_odom_;
    NaviFra::Pos o_odom_pos_;
    NaviFra::Pos o_odom_pos_start_;
    NaviFra::Pos robot_pos_;
    NaviFra::Pos robot_pos_start_;

    NaviFra::Pos o_icp_pos_;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point wait_ = std::chrono::steady_clock::now();
    string str_;
    string str_front_laser_link_;
    string str_rear_laser_link_;

    std::mutex mtx_lock_front_;
    std::mutex mtx_lock_rear_;
    std::vector<sensor_msgs::LaserScan> vec_front_scan_;
    std::vector<sensor_msgs::LaserScan> vec_rear_scan_;

    float f_front_increment_;
    float f_rear_increment_;

    tf::StampedTransform req_to_front_trans_;
    tf::StampedTransform req_to_rear_trans_;

    tf::TransformListener tfListener1_;
    tf::TransformListener tfListener2_;
    laser_geometry::LaserProjection projector_;

    float n_rotation_num_ = 1;
    float f_go_dist_ = 1;
    tf::TransformListener listener_;
    queue<string> seq_q_;

    // DD
    // TRI
    double d_angle_offset_;
    double d_wheel_encoder_;
    double d_base_length_;
    std::mutex mtx_lock_lidar_;
    sensor_msgs::PointCloud cloud_;
    sensor_msgs::PointCloud cloud_front_;
    sensor_msgs::PointCloud cloud_rear_;

    float f_matching_rmse_;
};
#endif