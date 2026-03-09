/**
 * @brief Pallet Template Capture Node
 *
 * Captures 3D point cloud data of a pallet placed in front of the sensor
 * and saves it as a PLY template file for ICP-based pallet detection.
 *
 * Usage:
 *   rosrun answer pallet_template_capture \
 *       _name:=type3 _width:=1.8 _height:=2.0 \
 *       _topic:=/sick_visionary_t_mini/points
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <sstream>
#include <thread>

class PalletTemplateCapture {
public:
    PalletTemplateCapture(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh)
        , capturing_(false)
        , captured_frames_(0)
        , accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        // Load parameters
        if (!pnh.getParam("name", name_)) {
            ROS_ERROR("Required parameter '~name' not set. (e.g. _name:=type3)");
            ros::shutdown();
            return;
        }
        if (!pnh.getParam("width", width_)) {
            ROS_ERROR(
                "Required parameter '~width' not set. (e.g. _width:=1.8)");
            ros::shutdown();
            return;
        }
        if (!pnh.getParam("height", height_)) {
            ROS_ERROR(
                "Required parameter '~height' not set. (e.g. _height:=2.0)");
            ros::shutdown();
            return;
        }

        pnh.param<std::string>(
            "topic", topic_, "/sick_visionary_t_mini/points");
        pnh.param<int>("frames", target_frames_, 3);  // 기본 3프레임 연속 누적
        pnh.param<double>("min_range", min_range_, 1.5);  // 카메라로부터 1.5m 이내 제거
        pnh.param<double>("max_range", max_range_, 5.0);
        pnh.param<int>("sor_k", sor_k_, 10);
        pnh.param<double>("sor_std", sor_std_, 1.0);
        pnh.param<std::string>("robot_name", robot_name_, "");
        pnh.param<bool>("normalize", normalize_, false);
        pnh.param<double>("ransac_threshold", ransac_threshold_, 0.02);
        pnh.param<bool>("ground_removal", ground_removal_, true);
        pnh.param<double>("ground_ransac_threshold", ground_ransac_threshold_, 0.03);
        pnh.param<int>("ground_removal_max_iter", ground_removal_max_iter_, 3);
        pnh.param<double>("front_face_depth", front_face_depth_, 0.6);  // 전면부 유지 깊이 [m]
        pnh.param<int>("depth_axis", depth_axis_, 2);  // 0=X, 1=Y, 2=Z (카메라 전방축, 정면=이 축 최소값)
        pnh.param<std::string>("base_frame", base_frame_, "base_link");  // base_link 기준 시각화용 (base_link에서 카메라 정면 = -X)
        pnh.param<double>("debug_range_min", debug_range_min_, 2.0);  // [m] 디버그용: 전방축 기준 이 거리 이상만 발행
        pnh.param<bool>("debug_depth_forward_positive", debug_depth_forward_positive_, true);  // 전방이 양수면 true, 음수면 false (예: Y전방=- → false)
        pnh.param<double>("debug_lateral_half", debug_lateral_half_, 1.0);  // [m] 디버그용: 레터럴(카메라 X축) ±이 값만 유지 (-1~+1 m)
        pnh.param<bool>("use_cluster_largest", use_cluster_largest_, true);  // 가장 큰 클러스터만 유지 (노이즈 제거)
        pnh.param<double>("cluster_tolerance", cluster_tolerance_, 0.3);    // [m] 유클리드 클러스터 거리
        pnh.param<int>("min_cluster_size", min_cluster_size_, 300);          // 최소 클러스터 포인트 수
        pnh.param<double>("save_roll_deg", save_roll_deg_, 90.0);   // [deg] 저장 시 X축(roll) 회전
        pnh.param<double>("save_pitch_deg", save_pitch_deg_, 0.0);   // [deg] 저장 시 Y축(pitch) 회전
        pnh.param<double>("save_yaw_deg", save_yaw_deg_, 90.0);       // [deg] 저장 시 Z축(yaw) 회전
        pnh.param<double>("save_voxel_leaf", save_voxel_leaf_, 0.01);  // [m] 저장 전 voxel 다운샘플 크기 (0=미적용)

        // Resolve template directory
        std::string pkg_path = ros::package::getPath("answer");
        template_dir_ = pkg_path + "/template/";
        if (robot_name_.empty()) {
            config_path_ = pkg_path + "/config/default/pallet_detector.json";
        }
        else {
            config_path_ = pkg_path + "/config/robot/" + robot_name_ +
                "/pallet_detector.json";
        }

        // Subscribe to point cloud
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            topic_, 10,
            &PalletTemplateCapture::PointCloudCallback, this);

        // RViz 시각화용 퍼블리셔 (선택된 포인트클라우드, latch)
        pub_cloud_selected_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                "pallet_template_capture/cloud_selected", 1, true);
        pub_cloud_normalized_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                "pallet_template_capture/cloud_normalized", 1, true);
        pub_cloud_base_link_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                "pallet_template_capture/cloud_selected_base_link", 1, true);
        pub_cloud_debug_range_ =
            nh_.advertise<sensor_msgs::PointCloud2>(
                "pallet_template_capture/cloud_debug_range_min", 1, true);  // 카메라 기준 debug_range_min 이상만 (시각화용)

        tf_buffer_.reset(new tf2_ros::Buffer);
        tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

        PrintHeader();
    }

    void Run()
    {
        std::string line;
        while (ros::ok()) {
            std::cout << "\n[CAPTURE] 팔레트를 센서 정면에 위치시킨 후 Enter를 "
                         "누르세요 (q: 종료) > "
                      << std::flush;

            if (!std::getline(std::cin, line)) {
                break;
            }
            if (line == "q" || line == "Q") {
                std::cout << "[INFO] 종료합니다.\n";
                break;
            }

            if (!last_cloud_received_) {
                std::cout
                    << "[WARN] 아직 포인트 클라우드 데이터를 수신하지 못했습니다.\n"
                    << "       토픽 '" << topic_ << "' 을 확인하세요.\n";
                continue;
            }

            // Start capture
            captured_frames_ = 0;
            accumulated_cloud_->clear();
            capturing_ = true;

            std::cout << "[CAPTURE] " << target_frames_
                      << "프레임 캡처 시작...\n";

            // Wait for frames
            ros::Rate rate(30);
            while (ros::ok() && captured_frames_ < target_frames_) {
                ros::spinOnce();
                rate.sleep();
            }
            capturing_ = false;

            if (accumulated_cloud_->empty()) {
                std::cout << "[ERROR] 캡처된 포인트가 없습니다.\n";
                continue;
            }

            std::cout << "[INFO] 캡처 완료: " << accumulated_cloud_->size()
                      << " points (raw)\n";

            // Process and save
            auto result = ProcessCloud(accumulated_cloud_);
            if (result->empty()) {
                std::cout << "[ERROR] 전처리 후 포인트가 없습니다.\n";
                continue;
            }

            // Normalize: align front face, center, remove yaw/lateral offset
            auto normalized = NormalizeTemplate(result);
            if (normalized->empty()) {
                std::cout << "[ERROR] 정규화 후 포인트가 없습니다.\n";
                continue;
            }

            // 저장 시 roll / pitch / yaw 회전 적용 (Rx * Ry * Rz 순서)
            pcl::PointCloud<pcl::PointXYZ>::Ptr to_save = normalized;
            const double kDegToRad = 3.14159265358979323846 / 180.0;
            bool any_rotation = (std::abs(save_roll_deg_) > 1e-6 || std::abs(save_pitch_deg_) > 1e-6 ||
                                std::abs(save_yaw_deg_) > 1e-6);
            if (any_rotation) {
                Eigen::Matrix3f Rx = Eigen::AngleAxisf(
                    static_cast<float>(save_roll_deg_ * kDegToRad), Eigen::Vector3f::UnitX()).toRotationMatrix();
                Eigen::Matrix3f Ry = Eigen::AngleAxisf(
                    static_cast<float>(save_pitch_deg_ * kDegToRad), Eigen::Vector3f::UnitY()).toRotationMatrix();
                Eigen::Matrix3f Rz = Eigen::AngleAxisf(
                    static_cast<float>(save_yaw_deg_ * kDegToRad), Eigen::Vector3f::UnitZ()).toRotationMatrix();
                Eigen::Matrix3f R = Rz * Ry * Rx;
                pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_save(new pcl::PointCloud<pcl::PointXYZ>);
                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                T.block<3, 3>(0, 0) = R;
                pcl::transformPointCloud(*normalized, *rotated_save, T);
                to_save = rotated_save;
                std::cout << "[SAVE] Roll " << save_roll_deg_ << " deg, Pitch " << save_pitch_deg_
                          << " deg, Yaw " << save_yaw_deg_ << " deg 적용\n";
            }

            // Voxel 다운샘플 (0.01 m)
            if (save_voxel_leaf_ > 1e-6 && !to_save->empty()) {
                float leaf = static_cast<float>(save_voxel_leaf_);
                pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::VoxelGrid<pcl::PointXYZ> vg;
                vg.setInputCloud(to_save);
                vg.setLeafSize(leaf, leaf, leaf);
                vg.filter(*voxeled);
                std::cout << "[SAVE] Voxel " << save_voxel_leaf_ << " m: " << to_save->size()
                          << " -> " << voxeled->size() << " pts\n";
                to_save = voxeled;
            }

            std::string ply_path = SavePLY(to_save);
            if (ply_path.empty()) {
                continue;
            }

            SaveFrontViewImage(to_save, ply_path);
            SaveDiagonalViewImage(to_save, ply_path);
            UpdateConfig();
            PrintSummary(to_save, ply_path);

            // RViz 시각화: 선택된 클라우드(센서 프레임), 정규화된 클라우드(template 프레임), base_link 기준 (base_link에서 카메라 정면 = -X)
            const std::string sensor_frame = last_frame_id_.empty() ? "camera_optical_frame" : last_frame_id_;
            PublishPointCloud(result, pub_cloud_selected_, sensor_frame);
            PublishPointCloud(to_save, pub_cloud_normalized_, "pallet_template");

            pcl::PointCloud<pcl::PointXYZ>::Ptr result_base_link =
                TransformToBaseLink(result, sensor_frame);
            if (result_base_link) {
                PublishPointCloud(result_base_link, pub_cloud_base_link_, base_frame_);
                std::cout << "[RViz] base_link 기준 발행: pallet_template_capture/cloud_selected_base_link\n";
            }
            else {
                std::cout << "[RViz] TF " << sensor_frame << " -> " << base_frame_
                          << " 없음. base_link 토픽 미발행. (roscore + TF 퍼블리시 중인지 확인)\n";
            }
            std::cout << "[RViz] 토픽: cloud_selected (" << sensor_frame
                      << "), cloud_normalized (pallet_template), cloud_selected_base_link ("
                      << base_frame_ << ")\n";
        }
    }

private:
    void PublishPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const ros::Publisher& pub,
        const std::string& frame_id)
    {
        if (!cloud || cloud->empty() || !pub) {
            return;
        }
        sensor_msgs::PointCloud2 msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id;
        msg.height = 1;
        msg.width = cloud->size();
        msg.is_dense = true;
        msg.is_bigendian = false;
        msg.point_step = 12;  // 3 * sizeof(float)
        msg.row_step = msg.point_step * msg.width;
        msg.fields.resize(3);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[0].count = 1;
        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[1].count = 1;
        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[2].count = 1;
        msg.data.resize(msg.row_step * msg.height);
        for (size_t i = 0; i < cloud->size(); ++i) {
            memcpy(&msg.data[i * msg.point_step], &cloud->points[i].x, 12);
        }
        pub.publish(msg);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr TransformToBaseLink(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& source_frame)
    {
        if (!cloud || cloud->empty() || !tf_buffer_) {
            return nullptr;
        }
        try {
            geometry_msgs::TransformStamped ts = tf_buffer_->lookupTransform(
                base_frame_, source_frame, ros::Time(0), ros::Duration(0.5));
            const auto& t = ts.transform.translation;
            const auto& r = ts.transform.rotation;
            Eigen::Quaternionf q(r.w, r.x, r.y, r.z);
            Eigen::Vector3f trans(t.x, t.y, t.z);
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block<3, 3>(0, 0) = q.toRotationMatrix();
            T.block<3, 1>(0, 3) = trans;
            pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*cloud, *out, T);
            return out;
        }
        catch (const tf2::TransformException& e) {
            return nullptr;
        }
    }

    void PrintHeader()
    {
        std::cout
            << "\n"
            << "========================================\n"
            << "  Pallet Template Capture Tool\n"
            << "========================================\n"
            << "  Name   : " << name_ << "\n"
            << "  Width  : " << width_ << " m\n"
            << "  Height : " << height_ << " m\n"
            << "  Topic  : " << topic_ << "\n"
            << "  Frames : " << target_frames_ << "\n"
            << "  PLY 파이프라인 = cloud_debug_range_min 동일\n"
            << "  전방축 " << (depth_axis_ == 0 ? "X" : depth_axis_ == 1 ? "Y" : "Z")
            << " " << debug_range_min_ << " ~ " << max_range_ << " m → 바닥 제거 → 레터럴 X ±"
            << debug_lateral_half_ << " m → SOR → 최대 클러스터 → 전면 " << front_face_depth_ << " m\n"
            << "  Ground removal : " << (ground_removal_ ? "ON" : "OFF")
            << " (RANSAC thr=" << ground_ransac_threshold_ << ")\n"
            << "  SOR    : K=" << sor_k_ << ", std=" << sor_std_ << "\n"
            << "  Cluster: " << (use_cluster_largest_ ? "ON" : "OFF")
            << " (tolerance=" << cluster_tolerance_ << " m, min_size=" << min_cluster_size_ << ")\n"
            << "  Normalize : " << (normalize_ ? "ON" : "OFF")
            << " (RANSAC thr=" << ransac_threshold_ << ")\n"
            << "  Save RPY : roll=" << save_roll_deg_ << " pitch=" << save_pitch_deg_
            << " yaw=" << save_yaw_deg_ << " deg\n"
            << "  Voxel    : " << save_voxel_leaf_ << " m (저장 전)\n"
            << "  Output : " << template_dir_ << name_ << ".ply\n"
            << "  Config : " << config_path_ << "\n"
            << "  RViz   : cloud_selected, cloud_normalized, cloud_selected_base_link ("
            << base_frame_ << ")\n"
            << "  Debug  : cloud_debug_range_min (실시간 동일 파이프라인 미리보기)\n"
            << "========================================\n";
    }

    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        last_cloud_received_ = true;
        last_frame_id_ = msg->header.frame_id;

        // 디버그용: 전방축 기준 거리 필터 후, 바닥 감지하여 바닥 위 포인트만 발행 (RViz 시각화)
        if (depth_axis_ >= 0 && depth_axis_ <= 2) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_debug(new pcl::PointCloud<pcl::PointXYZ>);
            sensor_msgs::PointCloud2ConstIterator<float> mx(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> my(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> mz(*msg, "z");
            for (size_t i = 0; i < msg->height * msg->width; ++i, ++mx, ++my, ++mz) {
                if (!std::isfinite(*mx) || !std::isfinite(*my) || !std::isfinite(*mz))
                    continue;
                float depth_val = (depth_axis_ == 0) ? *mx : (depth_axis_ == 1) ? *my : *mz;
                bool in_range = debug_depth_forward_positive_
                    ? (depth_val >= debug_range_min_ && depth_val <= max_range_)
                    : (depth_val <= -debug_range_min_ && depth_val >= -max_range_);
                if (in_range) {
                    pcl::PointXYZ pt;
                    pt.x = *mx;
                    pt.y = *my;
                    pt.z = *mz;
                    cloud_debug->push_back(pt);
                }
            }
            // 바닥 감지(RANSAC) 후 바닥 위 포인트만 유지
            if (cloud_debug->size() > 100) {
                pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(ground_ransac_threshold_);
                seg.setInputCloud(cloud_debug);
                seg.segment(*inliers, *coeffs);
                if (!inliers->indices.empty()) {
                    float nx = coeffs->values[0], ny = coeffs->values[1], nz = coeffs->values[2];
                    float abs_nd = (depth_axis_ == 0) ? std::abs(nx) : (depth_axis_ == 1) ? std::abs(ny) : std::abs(nz);
                    if (abs_nd < 0.7f) {  // 수평면 = 바닥
                        float d = coeffs->values[3];
                        float camera_side = (d >= 0.0f) ? 1.0f : -1.0f;
                        pcl::PointCloud<pcl::PointXYZ>::Ptr above_floor(new pcl::PointCloud<pcl::PointXYZ>);
                        for (const auto& pt : cloud_debug->points) {
                            float sd = nx * pt.x + ny * pt.y + nz * pt.z + d;
                            if (sd * camera_side > ground_ransac_threshold_)
                                above_floor->push_back(pt);
                        }
                        cloud_debug = above_floor;
                    }
                }
            }
            // 레터럴(카메라 X축) ±debug_lateral_half_ [m] 만 유지
            if (debug_lateral_half_ > 0 && !cloud_debug->empty()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr lateral_filtered(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& pt : cloud_debug->points) {
                    if (std::abs(pt.x) <= debug_lateral_half_)
                        lateral_filtered->push_back(pt);
                }
                cloud_debug = lateral_filtered;
            }
            // SOR 후 가장 큰 클러스터만 (PLY 저장과 동일)
            if (!cloud_debug->empty()) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(cloud_debug);
                sor.setMeanK(sor_k_);
                sor.setStddevMulThresh(sor_std_);
                sor.filter(*sor_cloud);
                cloud_debug = sor_cloud;
            }
            if (use_cluster_largest_ && cloud_debug->size() > static_cast<size_t>(min_cluster_size_)) {
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(cloud_debug);
                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(cluster_tolerance_);
                ec.setMinClusterSize(min_cluster_size_);
                ec.setMaxClusterSize(1000000);
                ec.setSearchMethod(tree);
                ec.setInputCloud(cloud_debug);
                ec.extract(cluster_indices);
                if (!cluster_indices.empty()) {
                    size_t largest_idx = 0;
                    for (size_t i = 1; i < cluster_indices.size(); ++i) {
                        if (cluster_indices[i].indices.size() > cluster_indices[largest_idx].indices.size())
                            largest_idx = i;
                    }
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    for (int idx : cluster_indices[largest_idx].indices)
                        cluster_cloud->push_back(cloud_debug->points[idx]);
                    cloud_debug = cluster_cloud;
                }
            }
            // 카메라 기준 클러스터 앞부분 front_face_depth_ (0.4 m) 만 유지
            if (front_face_depth_ > 0 && !cloud_debug->empty() && depth_axis_ >= 0 && depth_axis_ <= 2) {
                auto depth_coord = [this](const pcl::PointXYZ& pt) -> float {
                    if (depth_axis_ == 0) return pt.x;
                    if (depth_axis_ == 1) return pt.y;
                    return pt.z;
                };
                float front_val = debug_depth_forward_positive_
                    ? std::numeric_limits<float>::max()
                    : -std::numeric_limits<float>::max();
                for (const auto& pt : cloud_debug->points) {
                    float d = depth_coord(pt);
                    if (debug_depth_forward_positive_) { if (d < front_val) front_val = d; }
                    else { if (d > front_val) front_val = d; }
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                if (debug_depth_forward_positive_) {
                    float depth_max_keep = front_val + front_face_depth_;
                    for (const auto& pt : cloud_debug->points)
                        if (depth_coord(pt) <= depth_max_keep) front_cloud->push_back(pt);
                } else {
                    float depth_min_keep = front_val - front_face_depth_;
                    for (const auto& pt : cloud_debug->points)
                        if (depth_coord(pt) >= depth_min_keep) front_cloud->push_back(pt);
                }
                cloud_debug = front_cloud;
            }
            if (!cloud_debug->empty()) {
                PublishPointCloud(cloud_debug, pub_cloud_debug_range_, msg->header.frame_id);
            }
        }

        if (!capturing_) {
            return;
        }

        // 캡처: 디버그와 동일하게 전방축(depth_axis) 기준 거리만 필터 (유클리드 X)
        pcl::PointCloud<pcl::PointXYZ>::Ptr frame(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2ConstIterator<float> mx2(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> my2(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> mz2(*msg, "z");
        for (size_t i = 0; i < msg->height * msg->width; ++i, ++mx2, ++my2, ++mz2) {
            if (!std::isfinite(*mx2) || !std::isfinite(*my2) || !std::isfinite(*mz2))
                continue;
            float depth_val = (depth_axis_ == 0) ? *mx2 : (depth_axis_ == 1) ? *my2 : *mz2;
            bool in_range = debug_depth_forward_positive_
                ? (depth_val >= debug_range_min_ && depth_val <= max_range_)
                : (depth_val <= -debug_range_min_ && depth_val >= -max_range_);
            if (in_range) {
                pcl::PointXYZ pt;
                pt.x = *mx2;
                pt.y = *my2;
                pt.z = *mz2;
                frame->push_back(pt);
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_copy(new pcl::PointCloud<pcl::PointXYZ>);
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            *accumulated_cloud_ += *frame;
            *accumulated_copy = *accumulated_cloud_;
        }
        captured_frames_++;
        std::cout << "  [" << captured_frames_ << "/" << target_frames_
                  << "] " << frame->size() << " points captured (전방축 "
                  << (depth_axis_ == 0 ? "X" : depth_axis_ == 1 ? "Y" : "Z")
                  << " " << debug_range_min_ << "~" << max_range_ << " m)\n";
        // 3회 누적 데이터를 동일 파이프라인으로 처리해 cloud_debug_range_min에 표시
        if (!accumulated_copy->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr processed = ProcessCloud(accumulated_copy);
            if (!processed->empty()) {
                PublishPointCloud(processed, pub_cloud_debug_range_, last_frame_id_);
            }
        }
    }

    // cloud_debug_range_min과 동일 파이프라인: 전방축 거리 → 바닥 제거 → 레터럴 ± → SOR → 최대 클러스터
    pcl::PointCloud<pcl::PointXYZ>::Ptr ProcessCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud = *input;

        // Step 1: 바닥 감지(RANSAC 1회) 후 바닥 위만 유지
        if (ground_removal_ && cloud->size() > 100) {
            pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ground_ransac_threshold_);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coeffs);
            if (!inliers->indices.empty()) {
                float nx = coeffs->values[0], ny = coeffs->values[1], nz = coeffs->values[2];
                float abs_nd = (depth_axis_ == 0) ? std::abs(nx) : (depth_axis_ == 1) ? std::abs(ny) : std::abs(nz);
                if (abs_nd < 0.7f) {
                    float d = coeffs->values[3];
                    float camera_side = (d >= 0.0f) ? 1.0f : -1.0f;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr above_floor(new pcl::PointCloud<pcl::PointXYZ>);
                    for (const auto& pt : cloud->points) {
                        float sd = nx * pt.x + ny * pt.y + nz * pt.z + d;
                        if (sd * camera_side > ground_ransac_threshold_)
                            above_floor->push_back(pt);
                    }
                    std::cout << "[PROCESS] 바닥 제거 (RANSAC thr=" << ground_ransac_threshold_
                              << "): " << cloud->size() << " -> " << above_floor->size() << " pts\n";
                    cloud = above_floor;
                }
            }
        }

        // Step 2: 레터럴(카메라 X축) ±debug_lateral_half_ [m] 만 유지
        if (debug_lateral_half_ > 0 && !cloud->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr lateral_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& pt : cloud->points) {
                if (std::abs(pt.x) <= debug_lateral_half_)
                    lateral_filtered->push_back(pt);
            }
            std::cout << "[PROCESS] 레터럴 X ±" << debug_lateral_half_
                      << " m: " << cloud->size() << " -> " << lateral_filtered->size() << " pts\n";
            cloud = lateral_filtered;
        }

        // Step 3: SOR
        std::cout << "[PROCESS] SOR (K=" << sor_k_ << ", std=" << sor_std_ << ")...\n";
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(sor_k_);
        sor.setStddevMulThresh(sor_std_);
        sor.filter(*sor_cloud);
        std::cout << "  " << cloud->size() << " -> " << sor_cloud->size() << " points\n";
        cloud = sor_cloud;

        // Step 4: 가장 큰 클러스터만 유지
        if (use_cluster_largest_ && cloud->size() > static_cast<size_t>(min_cluster_size_)) {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(cluster_tolerance_);
            ec.setMinClusterSize(min_cluster_size_);
            ec.setMaxClusterSize(1000000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);
            if (!cluster_indices.empty()) {
                size_t largest_idx = 0;
                for (size_t i = 1; i < cluster_indices.size(); ++i) {
                    if (cluster_indices[i].indices.size() > cluster_indices[largest_idx].indices.size())
                        largest_idx = i;
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (int idx : cluster_indices[largest_idx].indices)
                    cluster_cloud->push_back(cloud->points[idx]);
                std::cout << "[PROCESS] Clustering (tolerance=" << cluster_tolerance_
                          << " m, min_size=" << min_cluster_size_ << "): "
                          << cluster_indices.size() << " clusters, largest "
                          << cluster_cloud->size() << " pts\n";
                cloud = cluster_cloud;
            }
        }

        // Step 5: 카메라 기준 클러스터 앞부분 front_face_depth_ [m] 만 유지
        if (front_face_depth_ > 0 && !cloud->empty() && depth_axis_ >= 0 && depth_axis_ <= 2) {
            auto depth_coord = [this](const pcl::PointXYZ& pt) -> float {
                if (depth_axis_ == 0) return pt.x;
                if (depth_axis_ == 1) return pt.y;
                return pt.z;
            };
            float front_val = debug_depth_forward_positive_
                ? std::numeric_limits<float>::max()
                : -std::numeric_limits<float>::max();
            for (const auto& pt : cloud->points) {
                float d = depth_coord(pt);
                if (debug_depth_forward_positive_) { if (d < front_val) front_val = d; }
                else { if (d > front_val) front_val = d; }
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (debug_depth_forward_positive_) {
                float depth_max_keep = front_val + front_face_depth_;
                for (const auto& pt : cloud->points)
                    if (depth_coord(pt) <= depth_max_keep) front_cloud->push_back(pt);
            } else {
                float depth_min_keep = front_val - front_face_depth_;
                for (const auto& pt : cloud->points)
                    if (depth_coord(pt) >= depth_min_keep) front_cloud->push_back(pt);
            }
            std::cout << "[PROCESS] 전면 " << front_face_depth_ << " m만 유지: "
                      << cloud->size() << " -> " << front_cloud->size() << " pts\n";
            cloud = front_cloud;
        }

        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr NormalizeTemplate(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
    {
        if (!normalize_) {
            return input;
        }

        std::cout << "[NORMALIZE] 팔레트 정면 정렬 시작...\n";

        // Step 1: Estimate forward (depth) direction from centroid
        // Sensor is at origin, pallet is in front → centroid points forward
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*input, centroid);
        Eigen::Vector3f forward_hint = centroid.head<3>().normalized();

        std::cout << "  포인트 중심: (" << centroid[0] << ", "
                  << centroid[1] << ", " << centroid[2] << ")\n";
        std::cout << "  전방 추정: (" << forward_hint.x() << ", "
                  << forward_hint.y() << ", " << forward_hint.z() << ")\n";

        // Step 2: Iterative RANSAC - find pallet face, skip ground/floor
        // Pallet face normal is approximately parallel to forward direction.
        // Ground/floor normal is approximately perpendicular to forward.
        Eigen::Vector3f pallet_normal;
        pcl::PointIndices::Ptr pallet_inliers(new pcl::PointIndices);
        bool found_face = false;

        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(
            new pcl::PointCloud<pcl::PointXYZ>);
        *remaining = *input;

        for (int attempt = 0; attempt < 3 && !found_face; ++attempt) {
            pcl::ModelCoefficients::Ptr coefficients(
                new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_threshold_);
            seg.setInputCloud(remaining);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty()) {
                std::cout << "  [시도 " << attempt + 1
                          << "] 평면을 찾을 수 없습니다.\n";
                break;
            }

            Eigen::Vector3f normal(
                coefficients->values[0],
                coefficients->values[1],
                coefficients->values[2]);

            float alignment = std::abs(normal.dot(forward_hint));

            std::cout << "  [시도 " << attempt + 1 << "] 평면 법선=("
                      << normal.x() << ", " << normal.y() << ", "
                      << normal.z() << "), 전방 정렬도=" << alignment
                      << ", inliers=" << inliers->indices.size() << "\n";

            if (alignment > 0.5f) {
                // Normal is roughly parallel to forward → pallet face
                pallet_normal = normal;
                if (pallet_normal.dot(forward_hint) > 0) {
                    pallet_normal = -pallet_normal;
                }
                pallet_inliers = inliers;
                found_face = true;
                std::cout << "  → 팔레트 정면 감지!\n";
            }
            else {
                // Normal is roughly perpendicular to forward → ground/wall
                std::cout << "  → 바닥/벽면으로 판단, 제거 후 재시도\n";
                pcl::PointCloud<pcl::PointXYZ>::Ptr next(
                    new pcl::PointCloud<pcl::PointXYZ>);
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(remaining);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*next);
                remaining = next;
            }
        }

        if (!found_face) {
            std::cout << "[WARN] 팔레트 정면을 감지하지 못했습니다. "
                      << "전방 방향 기준으로 정규화합니다.\n";
            pallet_normal = -forward_hint;
        }

        // Step 3: Build rotation matrix for target frame
        // Target: X=depth(into pallet), Y=lateral, Z=up
        // This matches the ICP matcher convention (initial_guess X = depth)
        Eigen::Vector3f x_axis = -pallet_normal;
        x_axis.normalize();

        Eigen::Vector3f up_hint(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f z_axis = up_hint - up_hint.dot(x_axis) * x_axis;
        if (z_axis.norm() < 1e-6f) {
            // x_axis is nearly vertical, use -Y as fallback up hint
            up_hint = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
            z_axis = up_hint - up_hint.dot(x_axis) * x_axis;
        }
        z_axis.normalize();

        Eigen::Vector3f y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        Eigen::Matrix4f rot_transform = Eigen::Matrix4f::Identity();
        rot_transform.block<1, 3>(0, 0) = x_axis.transpose();
        rot_transform.block<1, 3>(1, 0) = y_axis.transpose();
        rot_transform.block<1, 3>(2, 0) = z_axis.transpose();

        pcl::PointCloud<pcl::PointXYZ>::Ptr rotated(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*input, *rotated, rot_transform);

        float angle_deg = std::acos(std::clamp(
            x_axis.dot(Eigen::Vector3f(1, 0, 0)), -1.0f, 1.0f)) *
            180.0f / M_PI;
        std::cout << "  회전 보정: " << angle_deg
                  << "° (법선 → -X 축 정렬)\n";

        // Step 4: Center template using front face inliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        if (found_face && !pallet_inliers->indices.empty()) {
            pcl::copyPointCloud(*rotated, *pallet_inliers, *ref_cloud);
        }
        else {
            ref_cloud = rotated;
        }

        Eigen::Vector4f face_centroid;
        pcl::compute3DCentroid(*ref_cloud, face_centroid);

        Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
        translation(0, 3) = -face_centroid[0];
        translation(1, 3) = -face_centroid[1];
        translation(2, 3) = -face_centroid[2];

        pcl::PointCloud<pcl::PointXYZ>::Ptr result(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*rotated, *result, translation);

        std::cout << "  이동 보정: dx=" << -face_centroid[0]
                  << ", dy=" << -face_centroid[1]
                  << ", dz=" << -face_centroid[2] << "\n";
        std::cout << "[NORMALIZE] 정규화 완료 (" << result->size()
                  << " points, 좌표계: X=depth, Y=lateral, Z=up)\n";

        return result;
    }

    void SaveFrontViewImage(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const std::string &ply_path)
    {
        if (cloud->empty()) {
            return;
        }

        // Image size and margin
        const int img_w = 800;
        const int img_h = 800;
        const int margin = 40;
        const int draw_w = img_w - 2 * margin;
        const int draw_h = img_h - 2 * margin;

        // Front view projects Y-Z plane (Y=lateral, Z=vertical)
        // Color by X (depth into pallet)
        float x_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        float y_min = x_min, y_max = x_max;
        float z_min = x_min, z_max = x_max;

        for (const auto &pt : cloud->points) {
            x_min = std::min(x_min, pt.x);
            x_max = std::max(x_max, pt.x);
            y_min = std::min(y_min, pt.y);
            y_max = std::max(y_max, pt.y);
            z_min = std::min(z_min, pt.z);
            z_max = std::max(z_max, pt.z);
        }

        float x_span = (x_max - x_min > 1e-6f) ? (x_max - x_min) : 1.0f;
        float y_span = y_max - y_min;
        float z_span = z_max - z_min;

        if (y_span < 1e-6f || z_span < 1e-6f) {
            std::cout << "[WARN] 포인트 분포가 너무 좁아 이미지를 생성할 수 없습니다.\n";
            return;
        }

        // Uniform scaling to fit the larger axis
        float scale = std::min(draw_w / y_span, draw_h / z_span);
        float cy = (y_min + y_max) / 2.0f;
        float cz = (z_min + z_max) / 2.0f;

        // Dark background
        cv::Mat img(img_h, img_w, CV_8UC3, cv::Scalar(30, 30, 30));

        // Draw each point: Y→horizontal, Z→vertical, color by X(depth)
        for (const auto &pt : cloud->points) {
            int px = static_cast<int>((pt.y - cy) * scale + img_w / 2.0f);
            int py = static_cast<int>(-(pt.z - cz) * scale + img_h / 2.0f);

            if (px < 0 || px >= img_w || py < 0 || py >= img_h) {
                continue;
            }

            // Color by depth (X): near/front(blue) → far/back(red)
            float t = (x_span > 1e-6f)
                ? std::clamp((pt.x - x_min) / x_span, 0.0f, 1.0f)
                : 0.5f;
            int r = static_cast<int>(255 * t);
            int b = static_cast<int>(255 * (1.0f - t));
            int g = static_cast<int>(255 * std::min(t, 1.0f - t) * 2);

            cv::circle(img, cv::Point(px, py), 1,
                       cv::Scalar(b, g, r), cv::FILLED);
        }

        // Draw axis labels and info text
        cv::putText(img,
                    "Front View (Y-Z plane, colored by X depth)",
                    cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(200, 200, 200), 1);

        // Scale bar
        float bar_m = 0.5f;  // 0.5m scale bar
        int bar_px = static_cast<int>(bar_m * scale);
        if (bar_px > 20 && bar_px < draw_w) {
            cv::Point bar_start(margin, img_h - 15);
            cv::Point bar_end(margin + bar_px, img_h - 15);
            cv::line(img, bar_start, bar_end,
                     cv::Scalar(200, 200, 200), 2);
            cv::putText(img,
                        std::to_string(bar_m).substr(0, 4) + "m",
                        cv::Point(margin, img_h - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(200, 200, 200), 1);
        }

        // Info: point count, bounding box
        std::ostringstream info;
        info << "Points: " << cloud->size()
             << "  |  X(depth): " << std::fixed << std::setprecision(0)
             << (x_span * 1000) << "mm"
             << "  Y(lateral): " << (y_span * 1000) << "mm"
             << "  Z(height): " << (z_span * 1000) << "mm";
        cv::putText(img, info.str(),
                    cv::Point(10, img_h - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(180, 180, 180), 1);

        // Color legend
        for (int i = 0; i < 100; i++) {
            float lt = i / 99.0f;
            int lr = static_cast<int>(255 * lt);
            int lb = static_cast<int>(255 * (1.0f - lt));
            int lg = static_cast<int>(255 * std::min(lt, 1.0f - lt) * 2);
            cv::line(img,
                     cv::Point(img_w - margin, margin + i * (draw_h / 100)),
                     cv::Point(img_w - margin + 15,
                               margin + i * (draw_h / 100)),
                     cv::Scalar(lb, lg, lr), 2);
        }
        cv::putText(img, "Near",
                    cv::Point(img_w - margin - 5, margin - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(100, 100, 255), 1);
        cv::putText(img, "Far",
                    cv::Point(img_w - margin - 5, margin + draw_h + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(255, 100, 100), 1);

        // Save image alongside PLY file
        std::string img_path = ply_path.substr(0, ply_path.size() - 4) + "_front.png";
        cv::imwrite(img_path, img);
        std::cout << "[IMAGE] 정면 뷰 저장: " << img_path << "\n";
    }

    void SaveDiagonalViewImage(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const std::string &ply_path)
    {
        if (cloud->empty()) {
            return;
        }

        const int img_w = 800;
        const int img_h = 800;
        const int margin = 50;
        const int draw_w = img_w - 2 * margin;
        const int draw_h = img_h - 2 * margin;

        // Isometric-like rotation: rotate 30° around X, then 45° around Y
        const float pitch = 30.0f * M_PI / 180.0f;  // tilt down
        const float yaw = 45.0f * M_PI / 180.0f;    // turn sideways

        float cp = std::cos(pitch), sp = std::sin(pitch);
        float cy = std::cos(yaw), sy = std::sin(yaw);

        // Combined rotation: Ry * Rx
        // | cy  sy*sp  sy*cp |
        // | 0   cp     -sp   |
        // | -sy cy*sp  cy*cp |
        auto project = [&](float x, float y, float z,
                           float &px, float &py, float &depth) {
            px = cy * x + sy * sp * y + sy * cp * z;
            py = cp * y - sp * z;
            depth = -sy * x + cy * sp * y + cy * cp * z;
        };

        // First pass: compute projected bounding box
        float px_min = std::numeric_limits<float>::max();
        float px_max = std::numeric_limits<float>::lowest();
        float py_min = px_min, py_max = px_max;
        float d_min = px_min, d_max = px_max;

        struct Projected {
            float px, py, depth;
        };
        std::vector<Projected> projected(cloud->size());

        for (size_t i = 0; i < cloud->size(); i++) {
            const auto &pt = cloud->points[i];
            float ppx, ppy, pd;
            project(pt.x, pt.y, pt.z, ppx, ppy, pd);
            projected[i] = {ppx, ppy, pd};

            px_min = std::min(px_min, ppx);
            px_max = std::max(px_max, ppx);
            py_min = std::min(py_min, ppy);
            py_max = std::max(py_max, ppy);
            d_min = std::min(d_min, pd);
            d_max = std::max(d_max, pd);
        }

        float px_span = px_max - px_min;
        float py_span = py_max - py_min;
        float d_span = (d_max - d_min > 1e-6f) ? (d_max - d_min) : 1.0f;

        if (px_span < 1e-6f || py_span < 1e-6f) {
            return;
        }

        float scale = std::min(draw_w / px_span, draw_h / py_span);
        float pcx = (px_min + px_max) / 2.0f;
        float pcy = (py_min + py_max) / 2.0f;

        // Sort by depth (far first) for painter's algorithm
        std::vector<size_t> indices(cloud->size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
                  [&](size_t a, size_t b) {
                      return projected[a].depth < projected[b].depth;
                  });

        cv::Mat img(img_h, img_w, CV_8UC3, cv::Scalar(30, 30, 30));

        // Draw points sorted by depth
        for (size_t idx : indices) {
            const auto &p = projected[idx];
            int ix = static_cast<int>((p.px - pcx) * scale + img_w / 2.0f);
            int iy = static_cast<int>(-(p.py - pcy) * scale + img_h / 2.0f);

            if (ix < 0 || ix >= img_w || iy < 0 || iy >= img_h) {
                continue;
            }

            float t = std::clamp((p.depth - d_min) / d_span, 0.0f, 1.0f);
            int r = static_cast<int>(255 * t);
            int b = static_cast<int>(255 * (1.0f - t));
            int g = static_cast<int>(255 * std::min(t, 1.0f - t) * 2);

            cv::circle(img, cv::Point(ix, iy), 1,
                       cv::Scalar(b, g, r), cv::FILLED);
        }

        // Draw 3D axis indicator at bottom-left
        int ax_orig_x = margin + 30;
        int ax_orig_y = img_h - margin - 30;
        float ax_len = 40.0f;

        auto drawAxis = [&](float dx, float dy, float dz,
                            const cv::Scalar &color, const std::string &label) {
            float ex, ey, ed;
            project(dx, dy, dz, ex, ey, ed);
            float len = std::sqrt(ex * ex + ey * ey);
            if (len > 1e-6f) {
                ex = ex / len * ax_len;
                ey = ey / len * ax_len;
            }
            cv::Point end(ax_orig_x + static_cast<int>(ex),
                          ax_orig_y - static_cast<int>(ey));
            cv::arrowedLine(img, cv::Point(ax_orig_x, ax_orig_y),
                            end, color, 2, cv::LINE_AA, 0, 0.2);
            cv::putText(img, label,
                        cv::Point(end.x + 3, end.y - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
        };

        drawAxis(1, 0, 0, cv::Scalar(80, 80, 255), "X(depth)");
        drawAxis(0, 1, 0, cv::Scalar(80, 255, 80), "Y(lat)");
        drawAxis(0, 0, 1, cv::Scalar(255, 80, 80), "Z(up)");

        cv::putText(img, "Diagonal View (30deg pitch, 45deg yaw)",
                    cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(200, 200, 200), 1);

        // Info text
        std::ostringstream info;
        info << "Points: " << cloud->size();
        cv::putText(img, info.str(),
                    cv::Point(10, img_h - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(180, 180, 180), 1);

        std::string img_path =
            ply_path.substr(0, ply_path.size() - 4) + "_diagonal.png";
        cv::imwrite(img_path, img);
        std::cout << "[IMAGE] 대각 뷰 저장: " << img_path << "\n";
    }

    std::string SavePLY(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        if (!std::filesystem::is_directory(template_dir_)) {
            std::filesystem::create_directories(template_dir_);
        }

        std::string file_path = template_dir_ + name_ + ".ply";

        // Check if file exists
        if (std::filesystem::exists(file_path)) {
            std::cout << "[WARN] 파일이 이미 존재합니다: " << file_path << "\n"
                      << "       덮어쓰시겠습니까? (y/N) > " << std::flush;
            std::string answer;
            std::getline(std::cin, answer);
            if (answer != "y" && answer != "Y") {
                std::cout << "[INFO] 저장을 취소합니다.\n";
                return "";
            }
        }

        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = true;

        if (pcl::io::savePLYFileBinary(file_path, *cloud) == -1) {
            std::cout << "[ERROR] PLY 파일 저장 실패: " << file_path << "\n";
            return "";
        }

        std::cout << "[SAVE] " << file_path << " (" << cloud->size()
                  << " points)\n";
        return file_path;
    }

    void UpdateConfig()
    {
        if (!std::filesystem::exists(config_path_)) {
            std::cout << "[WARN] Config 파일이 없습니다: " << config_path_
                      << "\n"
                      << "       pallet_detector.json에 수동으로 추가하세요.\n";
            PrintManualConfig();
            return;
        }

        try {
            // Read existing config
            std::ifstream in(config_path_);
            std::string content(
                (std::istreambuf_iterator<char>(in)),
                std::istreambuf_iterator<char>());
            in.close();

            // Strip comments (// style) for JSON parsing
            std::string clean;
            std::istringstream stream(content);
            std::string line;
            while (std::getline(stream, line)) {
                // Remove // comments (simple approach: not inside strings)
                auto pos = line.find("//");
                if (pos != std::string::npos) {
                    // Check if // is inside a string (count quotes before it)
                    int quotes = 0;
                    for (size_t i = 0; i < pos; i++) {
                        if (line[i] == '"') quotes++;
                    }
                    if (quotes % 2 == 0) {
                        line = line.substr(0, pos);
                    }
                }
                clean += line + "\n";
            }

            Poco::JSON::Parser parser;
            auto parsed = parser.parse(clean);
            auto root = parsed.extract<Poco::JSON::Object::Ptr>();

            auto tmpl_obj = root->getObject("template");
            if (!tmpl_obj) {
                std::cout
                    << "[WARN] Config에 'template' 섹션이 없습니다.\n";
                PrintManualConfig();
                return;
            }

            auto tmpl_lists = tmpl_obj->getArray("template_lists");
            if (!tmpl_lists) {
                tmpl_lists = new Poco::JSON::Array();
            }

            // Check if already exists
            std::string file_name = name_ + ".ply";
            for (size_t i = 0; i < tmpl_lists->size(); i++) {
                auto entry =
                    tmpl_lists->getObject(static_cast<unsigned int>(i));
                if (entry &&
                    entry->getValue<std::string>("file_name") == file_name) {
                    std::cout
                        << "[INFO] Config에 이미 '" << file_name
                        << "' 엔트리가 존재합니다. 업데이트합니다.\n";
                    entry->set("width", width_);
                    entry->set("height", height_);
                    WriteConfig(root);
                    return;
                }
            }

            // Add new entry
            Poco::JSON::Object::Ptr new_entry = new Poco::JSON::Object();
            new_entry->set("file_name", file_name);
            new_entry->set("width", width_);
            new_entry->set("height", height_);
            tmpl_lists->add(new_entry);
            tmpl_obj->set("template_lists", tmpl_lists);

            WriteConfig(root);
            std::cout << "[CONFIG] " << config_path_ << " 에 '" << file_name
                      << "' 추가 완료\n";
        }
        catch (const std::exception &e) {
            std::cout << "[WARN] Config 자동 업데이트 실패: " << e.what()
                      << "\n";
            PrintManualConfig();
        }
    }

    void WriteConfig(const Poco::JSON::Object::Ptr &root)
    {
        std::ofstream out(config_path_);
        root->stringify(out, 4);
        out.close();
    }

    void PrintManualConfig()
    {
        std::cout << "\n  수동 추가 방법: pallet_detector.json의 "
                     "template_lists에 다음을 추가하세요:\n"
                  << "  {\n"
                  << "      \"file_name\": \"" << name_ << ".ply\",\n"
                  << "      \"width\": " << width_ << ",\n"
                  << "      \"height\": " << height_ << "\n"
                  << "  }\n\n";
    }

    void PrintSummary(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const std::string &path)
    {
        // Compute bounding box
        float x_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        float y_min = x_min, y_max = x_max;
        float z_min = x_min, z_max = x_max;

        for (const auto &pt : cloud->points) {
            x_min = std::min(x_min, pt.x);
            x_max = std::max(x_max, pt.x);
            y_min = std::min(y_min, pt.y);
            y_max = std::max(y_max, pt.y);
            z_min = std::min(z_min, pt.z);
            z_max = std::max(z_max, pt.z);
        }

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        std::cout
            << "\n"
            << "========================================\n"
            << "  Template Capture Result\n"
            << "========================================\n"
            << "  File    : " << path << "\n"
            << "  Points  : " << cloud->size() << "\n"
            << "  중심    : (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ") [m]\n"
            << "  X range : " << x_min << " ~ " << x_max
            << " (" << (x_max - x_min) * 1000 << " mm)\n"
            << "  Y range : " << y_min << " ~ " << y_max
            << " (" << (y_max - y_min) * 1000 << " mm)\n"
            << "  Z range : " << z_min << " ~ " << z_max
            << " (" << (z_max - z_min) * 1000 << " mm)\n"
            << "  Width   : " << width_ << " m (config)\n"
            << "  Height  : " << height_ << " m (config)\n"
            << "========================================\n\n";
    }

    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_cloud_selected_;
    ros::Publisher pub_cloud_normalized_;
    ros::Publisher pub_cloud_base_link_;
    ros::Publisher pub_cloud_debug_range_;
    std::string last_frame_id_;
    std::string base_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string name_;
    double width_;
    double height_;
    std::string topic_;
    int target_frames_;
    double min_range_;
    double max_range_;
    int sor_k_;
    double sor_std_;
    std::string robot_name_;
    bool normalize_;
    double ransac_threshold_;
    bool ground_removal_;
    double ground_ransac_threshold_;
    int ground_removal_max_iter_;
    double front_face_depth_;      // [m] 전면부 유지 깊이
    int depth_axis_;               // 0=X, 1=Y, 2=Z (카메라 전방축, 정면=이 축 최소)
    double debug_range_min_;       // [m] 디버그용: 전방축 기준 이 거리 이상만 cloud_debug_range_min 토픽으로 발행
    bool debug_depth_forward_positive_;  // 전방 방향이 depth 축 양수면 true, 음수면 false
    double debug_lateral_half_;   // [m] 디버그용: 레터럴(카메라 X축) ±이 값만 유지 (기본 1 m)
    bool use_cluster_largest_;   // 가장 큰 클러스터만 유지
    double cluster_tolerance_;   // [m] 유클리드 클러스터 거리
    int min_cluster_size_;       // 최소 클러스터 포인트 수
    double save_roll_deg_;       // [deg] 저장 시 X축(roll) 회전 (기본 90)
    double save_pitch_deg_;      // [deg] 저장 시 Y축(pitch) 회전
    double save_yaw_deg_;        // [deg] 저장 시 Z축(yaw) 회전
    double save_voxel_leaf_;     // [m] 저장 전 voxel leaf size (기본 0.01)

    // Paths
    std::string template_dir_;
    std::string config_path_;

    // Capture state
    std::atomic<bool> capturing_;
    std::atomic<int> captured_frames_;
    std::atomic<bool> last_cloud_received_{false};
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    std::mutex cloud_mutex_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pallet_template_capture");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Start spinner for background callback processing
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PalletTemplateCapture capture(nh, pnh);
    capture.Run();

    spinner.stop();
    return 0;
}
