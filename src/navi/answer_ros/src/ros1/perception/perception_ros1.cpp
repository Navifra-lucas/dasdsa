#include "ros1/perception/perception_ros1.h"

#include "ros/package.h"
namespace ANSWER {
namespace PERCEPTION {

// 파일 상단 정의: 람다에서 사용 (선언 순서 때문에 namespace 内에 정의)
static Scan3D ApplyTypeOffsetToScan3D(
    const Scan3D &scan, double x_off, double y_off, double z_off,
    double deg_off_rad)
{
    Scan3D out;
    out.MutablePointCloud().reserve(scan.GetPointCloud().size());
    const double c = std::cos(deg_off_rad);
    const double s = std::sin(deg_off_rad);
    for (const auto &p : scan.GetPointCloud()) {
        double x = p.x() * c - p.y() * s + x_off;
        double y = p.x() * s + p.y() * c + y_off;
        double z = p.z() + z_off;
        out.MutablePointCloud().emplace_back(Eigen::Vector3d(x, y, z));
    }
    return out;
}
static void PublishScan3DToPointCloud2(
    const Scan3D &scan, ros::Publisher &pub,
    const std::string &frame_id)
{
    if (scan.GetPointCloud().empty()) {
        LOG_WARN("PublishScan3DToPointCloud2: skip (empty point cloud)");
        return;
    }
    if (!pub) {
        LOG_WARN("PublishScan3DToPointCloud2: skip (invalid publisher)");
        return;
    }
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(scan.GetPointCloud().size());
    msg.is_dense = true;
    msg.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(
        3, "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(scan.GetPointCloud().size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    for (const auto &pt : scan.GetPointCloud()) {
        *iter_x = static_cast<float>(pt.x());
        *iter_y = static_cast<float>(pt.y());
        *iter_z = static_cast<float>(pt.z());
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    pub.publish(msg);
}

PerceptionRos1::PerceptionRos1(const std::string &node_name)
{
    LOG_INFO("hello from PerceptionRos2 node");

    auto pkg_path = ros::package::getPath("answer");
    Path::GetInstance().SetAllPaths(
        pkg_path);  // set paths before loading parameters
    AnswerStatus::GetInstance().SetStatus(STATUS::PERCEPTION);
    Initialize();

    auto log_level = Configurator::GetInstance()
                         .GetParamValue("ros", "base", "log_level")
                         .convert<std::string>();
    logger_ = std::make_shared<Logger>(
        node_name, log_level,
        Configurator::GetInstance()
            .GetParamValue("ros", "base", "base_path")
            .convert<std::string>());

#ifdef BUILD_WITH_VISUALIZER
    auto is_visualizer = Configurator::GetInstance()
                             .GetParamValue("ros", "base", "visualizer")
                             .convert<bool>();

    if (is_visualizer == true) {
        Configurator::GetInstance().LoadParameters("localizer2d");
        Configurator::GetInstance().LoadParameters("slam2d");
        visualizer_ = std::make_unique<Visualizer>(logger_.get());
        LOG_INFO("Visualizer enabled");
    }
    else {
        LOG_INFO("Visualizer disabled by parameter");
    }
#endif
    GenerateSubscriptions();
    GeneratePublishers();
}
PerceptionRos1::~PerceptionRos1()
{
}

inline Pose3D Pose2DTo3D(const Pose2D &pose)
{
    auto T = Eigen::Vector3d(pose.x(), pose.y(), 0);
    Eigen::Vector3d axis(0, 0, 1);
    Sophus::SO3d R = Sophus::SO3d::exp(axis * pose.yaw());
    return Pose3D(R, T);
}

void PerceptionRos1::Initialize()
{
    LOG_INFO("Package path: {}", Path::GetInstance().GetConfigPath());
    // first load parameters
    Configurator::GetInstance().LoadParameters("ros");
    Configurator::GetInstance().LoadParameters("perception");
    Configurator::GetInstance().LoadParameters("pallet_detector");
    Configurator::GetInstance().LoadParameters("wingbody_detector");

    pallet_icp_matcher_ = std::make_unique<PalletICPMatcher>();
    wingbody_detector_ = std::make_unique<WingbodyDetector>();
    UpdateParameters();

    b_cmd_start_ = false;
    b_wingbody_cmd_start_ = false;
    b_local_ = false;

    // Load extrinsics
    auto extrinsic = Configurator::GetInstance()
                         .GetParamValue("ros", "base", "base_to_depth_camera_1")
                         .extract<Poco::JSON::Object::Ptr>();
    auto translation = extrinsic->getArray("translation");
    auto rotation = extrinsic->getArray("rotation");

    Eigen::VectorXd extrinsic_vec(6);
    extrinsic_vec.setZero();
    for (size_t i = 0; i < translation->size(); ++i) {
        extrinsic_vec(i) = translation->get(i).convert<double>();
    }
    for (size_t i = 0; i < rotation->size(); ++i) {
        extrinsic_vec(i + 3) = (rotation->get(i).convert<double>());
    }
    extrinsics_[KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1] = extrinsic_vec;

    // Load 3d_lidar extrinsics
    auto r_to_3dliar_extrinsic =
        Configurator::GetInstance()
            .GetParamValue("ros", "base", "base_to_3d_lidar")
            .extract<Poco::JSON::Object::Ptr>();
    auto r_to_3dlidar_translation =
        r_to_3dliar_extrinsic->getArray("translation");
    auto r_to_3dlidar_rotation = r_to_3dliar_extrinsic->getArray("rotation");

    Eigen::VectorXd r_to_3dlidar_extrinsic_vec(6);
    r_to_3dlidar_extrinsic_vec.setZero();
    for (size_t i = 0; i < r_to_3dlidar_translation->size(); ++i) {
        r_to_3dlidar_extrinsic_vec(i) =
            r_to_3dlidar_translation->get(i).convert<double>();
    }
    for (size_t i = 0; i < r_to_3dlidar_rotation->size(); ++i) {
        r_to_3dlidar_extrinsic_vec(i + 3) =
            (r_to_3dlidar_rotation->get(i).convert<double>());
    }
    extrinsics_[KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D] =
        r_to_3dlidar_extrinsic_vec;

    LOG_INFO("Extrinsic [depth_camera_1]: tx={:.4f} ty={:.4f} tz={:.4f} rx={:.4f} ry={:.4f} rz={:.4f}",
             extrinsic_vec(0), extrinsic_vec(1), extrinsic_vec(2),
             extrinsic_vec(3), extrinsic_vec(4), extrinsic_vec(5));
    LOG_INFO("Extrinsic [3d_lidar]: tx={:.4f} ty={:.4f} tz={:.4f} rx={:.4f} ry={:.4f} rz={:.4f}",
             r_to_3dlidar_extrinsic_vec(0), r_to_3dlidar_extrinsic_vec(1), r_to_3dlidar_extrinsic_vec(2),
             r_to_3dlidar_extrinsic_vec(3), r_to_3dlidar_extrinsic_vec(4), r_to_3dlidar_extrinsic_vec(5));

}  // namespace PERCEPTION

void PerceptionRos1::UpdateParameters()
{
    f_detect_pallet_end_condition_ =
        Configurator::GetInstance()
            .GetParamValue(
                "perception", "pallet_detect", "detect_end_condition")
            .convert<float>();

    b_publish_global_pose_ =
        Configurator::GetInstance()
            .GetParamValue("perception", "pallet_detect", "publish_global_pose")
            .convert<bool>();

    f_yaw_offset_ =
        Configurator::GetInstance()
            .GetParamValue("perception", "pallet_detect", "yaw_offset_rad")
            .convert<float>();

    Poco::JSON::Array::Ptr tmpl_lists =
        Configurator::GetInstance()
            .GetParamValue("pallet_detector", "template", "template_lists")
            .extract<Poco::JSON::Array::Ptr>();

    for (auto i = 0; i < tmpl_lists->size(); ++i) {
        auto tmpl = tmpl_lists->get(i).extract<Poco::JSON::Object::Ptr>();

        double width = tmpl->getValue<double>("width");
        double height = tmpl->getValue<double>("height");
        double x_offset = tmpl->has("x_offset") ? tmpl->getValue<double>("x_offset") : 0.0;
        double y_offset = tmpl->has("y_offset") ? tmpl->getValue<double>("y_offset") : 0.0;
        double z_offset = tmpl->has("z_offset") ? tmpl->getValue<double>("z_offset") : 0.0;
        double deg_offset = tmpl->has("deg_offset") ? tmpl->getValue<double>("deg_offset") : 0.0;
        double out_x =
            tmpl->has("output_pose_offset_x") ? tmpl->getValue<double>("output_pose_offset_x") : 0.0;
        double out_y =
            tmpl->has("output_pose_offset_y") ? tmpl->getValue<double>("output_pose_offset_y") : 0.0;
        double out_z =
            tmpl->has("output_pose_offset_z") ? tmpl->getValue<double>("output_pose_offset_z") : 0.0;
        double out_deg =
            tmpl->has("output_pose_offset_deg") ? tmpl->getValue<double>("output_pose_offset_deg") : 0.0;

        vec_pallet_width_.emplace_back(width);
        vec_pallet_height_.emplace_back(height);
        vec_pallet_x_offset_.emplace_back(x_offset);
        vec_pallet_y_offset_.emplace_back(y_offset);
        vec_pallet_z_offset_.emplace_back(z_offset);
        vec_pallet_deg_offset_.emplace_back(deg_offset);
        vec_output_pose_offset_x_.emplace_back(out_x);
        vec_output_pose_offset_y_.emplace_back(out_y);
        vec_output_pose_offset_z_.emplace_back(out_z);
        vec_output_pose_offset_deg_.emplace_back(out_deg);

        LOG_INFO("Template[{}]: w={:.2f} h={:.2f} x_off={:.3f} y_off={:.3f} z_off={:.3f} deg_off={:.3f} out_off=({:.3f},{:.3f},{:.3f},{:.1f}°)",
                 i, width, height, x_offset, y_offset, z_offset, deg_offset, out_x, out_y, out_z, out_deg);
    }

    Poco::JSON::Object::Ptr first_to_center =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "template", "first_to_center")
            .extract<Poco::JSON::Object::Ptr>();
    {
        double x = first_to_center->getValue<double>("x");
        double y = first_to_center->getValue<double>("y");
        double z = first_to_center->getValue<double>("z");
        double deg = first_to_center->getValue<double>("deg");

        T_first_to_center_ = Pose2DTo3D(Pose2D(x, y, deg * M_PI / 180.0));
    }

    Poco::JSON::Array::Ptr lack_positions =
        Configurator::GetInstance()
            .GetParamValue("wingbody_detector", "template", "lack_positions")
            .extract<Poco::JSON::Array::Ptr>();
    {
        for (auto i = 0; i < lack_positions->size(); ++i) {
            auto pos =
                lack_positions->get(i).extract<Poco::JSON::Object::Ptr>();
            double x = pos->getValue<double>("x");
            double y = pos->getValue<double>("y");
            double z = pos->getValue<double>("z");
            double deg = pos->getValue<double>("deg");

            Pose3D position = Pose2DTo3D(Pose2D(x, y, deg * M_PI / 180.0));
            vec_lack_positions_.emplace_back(position);
        }
    }
}

void PerceptionRos1::GenerateSubscriptions()
{
    command_subscriptions[KEY::SUBSCRIPTION::COMMAND::PERCEPTION_CMD] =
        std::make_shared<ros::Subscriber>(nh_.subscribe<std_msgs::String>(
            Configurator::GetInstance()
                .GetParamValue("perception", "subscription", "perception_cmd")
                .convert<std::string>(),
            10, [this](const std_msgs::String::ConstPtr msg) {
                std::string command = msg->data;
                LOG_INFO("Received perception command: {}", command);

                Poco::JSON::Parser parser;
                Poco::Dynamic::Var result;

                try {
                    result = parser.parse(command);
                    Poco::JSON::Object::Ptr json_object =
                        result.extract<Poco::JSON::Object::Ptr>();

                    if (json_object) {
                        if (json_object->has("cmd")) {
                            std::string command =
                                json_object->get("cmd").convert<std::string>();

                            float dist =
                                json_object->get("dist").convert<float>();
                            float height =
                                json_object->get("height").convert<float>();
                            float width =
                                json_object->get("width").convert<float>();
                            int type = json_object->get("type").convert<int>();

                            double x = json_object->get("x").convert<double>();
                            double y = json_object->get("y").convert<double>();
                            double z = json_object->get("z").convert<double>();
                            double deg =
                                json_object->get("deg").convert<double>();

                            b_local_ =
                                json_object->get("local").convert<bool>();

                            if (command == "start") {
                                {
                                    pallet_type_ = type;
                                    initial_guess_ = Eigen::Vector3d(
                                        x +
                                            vec_pallet_height_[pallet_type_] /
                                                2,
                                        y, z);

                                    b_cmd_start_ = true;
                                    b_init_odom_ = true;

                                    LOG_INFO("Pallet cmd start: type={}, x={:.3f}, y={:.3f}, z={:.3f}, deg={:.3f}, local={}",
                                             pallet_type_, x, y, z, deg, b_local_);
                                }
                            }

                            if (command == "stop") {
                                {
                                    b_cmd_start_ = false;
                                    b_init_odom_ = false;
                                }
                            }
                        }
                        else {
                            LOG_WARN("JSON object missing 'cmd' field.");
                        }
                    }
                }
                catch (Poco::Exception &ex) {
                    LOG_ERROR(
                        "Failed to parse perception command JSON: {}",
                        ex.displayText());
                    return;
                }
            }));

    command_subscriptions[KEY::SUBSCRIPTION::COMMAND::WINGBODY_CMD] =
        std::make_shared<ros::Subscriber>(nh_.subscribe<std_msgs::String>(
            Configurator::GetInstance()
                .GetParamValue("perception", "subscription", "wingbody_cmd")
                .convert<std::string>(),
            10, [this](const std_msgs::String::ConstPtr msg) {
                std::string command = msg->data;
                LOG_INFO("Received wingbody command: {}", command);

                Poco::JSON::Parser parser;
                Poco::Dynamic::Var result;

                try {
                    result = parser.parse(command);
                    Poco::JSON::Object::Ptr json_object =
                        result.extract<Poco::JSON::Object::Ptr>();

                    if (json_object) {
                        if (json_object->has("cmd")) {
                            std::string command =
                                json_object->get("cmd").convert<std::string>();

                            // Request First Rack position
                            double x = json_object->get("x").convert<double>();
                            double y = json_object->get("y").convert<double>();
                            double z = json_object->get("z").convert<double>();
                            double deg =
                                json_object->get("deg").convert<double>();

                            if (command == "start") {
                                Pose2D cur_robot_pose;
                                {
                                    std::lock_guard<std::mutex> lock(
                                        mtx_robot_pose_);
                                    cur_robot_pose = o_current_robot_pose_;
                                }

                                Pose3D T_map_robot = Pose2DTo3D(cur_robot_pose);

                                Eigen::Vector3d T(x, y, z);
                                Sophus::SO3d R = Sophus::SO3d::exp(
                                    Eigen::Vector3d(0, 0, 1) *
                                    (deg * M_PI / 180.0));
                                Pose3D T_map_first(R, T);

                                if (vec_lack_positions_.size() > 2) {
                                    Pose3D T_first_to_center =
                                        T_first_to_center_;
                                    Pose3D T_map_center =
                                        T_map_first * T_first_to_center;

                                    wingbody_initial_guess_ =
                                        T_map_robot.inverse() * T_map_center;

                                    b_wingbody_cmd_start_ = true;
                                    b_init_wingbody_odom_ = true;
                                }
                                else {
                                    LOG_ERROR(
                                        "vec_lack_positions_ size is insufficient!");
                                }
                            }

                            if (command == "stop") {
                                {
                                    b_wingbody_cmd_start_ = false;
                                    b_init_wingbody_odom_ = false;
                                }
                            }
                        }
                        else {
                            LOG_WARN("JSON object missing 'cmd' field.");
                        }
                    }
                }
                catch (Poco::Exception &ex) {
                    LOG_ERROR(
                        "Failed to parse perception command JSON: {}",
                        ex.displayText());
                    return;
                }
            }));

    sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::FORK_PLC_INFO] =
        std::make_shared<ros::Subscriber>(nh_.subscribe<std_msgs::Float32>(
            Configurator::GetInstance()
                .GetParamValue("perception", "subscription", "fork_lift")
                .convert<std::string>(),
            10, [this](const std_msgs::Float32::ConstPtr msg) {
                float f_fork_lift_level = msg->data;

                LOG_DEBUG("fork lift level msg : {}", msg->data);
                LOG_INFO("Fork lift Z-offset: level={:.4f}", f_fork_lift_level);

                Eigen::VectorXd extrinsic_vec(6);
                extrinsic_vec.setZero();
                extrinsic_vec(2) = static_cast<double>(f_fork_lift_level);

                extrinsics_[KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1] +=
                    extrinsic_vec;
            }));
    {
        sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1] =
            std::make_shared<
                ros::Subscriber>(nh_.subscribe<sensor_msgs::PointCloud2>(
                Configurator::GetInstance()
                    .GetParamValue(
                        "perception", "subscription", "depth_camera_1")
                    .convert<std::string>(),
                10, [this](const sensor_msgs::PointCloud2::ConstPtr msg) {
                    Scan3D scan;
                    scan.MutablePointCloud().reserve(msg->height * msg->width);
                    sensor_msgs::PointCloud2ConstIterator<float> msg_x(
                        *msg, "x");
                    sensor_msgs::PointCloud2ConstIterator<float> msg_y(
                        *msg, "y");
                    sensor_msgs::PointCloud2ConstIterator<float> msg_z(
                        *msg, "z");
                    Eigen::Matrix3d rotation_matrix;
                    rotation_matrix << 0, 0, 1, 1, 0, 0, 0, 1, 0;

                    for (size_t i = 0; i < msg->height * msg->width;
                         ++i, ++msg_x, ++msg_y, ++msg_z) {
                        Eigen::Vector3d point_in_camera_frame(
                            *msg_x, *msg_y, *msg_z);
                        // Skip invalid points
                        if (std::isfinite(point_in_camera_frame(0)) &&
                            std::isfinite(point_in_camera_frame(1)) &&
                            std::isfinite(point_in_camera_frame(2))) {
                            Eigen::Vector3d corrected_point =
                                rotation_matrix * point_in_camera_frame;
                            scan.MutablePointCloud().emplace_back(
                                corrected_point);
                        }
                    }
                    if (extrinsics_.find(
                            KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1) !=
                        extrinsics_.end()) {
                        scan.SetExtrinsicParameter(
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1](0),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1](1),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1](2),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1](3),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1](4),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::DEPTH_CAMERA_1](5));
                        scan.TransformToRobotFrame();
                    }
                    LOG_DEBUG("Depth camera: {} points", scan.GetPointCloud().size());
                    if (b_cmd_start_) {
                        // robot to pallet
                        Pose3D delta = Pose3D();
                        if (b_init_odom_) {
                            b_init_odom_ = false;
                            o_prev_odom_ = SLAM2D::WheelOdometry::GetInstance()
                                               .GetOdomPose();
                            odom_add_guess_ = Pose3D();
                            odom_add_guess_.translation() = initial_guess_;
                            Pose3D delta_tr = Pose3D();
                            pallet_icp_matcher_->SetDeltaTransform(delta_tr);
                            pallet_icp_matcher_->SetICPResultPos(
                                odom_add_guess_);
                            LOG_INFO(
                                "initial guess {}", initial_guess_.transpose());
                            // pallet_icp_matcher_->SetCorrespondenceDistance(0.5);
                        }
                        // odom1 to odom2
                        auto prev_to_current = o_prev_odom_.inv() *
                            SLAM2D::WheelOdometry::GetInstance().GetOdomPose();
                        auto T = Eigen::Vector3d(
                            prev_to_current.x(), prev_to_current.y(), 0);
                        Eigen::Vector3d axis(0, 0, 1);
                        Sophus::SO3d R =
                            Sophus::SO3d::exp(axis * prev_to_current.yaw());
                        delta = Pose3D(R, T);

                        LOG_DEBUG("Odom delta: x={:.4f} y={:.4f} yaw={:.4f}",
                                  prev_to_current.x(), prev_to_current.y(), prev_to_current.yaw());

                        auto delta_tr =
                            pallet_icp_matcher_->GetDeltaTransform();

                        odom_add_guess_ = odom_add_guess_ * delta;
                        o_prev_odom_ =
                            SLAM2D::WheelOdometry::GetInstance().GetOdomPose();

                        LOG_DEBUG("odom_add_guess: ({:.3f},{:.3f},{:.3f})",
                                  odom_add_guess_.translation().x(),
                                  odom_add_guess_.translation().y(),
                                  odom_add_guess_.translation().z());

                        // 디버그: 센서 원본 매 프레임 발행 (후처리 전)
                        PublishScan3DToPointCloud2(
                            scan, pub_debug_sensor_raw_, debug_frame_id_);
                        Scan3D initial_tmpl = pallet_icp_matcher_->TransformTemplate(
                            pallet_type_, odom_add_guess_.matrix());
                        if (pallet_type_ >= 0 &&
                            pallet_type_ < static_cast<int>(vec_pallet_x_offset_.size())) {
                            initial_tmpl = ApplyTypeOffsetToScan3D(
                                initial_tmpl,
                                vec_pallet_x_offset_[pallet_type_],
                                vec_pallet_y_offset_[pallet_type_],
                                vec_pallet_z_offset_[pallet_type_],
                                vec_pallet_deg_offset_[pallet_type_] * M_PI / 180.0);
                        }
                        // template_initial만 매 프레임 발행. template_aligned는 ICP 완료 시 VisualizerCallback에서만 발행
                        PublishScan3DToPointCloud2(
                            initial_tmpl, pub_debug_template_initial_, debug_frame_id_);

                        // auto transformed = odom_add_guess_ * delta_tr;

                        double detect_norm = pallet_icp_matcher_->GetICPResultPos()
                                .translation()
                                .head<2>()
                                .norm();
                        if (detect_norm > f_detect_pallet_end_condition_) {
                            LOG_DEBUG("detect_end_condition PASS: norm={:.4f} > threshold={:.4f}",
                                      detect_norm, f_detect_pallet_end_condition_);
                            Pose2D cur_robot_pose;
                            {
                                std::lock_guard<std::mutex> lock(
                                    mtx_robot_pose_);

                                cur_robot_pose = o_current_robot_pose_;
                            }
                            Pose3D robot_pose;

                            auto robot_T = Eigen::Vector3d(
                                cur_robot_pose.x(), cur_robot_pose.y(), 0);
                            Sophus::SO3d robot_R =
                                Sophus::SO3d::exp(axis * cur_robot_pose.yaw());
                            robot_pose = Pose3D(robot_R, robot_T);
                            pallet_icp_matcher_->DetectPalletsThread(
                                scan, pallet_type_, odom_add_guess_, delta,
                                robot_pose, msg->header.stamp.sec);
                        }
                        else {
                            LOG_DEBUG("detect_end_condition FAIL: norm={:.4f} <= threshold={:.4f}",
                                      detect_norm, f_detect_pallet_end_condition_);
                        }
                    }
                }));
    }
    {
        sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D] =
            std::make_shared<
                ros::Subscriber>(nh_.subscribe<sensor_msgs::PointCloud2>(
                Configurator::GetInstance()
                    .GetParamValue("perception", "subscription", "lidar1")
                    .convert<std::string>(),
                10, [this](const sensor_msgs::PointCloud2::ConstPtr msg) {
                    Scan3D scan;
                    scan.MutablePointCloud().reserve(msg->height * msg->width);
                    sensor_msgs::PointCloud2ConstIterator<float> msg_x(
                        *msg, "x");
                    sensor_msgs::PointCloud2ConstIterator<float> msg_y(
                        *msg, "y");
                    sensor_msgs::PointCloud2ConstIterator<float> msg_z(
                        *msg, "z");

                    for (size_t i = 0; i < msg->height * msg->width;
                         ++i, ++msg_x, ++msg_y, ++msg_z) {
                        Eigen::Vector3d point_in_lidar_frame(
                            *msg_x, *msg_y, *msg_z);
                        // Skip invalid points
                        if (std::isfinite(point_in_lidar_frame(0)) &&
                            std::isfinite(point_in_lidar_frame(1)) &&
                            std::isfinite(point_in_lidar_frame(2))) {
                            scan.MutablePointCloud().emplace_back(
                                point_in_lidar_frame);
                        }
                    }
                    if (extrinsics_.find(
                            KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D) !=
                        extrinsics_.end()) {
                        scan.SetExtrinsicParameter(
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D](0),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D](1),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D](2),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D](3),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D](4),
                            extrinsics_
                                [KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D](5));
                        scan.TransformToRobotFrame();
                    }
                    if (b_wingbody_cmd_start_) {
                        Pose3D delta = Pose3D();
                        if (b_init_wingbody_odom_) {
                            b_init_wingbody_odom_ = false;
                            o_wingbody_prev_odom_ =
                                SLAM2D::WheelOdometry::GetInstance()
                                    .GetOdomPose();

                            wingbody_odom_add_guess_ = wingbody_initial_guess_;

                            Pose3D delta_tr = Pose3D();
                            wingbody_detector_->SetGlobalConfidence(0.0);
                            wingbody_detector_->SetDeltaTransform(delta_tr);
                            wingbody_detector_->SetICPResultPos(
                                wingbody_odom_add_guess_);
                        }

                        auto prev_to_current = o_wingbody_prev_odom_.inv() *
                            SLAM2D::WheelOdometry::GetInstance().GetOdomPose();

                        delta = Pose2DTo3D(prev_to_current);

                        // auto delta_tr =
                        // wingbody_detector_->GetDeltaTransform();
                        wingbody_odom_add_guess_ =
                            wingbody_odom_add_guess_ * delta;

                        o_wingbody_prev_odom_ =
                            SLAM2D::WheelOdometry::GetInstance().GetOdomPose();
                        {
                            Pose2D cur_robot_pose;
                            {
                                std::lock_guard<std::mutex> lock(
                                    mtx_robot_pose_);
                                cur_robot_pose = o_current_robot_pose_;
                            }

                            Pose3D robot_pose = Pose2DTo3D(cur_robot_pose);
                            wingbody_detector_->DetectWingbodyThread(
                                scan, wingbody_odom_add_guess_, delta,
                                robot_pose, msg->header.stamp.sec);
                        }
                    }
                }));
    }
    {
        sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::ODOM] =
            std::make_shared<ros::Subscriber>(nh_.subscribe<nav_msgs::Odometry>(
                "odom", 10, [this](const nav_msgs::Odometry::ConstPtr msg) {
                    Pose2D current_odom_pose = ConvertROSToOdom2D<
                        nav_msgs::Odometry::ConstPtr, ANSWER::Pose2D>(msg);
                    SLAM2D::WheelOdometry::GetInstance().SetOdomPose(
                        current_odom_pose);
                    // LOG_INFO("here");
                }));
    }
    {
        sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::ROBOT_POSE] =
            std::make_shared<
                ros::Subscriber>(nh_.subscribe<
                                 geometry_msgs::PoseWithCovarianceStamped>(
                "localization/robot_pos", 10,
                [this](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr
                           msg) {
                    {
                        std::lock_guard<std::mutex> lock(mtx_robot_pose_);

                        o_current_robot_pose_ = ConvertROSToOdom2D<
                            geometry_msgs::PoseWithCovarianceStamped::ConstPtr,
                            ANSWER::Pose2D>(msg);
                    }
                }));
    }
}

void PerceptionRos1::GeneratePublishers()
{
    // 디버그: RViz에서 템플릿/센서 포인트 클라우드 확인 (latched)
    pub_debug_template_initial_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            "/perception/debug/template_initial", 1, true);
    pub_debug_template_aligned_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            "/perception/debug/template_aligned", 1, true);
    pub_debug_sensor_raw_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            "/perception/debug/sensor_raw", 1, true);
    pub_debug_sensor_matched_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            "/perception/debug/sensor_matched", 1, true);

    LOG_INFO("Debug topics: template_initial, template_aligned, sensor_raw, sensor_matched");

    VisualizerCallback::point_cloud_3d_callback.add(
        [this](const Scan3D &point_cloud,
               const Eigen::Isometry3d & /*reference*/,
               const std::string &frame_name,
               const std::string & /*color_code*/,
               float /*scale*/) {
            // 후처리 끝난 센서 = ICP 매칭에 사용된 포인트클라우드 (ICP 완료 시마다 1회 발행)
            if (frame_name == "sensor_data") {
                PublishScan3DToPointCloud2(
                    point_cloud, pub_debug_sensor_matched_, debug_frame_id_);
                LOG_DEBUG("[debug] published sensor_matched (ICP completion)");
                return;
            }
            // template_aligned: ICP 매칭 결과 그대로 발행 (offset 미적용)
            // initial_guess는 매 프레임 콜백에서 이미 발행되므로 여기서는 스킵
            if (frame_name == "aligned_pallet") {
                PublishScan3DToPointCloud2(
                    point_cloud, pub_debug_template_aligned_, debug_frame_id_);
                LOG_INFO(
                    "[debug] published template_aligned (ICP completion, "
                    "pts={})",
                    static_cast<int>(point_cloud.GetPointCloud().size()));
            }
        });

    publishers_[KEY::PUBLICATION::PERCEPTION::PALLET_POSE] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<geometry_msgs::PoseStamped>(TOPICS::PALLET_POSE, 1));
    PerceptionResultCallback::pallet_pose_callback.add(
        [&](const Pose3D &pose, const Pose3D &robot_pose,
            const long int &stamp) {
            Pose3D robot_to_sensor;
            auto rs_T = Eigen::Vector3d(-0.363, 0.09, 0);
            Eigen::Vector3d axis(0, 0, 1);

            Sophus::SO3d rs_R =
                Sophus::SO3d::exp(axis * (M_PI + f_yaw_offset_));

            robot_to_sensor = Pose3D(rs_R, rs_T);

            Pose3D center_to_end = Pose3D();
            center_to_end.translation() =
                Eigen::Vector3d(-vec_pallet_height_[pallet_type_] / 2, 0, 0);

            Pose3D type_offset = Pose3D();
            if (pallet_type_ >= 0 &&
                pallet_type_ < static_cast<int>(vec_pallet_x_offset_.size())) {
                double x_off = vec_pallet_x_offset_[pallet_type_];
                double y_off = vec_pallet_y_offset_[pallet_type_];
                double z_off = vec_pallet_z_offset_[pallet_type_];
                double rad_off = vec_pallet_deg_offset_[pallet_type_] * M_PI / 180.0;
                type_offset.translation() = Eigen::Vector3d(x_off, y_off, z_off);
                type_offset.so3() = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, 1) * rad_off);
                if (x_off != 0.0 || y_off != 0.0 || z_off != 0.0 || rad_off != 0.0) {
                    LOG_INFO("Applying type_offset: x={:.3f} y={:.3f} z={:.3f} deg={:.3f}",
                             x_off, y_off, z_off, vec_pallet_deg_offset_[pallet_type_]);
                }
            }

            geometry_msgs::PoseStamped res;
            auto sensor_to_end = pose * center_to_end * type_offset;

            // 팔레트별 결과 pose 보정 (템플릿 offset과 별개, /perception/pose 발행값만 보정)
            if (pallet_type_ >= 0 &&
                pallet_type_ < static_cast<int>(vec_output_pose_offset_x_.size())) {
                double ox = vec_output_pose_offset_x_[pallet_type_];
                double oy = vec_output_pose_offset_y_[pallet_type_];
                double oz = vec_output_pose_offset_z_[pallet_type_];
                double odeg = vec_output_pose_offset_deg_[pallet_type_];
                if (ox != 0.0 || oy != 0.0 || oz != 0.0 || odeg != 0.0) {
                    Pose3D output_offset = Pose3D();
                    output_offset.translation() = Eigen::Vector3d(ox, oy, oz);
                    output_offset.so3() = Sophus::SO3d::exp(
                        Eigen::Vector3d(0, 0, 1) * (odeg * M_PI / 180.0));
                    sensor_to_end = sensor_to_end * output_offset;
                }
            }

            auto map_to_end = robot_pose * robot_to_sensor * sensor_to_end;

            LOG_INFO(
                "center to end  : {}", center_to_end.translation().transpose());
            LOG_INFO("out pose {}", pose.translation().transpose());
            LOG_INFO(
                "robot to end pose {}",
                sensor_to_end.translation().transpose());

            Eigen::Quaterniond sensor_to_end_q =
                sensor_to_end.so3().unit_quaternion();
            Eigen::Vector3d sensor_to_end_euler =
                sensor_to_end_q.toRotationMatrix().eulerAngles(2, 1, 0);

            double yaw = sensor_to_end_euler[0];
            LOG_INFO("sensor to end yaw {:.2f} degrees", yaw * 180.0 / M_PI);

            LOG_INFO(
                "map to end pose {}", map_to_end.translation().transpose());

            static long int count = 0;

            if (b_local_) {
                Eigen::Quaterniond q = sensor_to_end.so3().unit_quaternion();

                res.pose.position.x = sensor_to_end.translation().x();
                res.pose.position.y = sensor_to_end.translation().y();
                res.pose.position.z = sensor_to_end.translation().z();
                res.pose.orientation.x = q.x();
                res.pose.orientation.y = q.y();
                res.pose.orientation.z = q.z();
                res.pose.orientation.w = q.w();
            }
            else {
                Eigen::Quaterniond q = map_to_end.so3().unit_quaternion();

                res.pose.position.x = map_to_end.translation().x();
                res.pose.position.y = map_to_end.translation().y();
                res.pose.position.z = map_to_end.translation().z();
                res.pose.orientation.x = q.x();
                res.pose.orientation.y = q.y();
                res.pose.orientation.z = q.z();
                res.pose.orientation.w = q.w();
            }
            if (count++ > 3) {
                publishers_[KEY::PUBLICATION::PERCEPTION::PALLET_POSE]->publish(
                    res);
            }
        });

    publishers_[KEY::PUBLICATION::PERCEPTION::WINGBODY_DETECTION] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<geometry_msgs::PoseArray>(
                TOPICS::WINGBODY_DETECTION, 1));
    PerceptionResultCallback::wingbody_pose_callback.add([&](const Pose3D &pose,
                                                             const Pose3D
                                                                 &robot_pose,
                                                             const long int
                                                                 &stamp) {
        LOG_INFO("wingbody detection callback invoked.");
        constexpr double RAD2DEG = 180.0 / M_PI;
        constexpr double DEG2RAD = M_PI / 180.0;

        auto log_pose_rpy = [](const Sophus::SE3d &T) {
            Eigen::Vector3d euler_zyx = T.so3().matrix().eulerAngles(2, 1, 0);

            double yaw = euler_zyx[0] * RAD2DEG;  // Z
            double pitch = euler_zyx[1] * RAD2DEG;  // Y
            double roll = euler_zyx[2] * RAD2DEG;  // X

            LOG_INFO(
                "xyz=({:.3f},{:.3f},{:.3f}) yaw={:.2f}, pitch={:.2f}, roll={:.2f}",
                T.translation().x(), T.translation().y(), T.translation().z(),
                yaw, pitch, roll);
        };

        LOG_INFO(
            "wingbody initial guess : {}",
            wingbody_initial_guess_.translation().transpose());

        log_pose_rpy(wingbody_initial_guess_);

        LOG_INFO(
            "wingbody odom guess : {}",
            wingbody_odom_add_guess_.translation().transpose());

        log_pose_rpy(wingbody_odom_add_guess_);

        Pose3D updated_wingbody_pose = pose;
        Pose3D wingbody_delta = wingbody_initial_guess_.inverse() * pose;

        LOG_INFO(
            "wingbody delta : {}", wingbody_delta.translation().transpose());

        Pose3D T_first_center = T_first_to_center_;
        Pose3D T_center_first = T_first_center.inverse();

        Pose2D cur_robot_pose;
        {
            std::lock_guard<std::mutex> lock(mtx_robot_pose_);
            cur_robot_pose = o_current_robot_pose_;
        }

        Pose3D T_map_robot = Pose2DTo3D(cur_robot_pose);
        geometry_msgs::PoseArray ros_wingbody_poses;
        ros_wingbody_poses.header.stamp = ros::Time::now();
        ros_wingbody_poses.header.frame_id = "map";
        Pose3D T_map_center = T_map_robot * pose;

        log_pose_rpy(T_map_center);

        for (size_t i = 0; i < vec_lack_positions_.size(); ++i) {
            // Pose3D T_first_rack_i = vec_lack_positions_[i];
            // Pose3D T_center_rack_i = T_center_first * T_first_rack_i;

            Pose3D T_center_rack_i = vec_lack_positions_[i];
            Pose3D T_map_rack_i = T_map_center * T_center_rack_i;

            LOG_INFO("rack[{}]", i);
            log_pose_rpy(T_map_rack_i);

            geometry_msgs::Pose geometry_pose;
            Eigen::Quaterniond q = T_map_rack_i.so3().unit_quaternion();
            geometry_pose.position.x = T_map_rack_i.translation().x();
            geometry_pose.position.y = T_map_rack_i.translation().y();
            geometry_pose.position.z = T_map_rack_i.translation().z();

            geometry_pose.orientation.x = q.x();
            geometry_pose.orientation.y = q.y();
            geometry_pose.orientation.z = q.z();
            geometry_pose.orientation.w = q.w();

            ros_wingbody_poses.poses.push_back(geometry_pose);
        }

        publishers_[KEY::PUBLICATION::PERCEPTION::WINGBODY_DETECTION]->publish(
            ros_wingbody_poses);
    });
}

void PerceptionRos1::TerminateNode()
{
}
}  // namespace PERCEPTION
}  // namespace ANSWER