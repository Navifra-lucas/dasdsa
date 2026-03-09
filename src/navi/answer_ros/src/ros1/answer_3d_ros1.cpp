#include "ros1/answer_3d_ros1.h"

#include "ros/package.h"
namespace ANSWER {

Answer3DRos1 ::Answer3DRos1(const std::string &node_name)
{
    auto pkg_path = ros::package::getPath("answer");
    Path::GetInstance().SetAllPaths(
        pkg_path);  // set paths before loading parameters
    std::string ascii_art = R"(
         _    _   _ ______        _______ ____    _____ ____       
__/\__  / \  | \ | / ___\ \      / / ____|  _ \  |___ /|  _ \__/\__
\    / / _ \ |  \| \___ \\ \ /\ / /|  _| | |_) |   |_ \| | | \    /
/_  _\/ ___ \| |\  |___) |\ V  V / | |___|  _ <   ___) | |_| /_  _\
  \/ /_/   \_\_| \_|____/  \_/\_/  |_____|_| \_\ |____/|____/  \/  
)";
    LOG_INFO("{}", ascii_art);
    LOG_INFO(
        "ANSWER Git Hash: {}, Build Date: {}", APP_GIT_HASH, APP_BUILD_DATE);

    Initialize();

    auto log_level = Configurator::GetInstance()
                         .GetParamValue("ros", "base", "log_level")
                         .convert<std::string>();
    logger_ = std::make_shared<Logger>(
        node_name, log_level,
        Configurator::GetInstance()
            .GetParamValue("ros", "base", "base_path")
            .convert<std::string>());

    auto is_visualizer = Configurator::GetInstance()
                             .GetParamValue("ros", "base", "visualizer")
                             .convert<bool>();

    if (is_visualizer == true) {
        visualizer_ = std::make_unique<Visualizer>(logger_.get());
        LOG_INFO("Visualizer enabled");
    }
    else {
        LOG_INFO("Visualizer disabled by parameter");
    }

    GenerateSubscriptions();
    GeneratePublishers();
    GenerateServices();
    RegisterCallbacks();
}

Answer3DRos1 ::~Answer3DRos1()
{
}
void Answer3DRos1 ::TerminateNode()
{
    for (auto &sub : sensor_subscriptions) {
        sub.second->shutdown();
    }
    for (auto &sub : command_subscriptions) {
        sub.second->shutdown();
    }
    for (auto &pub : publishers) {
        pub.second->shutdown();
    }
    // AnswerStatus::GetInstance().TerminateAlarmCheckThread();
    logger_->TerminateLogger();
    logger_.reset();
}
void Answer3DRos1::Initialize()
{
    Configurator::GetInstance().LoadParameters("ros");

    // set extrinsics
    auto extrinsic = Configurator::GetInstance()
                         .GetParamValue("ros", "base", "base_to_3d_lidar")
                         .extract<Poco::JSON::Object::Ptr>();

    auto translation = extrinsic->getArray("translation");
    auto rotation = extrinsic->getArray("rotation");
    Eigen::VectorXd extrinsic_vec(6);
    extrinsic_vec.setZero();
    for (size_t i = 0; i < translation->size(); ++i) {
        extrinsic_vec(i) = translation->get(i).convert<double>();
    }
    for (size_t i = 0; i < rotation->size(); ++i) {
        extrinsic_vec(i + 3) =
            (rotation->get(i).convert<double>()) * (M_PI / 180.);
    }
    extrinsics_[KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D] = extrinsic_vec;

    extrinsic = Configurator::GetInstance()
                    .GetParamValue("ros", "base", "base_to_imu")
                    .extract<Poco::JSON::Object::Ptr>();
    translation = extrinsic->getArray("translation");
    rotation = extrinsic->getArray("rotation");
    extrinsic_vec.setZero();
    for (size_t i = 0; i < translation->size(); ++i) {
        extrinsic_vec(i) = translation->get(i).convert<double>();
    }
    for (size_t i = 0; i < rotation->size(); ++i) {
        extrinsic_vec(i + 3) =
            (rotation->get(i).convert<double>()) * (M_PI / 180.);
    }
    extrinsics_[KEY::SUBSCRIPTION::SENSOR::IMU] = extrinsic_vec;

    slam_3d_.reset();
    slam_3d_ = std::make_unique<SLAM3D::SLAM>();
}
void Answer3DRos1::GenerateSubscriptions()
{
    std::vector<std::string> point_cloud_topics;
    auto pcd_topics = Configurator::GetInstance()
                          .GetParamValue("ros", "topic", "point_cloud_3d_topic")
                          .extract<Poco::JSON::Array::Ptr>();
    for (size_t i = 0; i < pcd_topics->size(); ++i) {
        auto topic = pcd_topics->get(i).convert<std::string>();
        point_cloud_topics.push_back(topic);
    }

    {
        using Sensor = KEY::SUBSCRIPTION::SENSOR;
        using Under = std::underlying_type_t<Sensor>;
        for (int i = 0; i < point_cloud_topics.size(); ++i) {
            const auto key = static_cast<Sensor>(
                static_cast<Under>(Sensor::POINT_CLOUD_3D) + i);
            sensor_subscriptions[key] = std::make_shared<ros::Subscriber>(
                nh_.subscribe<sensor_msgs::PointCloud2>(
                    point_cloud_topics[i], 10,
                    [this, key](const sensor_msgs::PointCloud2::ConstPtr msg) {
                        Scan3D scan = ConvertROSToScan3D<
                            sensor_msgs::PointCloud2::ConstPtr,
                            sensor_msgs::PointCloud2ConstIterator<double>,
                            sensor_msgs::PointCloud2ConstIterator<float>,
                            sensor_msgs::PointCloud2ConstIterator<uint32_t>,
                            sensor_msgs::PointField, Scan3D>(msg);

                        if (extrinsics_.find(key) != extrinsics_.end()) {
                            scan.SetExtrinsicParameter(
                                extrinsics_[key](0), extrinsics_[key](1),
                                extrinsics_[key](2), extrinsics_[key](3),
                                extrinsics_[key](4), extrinsics_[key](5));
                            scan.TransformToRobotFrame();
                        }
                        slam_3d_->PushLiDARData(scan);
                    }));
        }
    }
    {
        sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::IMU] =
            std::make_shared<ros::Subscriber>(nh_.subscribe<sensor_msgs::Imu>(
                Configurator::GetInstance()
                    .GetParamValue("ros", "topic", "imu_topic")
                    .convert<std::string>(),
                100, [this](const sensor_msgs::Imu::ConstPtr msg) {
                    // IMU callback implementation
                    slam_3d_->PushIMUData(
                        ConvertROSToIMUData<
                            sensor_msgs::Imu::ConstPtr, IMUData>(msg));
                    // LOG_INFO("IMU data received");
                }));
    }
}
void Answer3DRos1::GenerateServices()
{
}
void Answer3DRos1::GeneratePublishers()
{
}
void Answer3DRos1::RegisterCallbacks()
{
}
}  // namespace ANSWER