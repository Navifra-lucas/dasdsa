#include "ros1/answer_ros1.h"

#include "2d_localizer/version.hpp"
#include "ros/package.h"

namespace ANSWER {
AnswerRos1::AnswerRos1(const std::string &node_name)
    : slam_(nullptr)
{
    auto pkg_path = ros::package::getPath("answer");
    Path::GetInstance().SetAllPaths(
        pkg_path);  // set paths before loading parameters
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
        visualizer_ = std::make_unique<Visualizer>(logger_.get());
        LOG_INFO("Visualizer enabled");
    }
    else {
        LOG_INFO("Visualizer disabled by parameter");
    }

#endif
    std::string ascii_art = R"(
               _    _   _ ______        _______ ____             
__/\____/\__  / \  | \ | / ___\ \      / / ____|  _ \__/\____/\__
\    /\    / / _ \ |  \| \___ \\ \ /\ / /|  _| | |_) \    /\    /
/_  _\/_  _\/ ___ \| |\  |___) |\ V  V / | |___|  _ </_  _\/_  _\
  \/    \/ /_/   \_\_| \_|____/  \_/\_/  |_____|_| \_\ \/    \/  
)";
    LOG_INFO("{}", ascii_art);
    LOG_INFO(
        "ANSWER Git Hash: {}, Build Date: {}", APP_GIT_HASH, APP_BUILD_DATE);
    AnswerStatus::GetInstance().StartAlarmCheckThread();

    GenerateSubscriptions();
    GeneratePublishers();
    GenerateServices();
    RegisterCallbacks();
}

AnswerRos1::~AnswerRos1()
{
    std::cout << "ROS1 Destructor called" << std::endl;
}
void AnswerRos1::TerminateNode()
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
    for (auto &srv : control_srv_servers_) {
        srv.second.shutdown();
    }
    AnswerStatus::GetInstance().TerminateAlarmCheckThread();
    visualizer_.reset();
    slam_.reset();
    localizer_.reset();
    logger_->TerminateLogger();
    logger_.reset();

    LOG_INFO("Terminate ROS1 Node");
}

void AnswerRos1::RegisterCallbacks()
{
    // related to localization
    {
        LocalizeResultCallback::log_callback.add(
            [this](const Poco::JSON::Object::Ptr log) {
                std::ostringstream oss;
                log->stringify(oss);
                LOG_DEBUG("{}", oss.str());
                std_msgs::String report_msg;
                report_msg.data = oss.str();
                publishers[KEY::PUBLICATION::SLAM::REPORT]->publish(report_msg);
            });
    }
    {
        LocalizeResultCallback::synced_result_callback.add(
            [&](Pose2D pose, LOCALIZATION2D::LocalizeResult result) {
                geometry_msgs::PoseWithCovarianceStamped pose_msg =
                    ConvertPose2DToROS<
                        geometry_msgs::PoseWithCovarianceStamped>(pose);

                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pose.x(), pose.y(), 0.0));
                tf::Quaternion q;
                q.setRPY(0, 0, pose.yaw());
                transform.setRotation(q);
                br_->sendTransform(tf::StampedTransform(
                    transform, ros::Time::now(), "map", "base_link"));
                publishers[KEY::PUBLICATION::SLAM::POSE]->publish(pose_msg);
            });
    }
    {
        LogCallback::log_callback.add([this](const std::string &log) {
            std_msgs::String status_msg;
            status_msg.data = log;
            publishers[KEY::PUBLICATION::SLAM::STATUS]->publish(status_msg);
        });
    }
    {
        LocalizeResultCallback::reflectors_callback.add(
            [this](const Poco::JSON::Object::Ptr reflectors) {
                if (reflectors->has("reflectors") == false) {
                    LOG_WARN("Reflector map is empty");
                    return;
                }
                std_msgs::String reflector_msg;

                auto string_data =
                    ANSWER::answer_utils::ConvertJSONToString(reflectors);
                reflector_msg.data = std::move(string_data);

                publishers[KEY::PUBLICATION::SLAM::DETECTED_REFLECTORS]
                    ->publish(reflector_msg);
            });
    }
    // for slam
    {
        VisualizerCallback::scan_pose_callback.add(
            [this](
                const Scan2D &scan, const ANSWER::Pose2D &pose,
                const std::string &frame_id, const std::string &color,
                const float scale) {
                if (AnswerStatus::GetInstance().GetStatus() !=
                    ANSWER::STATUS::SLAM) {
                    return;
                }
                geometry_msgs::PoseWithCovarianceStamped pose_msg =
                    ConvertPose2DToROS<
                        geometry_msgs::PoseWithCovarianceStamped>(pose);
                publishers[KEY::PUBLICATION::SLAM::POSE]->publish(pose_msg);
            });
    }
    // {
    //     SLAMResultCallback::pose_callback.add([this](
    //                                               const ANSWER::Pose2D &pose)
    //                                               {
    //         geometry_msgs::PoseWithCovarianceStamped pose_msg =
    //             ConvertPose2DToROS<geometry_msgs::PoseWithCovarianceStamped>(
    //                 pose);
    //         publishers[KEY::PUBLICATION::SLAM::POSE]->publish(pose_msg);
    //     });
    // }
    {
        SLAMResultCallback::pose_graph_callback.add(
            [this](const Poco::JSON::Object::Ptr pose_graph) {
                auto string_data =
                    ANSWER::answer_utils::ConvertJSONToString(pose_graph);
                std_msgs::String report_msg;
                report_msg.data = string_data;
                publishers[KEY::PUBLICATION::SLAM::POSE_GRAPH]->publish(
                    report_msg);
            });
    }
    {
        SLAMResultCallback::progress_callback.add(
            [this](const int &mapping_progress) {
                std_msgs::Int16 progress_msg;
                progress_msg.data = mapping_progress;
                publishers[KEY::PUBLICATION::SLAM::MAPPING_PROGRESS]->publish(
                    progress_msg);
            });
    }
    {
        LoggerCallback::log_level_callback.add(std::bind(
            &Logger::SetLogLevel, logger_.get(), std::placeholders::_1));
    }
    {
        LoggerCallback::record_bag_callback.add([this]() {
            std::vector<std::string> bag_files;
            std::string path = std::string(std::getenv("HOME")) +
                Configurator::GetInstance()
                    .GetParamValue("ros", "base", "base_path")
                    .convert<std::string>() +
                "/answer/";
            // filename as date
            std::time_t t = std::time(nullptr);
            std::tm tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
            std::string file_name = oss.str() + ".bag";
            bag_files.push_back("rosbag");
            bag_files.push_back("record");
            bag_files.push_back("-O");
            bag_files.push_back(path + file_name);
            // push pack topics
            auto record_topics = Configurator::GetInstance()
                                     .GetParamValue("ros", "topic", "record")
                                     .extract<Poco::JSON::Array::Ptr>();
            for (size_t i = 0; i < record_topics->size(); ++i) {
                auto topic = record_topics->get(i).convert<std::string>();
                bag_files.push_back(topic);
            }

            logger_->RecordBag(bag_files);
        });
    }

    {
        LoggerCallback::play_bag_callback.add([this](
                                                  const std::string bag_file) {
            std::vector<std::string> bag_files;

            auto play_topics = Configurator::GetInstance()
                                   .GetParamValue("ros", "topic", "play")
                                   .extract<Poco::JSON::Array::Ptr>();

            auto play_speed = Configurator::GetInstance()
                                  .GetParamValue("ros", "topic", "play_speed")
                                  .convert<int>();
            bag_files.push_back("rosbag");
            bag_files.push_back("play");
            bag_files.push_back(bag_file);
            bag_files.push_back("--topics");
            for (size_t i = 0; i < play_topics->size(); ++i) {
                auto topic = play_topics->get(i).convert<std::string>();
                bag_files.push_back(topic);
            }
            bag_files.push_back("-r " + std::to_string(play_speed));
            logger_->PlayBag(bag_files);
        });
    }
    {
        LoggerCallback::save_pose_callback.add([this](const Pose2D &pose) {
            std::string file_name = std::string(std::getenv("HOME")) +
                Configurator::GetInstance()
                    .GetParamValue("ros", "base", "pose_text_path")
                    .convert<std::string>() +
                "/pose.txt";
            std::ofstream pose_file(file_name);
            if (pose_file.is_open()) {
                pose_file << pose.x() << "\n"
                          << pose.y() << "\n"
                          << pose.yaw() * 180.0 / M_PI << "\n";
                pose_file.close();
                LOG_INFO("Saved initial pose to {}", file_name);
            }
            else {
                LOG_ERROR("Failed to open file for saving initial pose");
            }
        });
    }
}
/*
 *@brief Load parameters from config file
 */
void AnswerRos1::Initialize()
{
    last_update_stamp_ = ros::Time::now();
    b_imu_first_update_ = false;
#if 0
    auto pkg_path = ros::package::getPath("answer");
    // std::string base_dir =
    //     std::string(std::getenv("HOME")) +
    //     "/navifra_solution/navicore/configs/answer/";
    std::string hint1 = pkg_path + "/config";
    std::string hint2 = "./config";
    std::string config_path;
    // if (std::filesystem::is_directory(base_dir)) {
    //     config_path = base_dir;
    // }
    // else
    if (std::filesystem::is_directory(hint1)) {
        config_path = hint1;
    }
    else if (std::filesystem::is_directory(hint2)) {
        config_path = hint2;
    }
#endif
    LOG_INFO("Package path: {}", Path::GetInstance().GetConfigPath());

    Configurator::GetInstance().LoadParameters("ros");

    slam_ = std::make_unique<SLAM2D::SLAM>();

    std::string mode = ANSWER::ExeArguments::GetInstance().GetArgument(
        ANSWER::KEY::EXE_ARGUMENTS::DOF);

    if (Configurator::GetInstance()
            .GetParamValue("ros", "base", "use_imu")
            .convert<bool>() == true) {
        // ESKF::GetInstance().Initialize(config_path);
    }
    {
        Configurator::GetInstance().LoadParameters("localizer2d");
        localizer_ = std::make_unique<Localizer>();
    }

    // tf
    br_ = std::make_shared<tf::TransformBroadcaster>();

    // ESKF eskf(config_path);
}

/*
 *@brief Generate ROS2 subscriptions
 */
void AnswerRos1::GenerateSubscriptions()
{
    // // subscribe

    std::string mode = ANSWER::ExeArguments::GetInstance().GetArgument(
        ANSWER::KEY::EXE_ARGUMENTS::DOF);
    if (mode == ANSWER::KEY::EXE_ARGUMENTS::ANSWER_3D) {
        // sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_3D] =
        //     std::make_shared<ros::Subscriber>(nh_.subscribe(
        //         Configurator::GetInstance()
        //             .GetParamValue("ros", "topic", "point_cloud_3d_topic")
        //             .convert<std::string>(),
        //         1, &AnswerRos1::GetLidar3D, this,
        //         ros::TransportHints().tcpNoDelay(true)));
    }
    else {
        if (Configurator::GetInstance()
                .GetParamValue("ros", "topic", "use_scan_topic")
                .convert<bool>() == false) {
            sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::POINT_CLOUD_2D] =
                std::make_shared<ros::Subscriber>(nh_.subscribe(
                    Configurator::GetInstance()
                        .GetParamValue("ros", "topic", "point_cloud_topic")
                        .convert<std::string>(),
                    1, &AnswerRos1::GetLidar2D, this,
                    ros::TransportHints().tcpNoDelay(true)));
        }
        else {
            LOG_INFO("use_scan_topic is true");

            auto extrinsic =
                Configurator::GetInstance()
                    .GetParamValue("ros", "base", "robot_to_lidar1")
                    .extract<Poco::JSON::Object::Ptr>();
            auto translation = extrinsic->getArray("translation");
            auto rotation = extrinsic->getArray("rotation");
            Eigen::VectorXf extrinsic_vec(6);
            extrinsic_vec.setZero();
            for (size_t i = 0; i < translation->size(); ++i) {
                extrinsic_vec(i) = translation->get(i).convert<float>();
            }
            for (size_t i = 0; i < rotation->size(); ++i) {
                extrinsic_vec(i + 3) =
                    (rotation->get(i).convert<float>()) * (M_PI / 180.);
            }
            extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR] =
                extrinsic_vec;
            LOG_INFO("front scan extrinsic: {}", extrinsic_vec.transpose());

            sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::FRONT_LIDAR] =
                std::make_shared<ros::Subscriber>(nh_.subscribe(
                    Configurator::GetInstance()
                        .GetParamValue("ros", "topic", "scan_topic")
                        .convert<std::string>(),
                    1, &AnswerRos1::GetLaserScan2D, this,
                    ros::TransportHints().tcpNoDelay(true)));
        }
    }

    {
        command_subscriptions[KEY::SUBSCRIPTION::COMMAND::REGISTER_REFLECTOR] =
            std::make_shared<ros::Subscriber>(nh_.subscribe(
                TOPICS::REGISTER_REFLECTOR, 1,
                &AnswerRos1::GetRegisterReflector, this,
                ros::TransportHints().tcpNoDelay(true)));
    }

    if (Configurator::GetInstance()
            .GetParamValue("ros", "base", "use_imu")
            .convert<bool>() == true) {
        auto extrinsic = Configurator::GetInstance()
                             .GetParamValue("ros", "base", "base_to_imu")
                             .extract<Poco::JSON::Object::Ptr>();
        auto translation = extrinsic->getArray("translation");
        auto rotation = extrinsic->getArray("rotation");
        Eigen::VectorXf extrinsic_vec(6);
        extrinsic_vec.setZero();
        for (size_t i = 0; i < translation->size(); ++i) {
            extrinsic_vec(i) = translation->get(i).convert<float>();
        }
        for (size_t i = 0; i < rotation->size(); ++i) {
            extrinsic_vec(i + 3) =
                (rotation->get(i).convert<float>()) * (M_PI / 180.);
        }
        extrinsics_map_[KEY::SUBSCRIPTION::SENSOR::IMU] = extrinsic_vec;

        sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::IMU] =
            std::make_shared<ros::Subscriber>(nh_.subscribe(
                Configurator::GetInstance()
                    .GetParamValue("ros", "topic", "imu_topic")
                    .convert<std::string>(),
                100, &AnswerRos1::GetImu, this,
                ros::TransportHints().tcpNoDelay(true)));
    }
    else {
        sensor_subscriptions[KEY::SUBSCRIPTION::SENSOR::ODOM] =
            std::make_shared<ros::Subscriber>(nh_.subscribe(
                Configurator::GetInstance()
                    .GetParamValue("ros", "topic", "odom_topic")
                    .convert<std::string>(),
                1, &AnswerRos1::GetOdometry2D, this,
                ros::TransportHints().tcpNoDelay(true)));
    }

    command_subscriptions[KEY::SUBSCRIPTION::COMMAND::INIT_POSE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            TOPICS::INIT_POSE, 1, &AnswerRos1::GetInitialPose, this,
            ros::TransportHints().tcpNoDelay(true)));

    command_subscriptions[KEY::SUBSCRIPTION::COMMAND::EXTERNAL_POSE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            TOPICS::EXTERNAL_POSE, 1, &AnswerRos1::GetExternalPose, this,
            ros::TransportHints().tcpNoDelay(true)));

    command_subscriptions[KEY::SUBSCRIPTION::COMMAND::CLEAR_ALARM] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            TOPICS::CLEAR_ALARM, 1, &AnswerRos1::GetClearAlarm, this,
            ros::TransportHints().tcpNoDelay(true)));

    command_subscriptions[KEY::SUBSCRIPTION::COMMAND::CORRECTION_TRIGGER] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            TOPICS::CORRECTION_TRIGGER, 10, &AnswerRos1::GetCorrectionTrigger,
            this, ros::TransportHints().tcpNoDelay(true)));

    command_subscriptions[KEY::SUBSCRIPTION::COMMAND::MAP_SAVE] =
        std::make_shared<ros::Subscriber>(nh_.subscribe(
            TOPICS::MAP_SAVE, 10, &AnswerRos1::GetMapSave, this,
            ros::TransportHints().tcpNoDelay(true)));
}

void AnswerRos1::GeneratePublishers()
{
    publishers[KEY::PUBLICATION::SLAM::MAP] = std::make_shared<ros::Publisher>(
        nh_.advertise<nav_msgs::OccupancyGrid>(TOPICS::MAP, 1));

    publishers[KEY::PUBLICATION::SLAM::POSE] = std::make_shared<ros::Publisher>(
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            TOPICS::ROBOT_POSE, 1));

    publishers[KEY::PUBLICATION::SLAM::REPORT] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<std_msgs::String>(TOPICS::REPORT, 1));

    publishers[KEY::PUBLICATION::SLAM::STATUS] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<std_msgs::String>(TOPICS::ANSWER_STATUS, 1));

    publishers[KEY::PUBLICATION::SLAM::POSE_GRAPH] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<std_msgs::String>(TOPICS::POSE_GRAPH, 1));

    publishers[KEY::PUBLICATION::SLAM::MAPPING_PROGRESS] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<std_msgs::Int16>(TOPICS::MAPPING_PROGRESS, 100));
    publishers[KEY::PUBLICATION::SLAM::DETECTED_REFLECTORS] =
        std::make_shared<ros::Publisher>(
            nh_.advertise<std_msgs::String>(TOPICS::DETECTED_REFLECTORS, 1));
}
void AnswerRos1::GenerateServices()
{
    // service
    {
        control_srv_servers_[KEY::SERVICES::CONTROL::ANSWER_CONTROL] =
            nh_.advertiseService(
                SERVICES::CONTROL_ANSWER, &AnswerRos1::ControlServiceCallback,
                this);
    }
    {
        control_srv_servers_[KEY::SERVICES::CONTROL::LOG_LEVEL] =
            nh_.advertiseService(
                SERVICES::LOG_LEVEL, &AnswerRos1::LogLevelServiceCallback,
                this);
    }

    {
        control_srv_servers_[KEY::SERVICES::CONTROL::UPDATE_PARAM] =
            nh_.advertiseService(
                SERVICES::UPDATE_PARAM, &AnswerRos1::UpdateParamServiceCallback,
                this);
    }

    {
        control_srv_servers_[KEY::SERVICES::CONTROL::SAMPLING_MATCH] =
            nh_.advertiseService(
                SERVICES::SAMPLING_MATCH,
                &AnswerRos1::SamplingMatchServiceCallback, this);
    }
}
bool AnswerRos1::ControlServiceCallback(SRV::Request &req, SRV::Response &res)
{
    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result;
    try {
        result = parser.parse(req.data);
    }
    catch (Poco::Exception &e) {
        LOG_ERROR("Error parsing JSON: {}", e.displayText());
        return false;
    }
    // LOG_INFO("Received JSON: {}", req.data);

    Poco::JSON::Object::Ptr obj = result.extract<Poco::JSON::Object::Ptr>();
    Poco::JSON::Object::Ptr output = new Poco::JSON::Object();
    if (obj->has("command")) {
        std::string command = obj->get("command").toString();
        LOG_INFO("Requested Command: {}", command);
        if (command == "start_slam") {
            int slam_mode = 1;
            StartSLAM::start_slam_callback(slam_mode);
        }
        else if (command == "terminate_slam") {
            TerminateSLAM::terminate_slam_callback();
        }
        else if (command == "start_localization") {
            // 1. start localization
            StartLocalization::start_localization_callback();

            // 2. load map default map
            std::string default_map = std::string(std::getenv("HOME")) + "/" +
                Configurator::GetInstance()
                    .GetParamValue("ros", "base", "map_path")
                    .convert<std::string>() +
                "/map/latest/map.json";
            auto object = answer_utils::ReadJSONFile(default_map);
            if (object) {
                LOG_INFO("read default map");
                LoadMap::load_json_map_callback(object);

                bool b_publish = true;
                // SLAMResultCallback::map_callback(object, b_publish);
            }

            // 3. load initial pose text file, formatted as x \n y \n degree
            std::string default_pose_file = std::string(std::getenv("HOME")) +
                Configurator::GetInstance()
                    .GetParamValue("ros", "base", "pose_text_path")
                    .convert<std::string>() +
                "/pose.txt";
            int retry_delay_ms = 500;
            for (int attempt = 1; attempt <= 10; ++attempt) {
                int fd = open(default_pose_file.c_str(), O_RDONLY);
                LOG_INFO("Read pose.txt.. attempt {}", attempt);
                if (fd == -1) {
                    LOG_ERROR(
                        "Attempt {}: Failed to open file descriptor for {}",
                        attempt, default_pose_file);
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(retry_delay_ms));
                    continue;
                }

                if (flock(fd, LOCK_SH) == -1) {
                    LOG_ERROR(
                        "Attempt {}: File is locked, retrying...", attempt);
                    close(fd);
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(retry_delay_ms));
                    continue;
                }

                std::ifstream pose_file(default_pose_file);
                if (pose_file.is_open()) {
                    std::string line;
                    std::vector<float> pose_vector;
                    while (std::getline(pose_file, line)) {
                        std::istringstream iss(line);
                        float value;
                        while (iss >> value) {
                            pose_vector.push_back(value);
                        }
                    }
                    pose_file.close();
                    flock(fd, LOCK_UN);
                    close(fd);

                    if (pose_vector.size() == 3) {
                        LOG_INFO(
                            "Read initial pose: {}, {}, {}", pose_vector[0],
                            pose_vector[1], pose_vector[2]);
                        Pose2D initial_pose(
                            pose_vector[0], pose_vector[1],
                            pose_vector[2] * M_PI / 180);
                        LocalizeCallback::init_pose_callback(initial_pose);
                    }
                }
                else {
                    LOG_ERROR("Pose vector size mismatch.");
                }
                flock(fd, LOCK_UN);
                close(fd);
                break;
            }
        }
        else if (command == "terminate_localization") {
            TerminateLocalization::terminate_localization_callback();
        }
        else if (command == "load_map") {
            StartLocalization::start_localization_callback();
            auto object = answer_utils::ConvertStringToJSON(req.data);
            LoadMap::load_json_map_callback(object);

            bool b_publish = true;
            // SLAMResultCallback::map_callback(object, b_publish);
        }
        else if (command == "restart") {
            if (AnswerStatus::GetInstance().GetStatus() ==
                ANSWER::STATUS::LOCALIZATION) {
                TerminateLocalization::terminate_localization_callback();
                // 1. start localization
                StartLocalization::start_localization_callback();

                // 2. load map default map
                std::string default_map = std::string(std::getenv("HOME")) +
                    "/" +
                    Configurator::GetInstance()
                        .GetParamValue("ros", "base", "map_path")
                        .convert<std::string>() +
                    "/map/latest/map.json";
                auto object = answer_utils::ReadJSONFile(default_map);
                if (object) {
                    LOG_INFO("read default map");
                    LoadMap::load_json_map_callback(object);

                    bool b_publish = true;
                    // SLAMResultCallback::map_callback(object, b_publish);
                }

                // 3. load initial pose text file, formatted as x \n y \n degree
                std::string default_pose_file =
                    std::string(std::getenv("HOME")) +
                    Configurator::GetInstance()
                        .GetParamValue("ros", "base", "pose_text_path")
                        .convert<std::string>() +
                    "/pose.txt";
                std::ifstream pose_file(default_pose_file);
                if (pose_file.is_open()) {
                    std::string line;
                    std::vector<float> pose_vector;
                    while (std::getline(pose_file, line)) {
                        std::istringstream iss(line);
                        float value;
                        while (iss >> value) {
                            pose_vector.push_back(value);
                        }
                    }
                    pose_file.close();
                    if (pose_vector.size() == 3) {
                        LOG_INFO(
                            "Read initial pose: {}, {}, {}", pose_vector[0],
                            pose_vector[1], pose_vector[2]);
                        Pose2D initial_pose(
                            pose_vector[0], pose_vector[1],
                            pose_vector[2] * M_PI / 180);
                        LocalizeCallback::init_pose_callback(initial_pose);
                    }
                }
            }
            else if (
                AnswerStatus::GetInstance().GetStatus() ==
                ANSWER::STATUS::SLAM) {
                TerminateSLAM::terminate_slam_callback();
                int slam_mode = 1;
                StartSLAM::start_slam_callback(slam_mode);
            }
        }
        else {
            LOG_ERROR("Unknown command: {}", command);
            return false;
        }
        // output->set("result", "Success");
        output->set("status", AnswerStatus::GetInstance().GetStatusString());
        std::ostringstream oss;
        output->stringify(oss, 4);
        res.reason = oss.str();
        res.success = true;
    }
    else {
        LOG_ERROR("No command in service request");
        output->set("result", "Fail");
        output->set("status", AnswerStatus::GetInstance().GetStatusString());
        std::ostringstream oss;
        output->stringify(oss, 4);
        res.reason = oss.str();
        res.success = false;
        return false;
    }

    return true;
}  // namespace ANSWER
bool AnswerRos1::LogLevelServiceCallback(SRV::Request &req, SRV::Response &res)
{
    if (req.data.empty()) {
        LOG_ERROR("Received empty request data");
        res.success = false;
        res.reason = "Empty request data";
        return false;
    }
    LOG_CRITICAL("Log level set to: {}", req.data);

    LoggerCallback::log_level_callback(req.data);
    res.success = true;
    res.reason = "Log level set successfully";
    return true;
}

bool AnswerRos1::UpdateParamServiceCallback(
    SRV::Request &req, SRV::Response &res)
{
    if (req.data.empty()) {
        LOG_ERROR("Received empty request data");
        res.success = false;
        res.reason = "Empty request data";
        return false;
    }
    LOG_CRITICAL("Update param : {}", req.data);

    ServiceCallback::update_parameters_callback();
    res.success = true;
    res.reason = "Update param set successfully";
    return true;
}
bool AnswerRos1::SamplingMatchServiceCallback(
    SRV::Request &req, SRV::Response &res)
{
    if (req.data.empty()) {
        LOG_ERROR("Received empty request data");
        res.success = false;
        res.reason = "Empty request data";
        return false;
    }
    LOG_CRITICAL("sampling based matching : {}", req.data);

    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result;
    Poco::JSON::Object::Ptr obj;
    try {
        result = parser.parse(req.data);
        obj = result.extract<Poco::JSON::Object::Ptr>();
    }
    catch (Poco::Exception &e) {
        LOG_ERROR("Error parsing JSON: {}", e.displayText());
        return false;
    }
    if (obj->has("method")) {
        if (obj->has("sample_num") == false ||
            obj->has("sample_max_dist") == false ||
            obj->has("sample_max_angle") == false ||
            obj->has("sample_dist_resolution") == false ||
            obj->has("sample_angle_resolution") == false) {
            LOG_ERROR("Missing parameters for random sampling");
            res.success = false;
            res.reason = "Missing parameters for random sampling";
            return false;
        }
        std::string method = obj->get("method").toString();

        bool random = true;
        auto sample_num = obj->get("sample_num").convert<int>();
        float sample_max_dist = obj->get("sample_max_dist").convert<float>();
        float sample_max_angle = obj->get("sample_max_angle").convert<float>();
        float sample_dist_resolution =
            obj->get("sample_dist_resolution").convert<float>();
        float sample_angle_resolution =
            obj->get("sample_angle_resolution").convert<float>();
        if (method == "random") {
            ServiceCallback::resampling_based_init_matching_callback(
                random, sample_num, sample_max_dist, sample_max_angle,
                sample_dist_resolution, sample_angle_resolution);
        }
        else if (method == "uniform") {
            random = false;
            ServiceCallback::resampling_based_init_matching_callback(
                random, sample_num, sample_max_dist, sample_max_angle,
                sample_dist_resolution, sample_angle_resolution);
        }
        else {
            LOG_ERROR("Unknown method: {}", method);
            res.success = false;
            res.reason = "Unknown method: " + method;
            return false;
        }
    }
    else {
        LOG_ERROR("No method in service request");
        res.success = false;
        res.reason = "No method in service request";
        return false;
    }
    res.success = true;
    res.reason = "random sampling matching set successfully";

    return true;
}
}  // namespace ANSWER