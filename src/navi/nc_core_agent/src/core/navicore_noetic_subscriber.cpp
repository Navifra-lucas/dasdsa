#include "core_agent/core/alarm_utils.h"
#include "core_agent/core_agent.h"

#include <Poco/DigestEngine.h>
#include <Poco/JSON/Parser.h>
#include <Poco/MD5Engine.h>
#include <Poco/UUIDGenerator.h>
#include <boost/format.hpp>
#include <core_agent/core/navicore_message.h>
#include <core_agent/core/navicore_noetic.h>
#include <tf/transform_datatypes.h>

#include <iostream>

using namespace NaviFra;

bool plc_alarm_clear_ = false;

void NaviCoreNoetic::onFrontCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    if (isUseLidar != true)
        return;
    auto lidarMerger = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
    lidarMerger->update(LIDAR::LIDAR_FRONT, msg);

    ros::Time time(msg->header.stamp.sec, msg->header.stamp.nsec);
    timeFrontCloud_ = time.toSec();
}

void NaviCoreNoetic::onRearCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    if (isUseLidar != true)
        return;
    auto lidarMerger = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
    lidarMerger->update(LIDAR::LIDAR_REAR, msg);

    ros::Time time(msg->header.stamp.sec, msg->header.stamp.nsec);
    timeRearCloud_ = time.toSec();
}

void NaviCoreNoetic::onLeftCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    if (isUseLidar != true)
        return;
    auto lidarMerger = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
    lidarMerger->update(LIDAR::LIDAR_LEFT, msg);

    ros::Time time(msg->header.stamp.sec, msg->header.stamp.nsec);
    timeLeftCloud_ = time.toSec();
}

void NaviCoreNoetic::onRightCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    if (isUseLidar != true)
        return;
    auto lidarMerger = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
    lidarMerger->update(LIDAR::LIDAR_RIGHT, msg);

    ros::Time time(msg->header.stamp.sec, msg->header.stamp.nsec);
    timeRightCloud_ = time.toSec();

    return;
}

void NaviCoreNoetic::onV2VCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    if (isUseLidar != true)
        return;
    auto lidarMerger = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
    lidarMerger->update(LIDAR::LIDAR_V2V, msg);

    ros::Time time(msg->header.stamp.sec, msg->header.stamp.nsec);
    timeV2VCloud_ = time.toSec();

    return;
}

void NaviCoreNoetic::onCameraCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    if (isUseLidar != true)
        return;
    auto lidarMerger = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
    lidarMerger->update(LIDAR::LIDAR_CAMERA, msg);

    ros::Time time(msg->header.stamp.sec, msg->header.stamp.nsec);
    timeCameraCloud_ = time.toSec();

    return;
}

void NaviCoreNoetic::onObsCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    if (isUseLidar != true)
        return;
    auto lidarMerger = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
    lidarMerger->update(LIDAR::LIDAR_OBS, msg);

    ros::Time time(msg->header.stamp.sec, msg->header.stamp.nsec);
    timeObsCloud_ = time.toSec();

    return;
}

void NaviCoreNoetic::onObstacle(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    // NLOG(info) << "onObstacle";
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_OBSTACLE)) {
        Poco::Timestamp now;
        static Poco::Timestamp last;
        // NLOG(info) << "onObstacle - last: " << last.epochMicroseconds() << ", now: " << now.epochMicroseconds();
        // if ((now - last) / 1000 <= 1000)
        //     return;
        // NLOG(info) << "onObstacle - processing";
        try {
            std::string points;
            points.append("[");
            size_t count = 0;
            for (const auto& point : msg->polygon.points) {
                if (count == 0) {
                    std::string value;
                    Poco::format(value, "[%.3hf,%.3hf,0]", point.x, point.y);
                    points.append(value);
                }
                else {
                    std::string value;
                    Poco::format(value, ",[%.3hf,%.3hf,0]", point.x, point.y);
                    points.append(value);
                }
                count++;
            }
            points.append("]");

            Poco::JSON::Object::Ptr message = new Poco::JSON::Object();

            message->set("polygon_name", "obstacle_pos");
            message->set("points", points);

            // NLOG(info) << boost::format("Recv Obstacle Polygon [%1%]") % points;

            MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_OBSTACLE, std::move(message));
        }
        catch (const Poco::Exception& ex) {
            NLOG(error) << ex.displayText();
        }

        last.update();
    }
}

void NaviCoreNoetic::onIsSlam(const std_msgs::Bool::ConstPtr msg)
{
    if (msg->data) {
        if (!InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->isSLAM()) {
            InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setSLAM(true);
        }
    }
    return;
}

void NaviCoreNoetic::onMappingProgress(const std_msgs::Int16::ConstPtr msg)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_MAPING_PROGRESS)) {
        Poco::JSON::Object::Ptr message = new Poco::JSON::Object();
        message->set("percent", static_cast<double>(msg->data) / 100);

        MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_MAPING_PROGRESS, message);
    }
}

void NaviCoreNoetic::onRobotPos(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    if (!isSLAM())
        return;

    Poco::Timestamp now;
    if ((now - statusPubTime_) / 1000 < 100) {
        return;
    }

    Position position;
    Orientation orientation;

    position.x = msg.pose.pose.position.x;
    position.y = msg.pose.pose.position.y;
    position.z = msg.pose.pose.position.z;

    orientation.x = msg.pose.pose.orientation.x;
    orientation.y = msg.pose.pose.orientation.y;
    orientation.z = msg.pose.pose.orientation.z;
    orientation.w = msg.pose.pose.orientation.w;
    // NLOG(info) << "position: " << position.x << ", " << position.y << ", " << position.z;

    InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY)->update(position, orientation);
}

void NaviCoreNoetic::onNavifrainfo(const core_msgs::NavicoreStatus msg)
{
    try {
        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

        if (msg.s_current_node_id.size() > 1) {
            s_current_node_id_ = msg.s_current_node_id;
        }
        robotInfo->updateRobotInfo(
            {msg.f_robot_current_linear_vel_x, msg.f_robot_current_linear_vel_y, msg.f_robot_current_angular_vel_w},
            {s_current_node_id_, msg.s_next_node_id, msg.s_goal_node_id, msg.f_path_progress}, msg.n_confidence);

        robotInfo->updateFootprint(msg.f_robot_pos_x_m, msg.f_robot_pos_y_m);
        robotInfo->updateErrorPose(msg.f_error_pos_x_m, msg.f_error_pos_y_m, msg.f_error_pos_deg);
        robotInfo->setStatus(msg.s_status);
        robotInfo->setBattery(msg.f_robot_battery);
        robotInfo->setErrordist(msg.f_path_error_dist_m);
        robotInfo->updateTargetVelocity(msg.f_robot_target_linear_vel_x, msg.f_robot_target_linear_vel_y, msg.f_robot_target_angular_vel_w);

        float robotOrientationDeg = msg.f_robot_pos_deg;
        tf::Quaternion q;
        q.setRPY(0, 0, robotOrientationDeg * 3.141592 / 180);
        std::vector<float> robotOrientationQuat = {0, 0, 0, 0};

        Position position;
        Orientation orientation;

        position.x = isnan(msg.f_robot_pos_x_m) ? 0 : msg.f_robot_pos_x_m;
        position.y = isnan(msg.f_robot_pos_y_m) ? 0 : msg.f_robot_pos_y_m;
        position.deg = isnan(msg.f_robot_pos_deg) ? 0 : msg.f_robot_pos_deg;
        orientation.x = isnan(q.getX()) ? 0 : q.getX();
        orientation.y = isnan(q.getY()) ? 0 : q.getY();
        orientation.z = isnan(q.getZ()) ? 0 : q.getZ();
        orientation.w = isnan(q.getW()) ? 0 : q.getW();

        InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY)->update(position, orientation);
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

std::vector<std::string> NaviCoreNoetic::splitString(const std::string& str, char delimiter)
{
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        result.push_back(token);  // 빈 문자열도 허용
    }

    // 문자열이 구분자로 끝날 경우 마지막 빈 토큰 추가
    if (!str.empty() && str.back() == delimiter) {
        result.push_back("");
    }

    // 특수 케이스: 입력이 구분자만 하나일 때 (예: "/")
    if (str == std::string(1, delimiter)) {
        result = {"", ""};
    }

    return result;
}

void NaviCoreNoetic::onTaskInfo(const std_msgs::String msg)
{
    static std::string s_previous_task_id = "";
    try {
        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        std::vector<std::string> vec_uuid = splitString(msg.data, '/');
        if (vec_uuid.size() != 2) {
            NLOG(error) << "Invalid task info format: " << msg.data;
            return;
        }
        if (vec_uuid[1] == "") {
            vec_uuid[1] = s_previous_task_id;
        }
        robotInfo->updateJob(vec_uuid[0], vec_uuid[1]);
        s_previous_task_id = vec_uuid[1];
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void NaviCoreNoetic::onTaskAlarm(const core_msgs::TaskAlarm msg)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_TASK_ALAM)) {
        Poco::JSON::Object::Ptr data = new Poco::JSON::Object();
        Poco::JSON::Object::Ptr message = new Poco::JSON::Object();

        Poco::UUID uuid = Poco::UUIDGenerator::defaultGenerator().create();

        message->set("uuid", uuid.toString());
        message->set("action", "task_alarm");
        message->set("from", "task_manager");
        message->set("result", "");
        data->set("alarm", msg.alarm);
        data->set("task_id", msg.uuid);

        message->set("data", data);
        NLOG(info) << boost::format("Recv Task Alarm UUID [%1%] , Alarm [%2%]") % msg.uuid % msg.alarm;

        MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_TASK_ALAM, std::move(message));
    }
}

void NaviCoreNoetic::onTaskResponse(const core_msgs::TaskAlarm msg)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_TASK_RESPONSE)) {
        Poco::JSON::Object data;
        Poco::JSON::Object::Ptr message = new Poco::JSON::Object();

        NLOG(info) << boost::format("Recv Task Response UUID [%1%] , Alarm [%2%]") % msg.uuid % msg.alarm;

        message->set("uuid", msg.uuid);
        message->set("result", "success");
        data.set("alarm", msg.alarm);
        message->set("data", data);

        MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_TASK_RESPONSE, std::move(message));
    }
}

void NaviCoreNoetic::onMap(const nav_msgs::OccupancyGrid msg)
{
    // try {
    //     if (msg.data.size() == 0)
    //         return;
    //     Poco::JSON::Object::Ptr slam = NcInMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getSlamMap();
    //     Poco::JSON::Object message;
    //     message.set("id", NcInMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
    //     message.set("can_save", true);
    //
    //     if (slam.get()) {
    //         auto names = slam->getNames();
    //         for (const auto& name : names) {
    //             message.set(name, slam->get(name));
    //         }
    //     }
    //
    //     std::ostringstream oss;
    //     message.stringify(oss);
    //     DefaultPublisher::instance().publisher->publish(
    //         NcBrainMessage::MESSAGE_ROBOT_STATUS_SLAM + NcInMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
    //         oss.str());
    // }
    // catch (Poco::Exception& ex) {
    //     NLOG(error) << ex.displayText();
    // }
}

void NaviCoreNoetic::onErrorDist(const std_msgs::Float64 msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setErrordist(msg.data);
}

void NaviCoreNoetic::onSetSeverityMin(const std_msgs::Int16 msg)
{
    if (msg.data < 0 && msg.data > 5)
        return;
    NaviFra::Logger::get().SetSeverityMin((severity_level)msg.data);
}

void NaviCoreNoetic::onGlobalPath(const nav_msgs::Path path)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_GLOBAL_PATH)) {
        Poco::JSON::Object::Ptr message = new Poco::JSON::Object();
        Poco::JSON::Array global_path;
        for (const auto& pose : path.poses) {
            Poco::JSON::Object position;
            Poco::JSON::Object orientation;
            Poco::JSON::Object path;
            position.set("x", pose.pose.position.x);
            position.set("y", pose.pose.position.y);
            position.set("z", pose.pose.position.z);
            orientation.set("x", pose.pose.orientation.x);
            orientation.set("y", pose.pose.orientation.y);
            orientation.set("z", pose.pose.orientation.z);
            orientation.set("w", pose.pose.orientation.w);
            path.set("position", position);
            path.set("orientation", orientation);
            global_path.add(path);
        }

        message->set("global_path", global_path);

        MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_GLOBAL_PATH, std::move(message));
    }
}

void NaviCoreNoetic::onLocalPath(const nav_msgs::Path path)
{
    Poco::Timestamp now;
    static Poco::Timestamp last;

    if ((now - last) / 200 <= 200)
        return;

    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_LOCAL_PATH)) {
        Poco::JSON::Object::Ptr message = new Poco::JSON::Object();
        Poco::JSON::Array local_path;
        for (const auto& pose : path.poses) {
            Poco::JSON::Object position;
            Poco::JSON::Object orientation;
            Poco::JSON::Object path;
            position.set("x", pose.pose.position.x);
            position.set("y", pose.pose.position.y);
            position.set("z", pose.pose.position.z);
            orientation.set("x", pose.pose.orientation.x);
            orientation.set("y", pose.pose.orientation.y);
            orientation.set("z", pose.pose.orientation.z);
            orientation.set("w", pose.pose.orientation.w);
            path.set("position", position);
            path.set("orientation", orientation);
            local_path.add(path);
        }
        message->set("local_path", local_path);
        MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_LOCAL_PATH, std::move(message));
    }

    last.update();
}

void NaviCoreNoetic::onCaliResult(const std_msgs::Float64MultiArray::ConstPtr msg)
{
    try {
        if (msg->data.size() == 0)
            return;
        auto dmCalibration = InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY);

        if (dmCalibration->size() == 7) {
            dmCalibration->clear();
        }

        for (int i = 0; i < msg->data.size(); i++) {
            dmCalibration->appendResult(msg->data[i]);
        }
    }
    catch (const Poco::Exception& ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }
}

void NaviCoreNoetic::onCaliProgressValue(const std_msgs::Float64MultiArray::ConstPtr msg)
{
    try {
        if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_CALI_PROGRESS)) {
            if (msg->data.size() == 0)
                return;

            Poco::JSON::Object::Ptr message = new Poco::JSON::Object();

            // message.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
            message->set("type", InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY)->currentType());
            std::string current = InMemoryRepository::instance().get<RobotCalibration>(RobotCalibration::KEY)->currentType();
            if (current == TypeCalibration::FOWARD || current == TypeCalibration::LEFT || current == TypeCalibration::BACKWARD ||
                current == TypeCalibration::RIGHT) {
                LOG_INFO(
                    "TYPE_ROBOT_CALIBRATION _FOWARD _BACKWARD _LEFT _RIGHT Percent "
                    ": %.3f",
                    msg->data[3]);
                float fpercent = msg->data[3];

                if (fpercent == 100) {
                    fpercent = 1.0;
                }
                else if (fpercent > 0.99) {
                    fpercent = 0.99;
                }

                message->set("percent", fpercent);
                message->set("x", msg->data[0]);
                message->set("y", msg->data[1]);
                message->set("deg", msg->data[2]);
            }
            else if (current == TypeCalibration::LIDAR) {
                LOG_INFO("Lidar Percent : %.3f", msg->data[3] / 100);
                float fpercent = msg->data[3] / 100;
                if (fpercent == 100) {
                    fpercent = 1.0;
                }
                else if (fpercent > 0.99) {
                    fpercent = 0.99;
                }

                message->set("percent", fpercent);
                switch ((int)msg->data[0]) {
                    case 0:  // move
                        message->set("cmd", "move");
                        break;
                    case 1:  // rotation
                        message->set("cmd", "rotation");
                        break;
                    case 2:  // cal sensor info
                        message->set("cmd", "cal_sensor_info");
                        break;
                    case 3:  // cal yaw
                        message->set("cmd", "cal_yaw");
                        break;
                    case 4:  // cal done
                        message->set("cmd", "cal_done");
                        break;
                }
                message->set("completed", (int)msg->data[0]);
                message->set("total", 4);
            }
            else {
                LOG_ERROR("Can not found type of robot-calibration !");
            }

            MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_CALI_PROGRESS, std::move(message));
        }
    }
    catch (const Poco::Exception& ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }
}

void NaviCoreNoetic::onPredictCollision(const geometry_msgs::PolygonStamped msg)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_PREDICT_COLLISION)) {
        std::string points;
        points.append("[");
        size_t count = 0;
        for (const auto& point : msg.polygon.points) {
            if (count == 0) {
                std::string value;
                Poco::format(value, "[%.3hf,%.3hf,0]", point.x, point.y);
                points.append(value);
            }
            else {
                std::string value;
                Poco::format(value, ",[%.3hf,%.3hf,0]", point.x, point.y);
                points.append(value);
            }
            count++;
        }
        points.append("]");

        Poco::JSON::Object::Ptr message = new Poco::JSON::Object();

        message->set("polygon_name", "predict_collision");
        message->set("points", points);

        MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_PREDICT_COLLISION, message);
    }
}

void NaviCoreNoetic::onMotorInfo(const core_msgs::MotorInfo::ConstPtr msg)
{
    InMemoryRepository::instance().get<RobotWheelInfoStore>(RobotWheelInfoStore::KEY)->updateWheelInfo(msg);

    try {
        if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESASGE_MOTOR_INFO)) {
            InMemoryRepository::instance().get<MotorInfoStore>(MotorInfoStore::KEY)->append(msg);
            if (InMemoryRepository::instance().get<MotorInfoStore>(MotorInfoStore::KEY)->size() == 5) {
                MessageHandlerManager::instance().handleMessage(
                    CoreMessage::CORE_MESASGE_MOTOR_INFO,
                    InMemoryRepository::instance().get<MotorInfoStore>(MotorInfoStore::KEY)->toObject());
                InMemoryRepository::instance().get<MotorInfoStore>(MotorInfoStore::KEY)->clear();
            }
        }
    }
    catch (Poco::Exception ex) {
        NLOG(error) << "onMotorInfo error : " << ex.displayText();
    }
}

void NaviCoreNoetic::onOdometry(const nav_msgs::Odometry::ConstPtr msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->updateVelocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);
}

void NaviCoreNoetic::onCollision(const geometry_msgs::PolygonStamped polygon)
{
    auto collision = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);
    std::vector<Position> position;
    for (auto point : polygon.polygon.points) {
        position.push_back({point.x, point.y, point.z});
    }
    collision->updateCollision(position);
}

void NaviCoreNoetic::onCustomMessgae(const std_msgs::String::ConstPtr msg)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_CUSTOM)) {
        static std::string strMessage;
        Poco::JSON::Object::Ptr message = new Poco::JSON::Object();
        if (strMessage != msg->data) {
            strMessage = msg->data;
            message->set("uuid", Poco::UUIDGenerator::defaultGenerator().create().toString());
            message->set("type", "info");
            message->set("message", strMessage);

            MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_CUSTOM, std::move(message));
        }
    }
}

void NaviCoreNoetic::onChatterCallback(const rosgraph_msgs::Log::ConstPtr msg)
{
    if (msg->level <= rosgraph_msgs::Log::WARN) {
        // NLOG(error) << "Node Error Name: " << msg->name << " " << msg->msg;
    }
}

void NaviCoreNoetic::onMotorDriveServeOn(const std_msgs::String::ConstPtr msg)
{
    Poco::JSON::Object objMainMessage, objSubMessage;
    objSubMessage.set("value", true);
    objMainMessage.set("motor_drive_serveon", objSubMessage);
}

/*
    AGENT에서 Paramread라는 토픽을 보내면 LaunchManger에서 파라메터 업데이트 하고 토픽을 보냄
*/

void NaviCoreNoetic::onParamUpdate(const std_msgs::String::ConstPtr msg)
{
    NLOG(info) << "Received ROS Param update";
    // // TODO update

    std::vector<float> robot_outline;
    std::vector<float> robot_collision;
    bool b_use_detection_mode_flag = false;
    int n_lift_max_dist;
    int n_lift_min_dist;
    ros::param::get("obstacle/outline", robot_outline);
    ros::param::get("obstacle/collision", robot_collision);
    ros::param::get("obstacle/b_use_detection_mode_flag", b_use_detection_mode_flag);
    ros::param::get("fork_lift/lift_max_dist", n_lift_max_dist);
    ros::param::get("fork_lift/lift_min_dist", n_lift_min_dist);

    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    auto collision = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);

    float collision_offestX = 0.f;
    float collision_offestY = 0.f;
    if (robot_outline.size() == 4) {
        LOG_INFO("Read Robot Outline successfully.");
        collision->initShape(robot_outline[0], robot_outline[1], robot_outline[3], robot_outline[2], collision_offestX, collision_offestY);
    }
    if (robot_collision.size() == 4 && b_use_detection_mode_flag) {
        LOG_INFO("Read Robot collision successfully.");
        collision->initCollision(
            robot_collision[0], robot_collision[1], robot_collision[3], robot_collision[2], collision_offestX, collision_offestY);
    }
    else {
        LOG_INFO("Read Robot collision not read.");
        collision->initCollision(0, 0, 0, 0, collision_offestX, collision_offestY);
    }

    int n_kinematics = 0;
    float f_base = 0;
    ros::param::get("motion_base/n_kinematics_type", n_kinematics);
    if (n_kinematics == 0)  // dd
    {
        ros::param::get("driver_wheel/f_FL_wheel_y_m", f_base);
        f_base *= 2;
    }
    else if (n_kinematics == 1)  // qd
    {
        ros::param::get("driver_wheel/f_FL_wheel_x_m", f_base);
    }
    else if (n_kinematics == 2)  // sd
    {
        ros::param::get("driver_wheel/f_FL_wheel_x_m", f_base);
    }

    robotInfo->setForkLiftMax(n_lift_max_dist);
    robotInfo->setForkLiftMin(n_lift_min_dist);
    robotInfo->setRobotBase(f_base);
}

void NaviCoreNoetic::onSLAMGraph(const std_msgs::String::ConstPtr msg)
{
    Poco::JSON::Object::Ptr jsonObject = nullptr;
    try {
        // Create a Poco JSON parser
        Poco::JSON::Parser parser;

        // Parse the JSON string
        Poco::Dynamic::Var result = parser.parse(msg->data);

        // Convert to JSON Object
        jsonObject = result.extract<Poco::JSON::Object::Ptr>();
    }
    catch (Poco::Exception& ex) {
        LOG_ERROR("Error %s", ex.displayText());
    }
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_SLAM_GRAPH)) {
        constexpr char NODE_TYPE_CD[] = "D40099";
        constexpr char LINK_TYPE_CD[] = "D40199";

        try {
            if (msg->data.empty())
                return;

            // Poco::JSON::Object::Ptr message = new Poco::JSON::Object();

            // Poco::JSON::Array nodes;
            // Poco::JSON::Array links;

            // for (const auto& node : msg->nodes) {
            //     Poco::JSON::Object data, position, orientation;
            //     data.set("id", node.id);
            //     data.set("node_type_cd", NODE_TYPE_CD);
            //     position.set("x", node.position.x);
            //     position.set("y", node.position.y);
            //     position.set("z", node.position.z);
            //     data.set("position", position);

            //     orientation.set("x", node.orientation.x);
            //     orientation.set("y", node.orientation.y);
            //     orientation.set("z", node.orientation.z);
            //     orientation.set("w", node.orientation.w);
            //     data.set("orientation", orientation);

            //     nodes.add(data);
            // }

            // for (const auto& link : msg->links) {
            //     Poco::JSON::Object data, connected;
            //     data.set("id", link.id);
            //     data.set("link_type_cd", LINK_TYPE_CD);

            //     connected.set("from", link.from);
            //     connected.set("to", link.to);
            //     data.set("connected", connected);

            //     links.add(data);
            // }

            // message->set("nodes", nodes);
            // message->set("links", links);

            MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_SLAM_GRAPH, jsonObject);
        }
        catch (Poco::Exception& ex) {
            NLOG(error) << ex.displayText();
        }
    }
}

void NaviCoreNoetic::onHardwareInfo(const std_msgs::String::ConstPtr msg)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_HARDWARE_INFO)) {
        try {
            Poco::JSON::Parser parser;
            Poco::Dynamic::Var result = parser.parse(msg->data);
            Poco::JSON::Object::Ptr message = result.extract<Poco::JSON::Object::Ptr>();

            MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_HARDWARE_INFO, message);
        }
        catch (Poco::Exception& ex) {
            NLOG(error) << ex.displayText();
        }
    }
}

struct AlarmInfo {
    std::string category;
    int alarm_id;
    std::string description;
};

std::vector<AlarmInfo> active_alarms;  // 발생한 모든 알람 저장

void NaviCoreNoetic::onRecvAlarm(const core_msgs::NaviAlarm::ConstPtr& msg)
{
    try {
        NLOG(info) << msg->alarm_text;
        std::string category = classifyAlarm(static_cast<std::string>(msg->alarm_text));

        // AlarmInfo 구조체 생성
        AlarmInfo alarm;
        alarm.category = category;
        alarm.alarm_id = msg->alarm;
        alarm.description = static_cast<std::string>(msg->alarm_text);

        // 알람 리스트에 추가
        if (alarm.alarm_id > 2000 && alarm.category != "a") {
            active_alarms.push_back(alarm);
        }
        // 기존 로그 함수 호출
        NLOG(info) << category << " / " << msg->alarm << " / " << alarm.description;
        logClassifiedAlarm(category, msg->alarm, alarm.description);
    }
    catch (const std::exception& e) {
        NLOG(error) << e.what() << '\n';
    }

    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_NAVI_ALARM) &&
        msg->alarm >= 2000) {  // 2000 번 대 이하 알람은 시스템 알람임
        try {
            Poco::JSON::Object::Ptr message = new Poco::JSON::Object();
            message->set("alarm_id", msg->alarm);
            message->set("descrition", static_cast<std::string>(msg->alarm_text));
            MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_NAVI_ALARM, message);
        }
        catch (Poco::Exception& ex) {
            NLOG(error) << ex.displayText();
        }
    }
}

void NaviCoreNoetic::onPlcAlarmClear(const std_msgs::Bool::ConstPtr& msg)
{
    try {
        if (msg->data) {
            plc_alarm_clear_ = true;
            NLOG(info) << "PLC Alarm Clear";

            for (const auto& alarm : active_alarms) {
                NLOG(info) << "현재 활성 알람 재처리: " << alarm.description;
                logClassifiedAlarm(alarm.category, alarm.alarm_id, alarm.description);
            }
            // 재처리 후 필요하면 clear
            active_alarms.clear();
        }
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
        plc_alarm_clear_ = false;
    }
}

void NaviCoreNoetic::onSetReflectors(const std_msgs::String::ConstPtr& msg)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_SET_REFLECTORS)) {
        // NLOG(info) << "onSetReflectors: " << msg->data;
        // }
        auto circle_fitted = [](sensor_msgs::PointCloud& cloud, const float& center_x, const float& center_y, const float radius,
                                const int resolution) {
            // cloud.points.resize(360 / resolution);
            for (int i = 0; i < 360; i += resolution) {
                float angle = i * M_PI / 180.0f;
                geometry_msgs::Point32 point;
                point.x = center_x + radius * cos(angle);
                point.y = center_y + radius * sin(angle);
                point.z = 0.0f;  // Assuming a flat plane
                cloud.points.push_back(point);
            }
        };
        auto square_polygon = [](const std::string& name, const float& center_x, const float& center_y, const float size) {
            geometry_msgs::PolygonStamped polygon;
            geometry_msgs::Point32 p1, p2, p3, p4;
            p1.x = center_x - size / 2;
            p1.y = center_y - size / 2;
            p2.x = center_x + size / 2;
            p2.y = center_y - size / 2;
            p3.x = center_x + size / 2;
            p3.y = center_y + size / 2;
            p4.x = center_x - size / 2;
            p4.y = center_y + size / 2;

            // polygon.polygon.points.resize(4);
            polygon.polygon.points.emplace_back(p1);
            polygon.polygon.points.emplace_back(p2);
            polygon.polygon.points.emplace_back(p3);
            polygon.polygon.points.emplace_back(p4);

            if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_OBSTACLE)) {
                try {
                    std::string points;
                    points.append("[");
                    size_t count = 0;
                    for (const auto& point : polygon.polygon.points) {
                        // NLOG(info) << "onObstacle - point: " << point.x << ", " << point.y;
                        if (count == 0) {
                            std::string value;
                            Poco::format(value, "[%.3hf,%.3hf,0]", point.x, point.y);
                            points.append(value);
                        }
                        else {
                            std::string value;
                            Poco::format(value, ",[%.3hf,%.3hf,0]", point.x, point.y);
                            points.append(value);
                        }
                        count++;
                    }
                    points.append("]");

                    Poco::JSON::Object::Ptr message = new Poco::JSON::Object();

                    message->set("polygon_name", name);
                    message->set("points", points);

                    // NLOG(info) << boost::format("Recv Obstacle Polygon [%1%]") % points;

                    MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_OBSTACLE, std::move(message));
                }
                catch (const Poco::Exception& ex) {
                    NLOG(error) << ex.displayText();
                }
            }
        };

        int resolution = 12;
        float radius = 0.4f;

        sensor_msgs::PointCloud left_cloud, right_cloud;
        geometry_msgs::PolygonStamped detected_polygon;
        left_cloud.header.frame_id = "map";
        left_cloud.header.stamp = ros::Time::now();
        right_cloud.header.frame_id = "map";
        right_cloud.header.stamp = ros::Time::now();

        Poco::JSON::Object::Ptr jsonObject = nullptr;
        try {
            // Create a Poco JSON parser
            Poco::JSON::Parser parser;

            // Parse the JSON string
            Poco::Dynamic::Var result = parser.parse(msg->data);

            // Convert to JSON Object
            jsonObject = result.extract<Poco::JSON::Object::Ptr>();
        }
        catch (Poco::Exception& ex) {
            LOG_ERROR("Error {}", ex.displayText());
        }
        try {
            if (!jsonObject->has("reflectors")) {
                NLOG(error) << "onSetReflectors: JSON object does not contain 'reflectors' array";
                return;
            }

            if (!jsonObject->has("sync_pose")) {
                NLOG(error) << "onSetReflectors: JSON object does not contain 'sync_pose' array";
                return;
            }

            Poco::JSON::Object::Ptr message = new Poco::JSON::Object();
            Poco::JSON::Array::Ptr reflectors = jsonObject->getArray("reflectors");
            Poco::JSON::Object::Ptr sync_pose = jsonObject->getObject("sync_pose");
            float map_to_robot_position_x = sync_pose->getValue<float>("x");
            float map_to_robot_position_y = sync_pose->getValue<float>("y");
            float map_to_robot_yaw = sync_pose->getValue<float>("yaw");
            for (size_t i = 0; i < reflectors->size(); i++) {
                Poco::Dynamic::Var row = reflectors->get(i);
                Poco::JSON::Object::Ptr row_obj = row.extract<Poco::JSON::Object::Ptr>();
                if (row_obj->has("uuid") == false) {
                    NLOG(error) << "Detected reflector row is not valid, expected uuid";
                    continue;
                }
                float local_x = row_obj->getValue<float>("x");
                float local_y = row_obj->getValue<float>("y");
                std::string uuid = row_obj->getValue<std::string>("uuid");
                message->set("uuid", uuid);
                message->set("name", "detected_" + std::to_string(i));
                message->set("marker_type_cd", "D40402");

                float map_to_reflector_x = map_to_robot_position_x + cos(map_to_robot_yaw) * local_x - sin(map_to_robot_yaw) * local_y;
                float map_to_reflector_y = map_to_robot_position_y + sin(map_to_robot_yaw) * local_x + cos(map_to_robot_yaw) * local_y;

                Poco::JSON::Object position;
                Poco::JSON::Object orientation;
                position.set("x", map_to_reflector_x);
                position.set("y", map_to_reflector_y);
                position.set("z", 0);
                orientation.set("x", 0);
                orientation.set("y", 0);
                orientation.set("z", 0);
                orientation.set("w", 1);

                // circle_fitted(right_cloud, global_x, global_y, radius, resolution);
                // circle_fitted(right_cloud, global_x, global_y, radius / 4, resolution);
#if 0
                // transform global x,y to the robot pose frame considering the robot's current pose
                auto pose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
                Position map_to_robot_position = pose->getPosition();
                Orientation map_to_robot_orientation = pose->getOrientation();
                // quaternion to roll pitch yaw
                tf::Quaternion q(
                    map_to_robot_orientation.x, map_to_robot_orientation.y, map_to_robot_orientation.z, map_to_robot_orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                float robot_to_reflector_x, robot_to_reflector_y;

                // considering the robot's heading
                robot_to_reflector_x = cos(yaw) * (global_x - map_to_robot_position.x) + sin(yaw) * (global_y - map_to_robot_position.y);
                robot_to_reflector_y = -sin(yaw) * (global_x - map_to_robot_position.x) + cos(yaw) * (global_y - map_to_robot_position.y);

                // square_polygon(detected_polygon, robot_to_reflector_x, robot_to_reflector_y, radius);
                // square_polygon(detected_polygon, robot_to_reflector_x, robot_to_reflector_y, radius / 2);
                square_polygon("detected_" + std::to_string(i), robot_to_reflector_x, robot_to_reflector_y, radius);
#endif
                square_polygon("detected_" + std::to_string(i), local_x, local_y, radius);

                message->set("position", position);
                message->set("orientation", orientation);
                MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_SET_REFLECTORS, message);
            }

            // onCameraCloud(boost::make_shared<sensor_msgs::PointCloud const>(right_cloud));
            // onObstacle(boost::make_shared<geometry_msgs::PolygonStamped const>(detected_polygon));
        }
        catch (Poco::Exception& ex) {
            NLOG(error) << ex.displayText();
        }

        // MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_SET_REFLECTORS, message);
    }
}
void NaviCoreNoetic::onSetMarker(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (MessageHandlerManager::instance().has(CoreMessage::CORE_MESSAGE_SET_MARKER)) {
        try {
            Poco::JSON::Object::Ptr message = new Poco::JSON::Object();

            Poco::MD5Engine md5;
            md5.update(msg->header.frame_id);

            const Poco::DigestEngine::Digest& digest = md5.digest();

            std::string s_uuid = Poco::DigestEngine::digestToHex(digest);

            message->set("uuid", s_uuid);
            message->set("name", msg->header.frame_id);
            message->set("marker_type_cd", "D40401");

            Poco::JSON::Object position;
            Poco::JSON::Object orientation;
            position.set("x", msg->pose.position.x);
            position.set("y", msg->pose.position.y);
            position.set("z", msg->pose.position.z);
            orientation.set("x", msg->pose.orientation.x);
            orientation.set("y", msg->pose.orientation.y);
            orientation.set("z", msg->pose.orientation.z);
            orientation.set("w", msg->pose.orientation.w);

            message->set("position", position);
            message->set("orientation", orientation);
            MessageHandlerManager::instance().handleMessage(CoreMessage::CORE_MESSAGE_SET_MARKER, message);
        }
        catch (Poco::Exception& ex) {
            NLOG(error) << ex.displayText();
        }
    }
}

void NaviCoreNoetic::onBatteryInfo(const core_msgs::BatteryInfo::ConstPtr& msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    // robotInfo->setChargingState(msg->b_charging_state);
}

void NaviCoreNoetic::onRecvCharge(const std_msgs::Bool::ConstPtr& msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setChargingState(msg->data);
}

void NaviCoreNoetic::onRecvChargingCmd(const task_msgs::Charging::ConstPtr& msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    if(msg->mode == "charging") {
        robotInfo->setBatteryCmd(1);
        std_msgs::Bool b_msg;
        b_msg.data = true;
        if (sendROSMessage("charging_success", b_msg) != true) {
            LOG_ERROR("ros publisher not found action command charging_success");
        }   
    }
    else if(msg->mode == "uncharging") {
        robotInfo->setBatteryCmd(0);
    }
}

void NaviCoreNoetic::onRecvLightingCmd(const std_msgs::Bool::ConstPtr& msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setLight(msg->data);
}

void NaviCoreNoetic::onRecvOssdFieldCmd(const std_msgs::Int16::ConstPtr& msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setOSSDField(msg->data);
}

void NaviCoreNoetic::onPallectID(const std_msgs::String::ConstPtr& msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setPalletID(msg->data);
}

void NaviCoreNoetic::onWiaNowTask(const std_msgs::String::ConstPtr& msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    robotInfo->setNowTask(msg->data);
}

void NaviCoreNoetic::onForkCmd(const core_msgs::WiaForkInfo::ConstPtr& msg)
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    NLOG(info)<<"Recv Fork Cmd : Cancal - "<<msg->b_cancel<<", Height - "<<msg->n_fork_height<<", Tilt - "<<msg->f_fork_tilt<<", Wide - "<<msg->n_fork_wide;
    if(msg->b_cancel) {
        robotInfo->setForkUpDownCmd(0);
        robotInfo->setTiltingUpDownCmd(0);
        robotInfo->setForkWidthCmd(0);
    }
    else {
        if(msg->n_fork_height >= 0) {
            int n_lift_max_dist = robotInfo->getForkLiftMax();
            int n_lift_min_dist = robotInfo->getForkLiftMin();
            int n_fork_height = msg->n_fork_height;
            if(msg->n_fork_height > n_lift_max_dist) {
                n_fork_height = n_lift_max_dist;
            }
            else if(msg->n_fork_height < n_lift_min_dist) {
                n_fork_height = n_lift_min_dist;
            }
            robotInfo->setForkUpDownPosition(n_fork_height);
            robotInfo->setForkUpDownCmd(1);
        }
        if(msg->f_fork_tilt >= 0) {
            int n_tilt = 1;
            if(msg->f_fork_tilt > 0.1f) {
                n_tilt = 2;
            }
            robotInfo->setTiltingUpDownCmd(n_tilt);
        }
        if(msg->n_fork_wide >= 0) {
            int n_width = 1;
            if(msg->n_fork_wide > 0.1f) {
                n_width = 2;
            }
            robotInfo->setForkWidthCmd(n_width);
        }
    }
}

// void NcNaviCoreNoetic::onPlcServerReceive(const std_msgs::String::ConstPtr msg)
//{
// try {
//     if (msg->data.size() == 0)
//         return;
//     auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
//     robotInfo->updateJigInfo(msg);
//
//     Poco::JSON::Parser parser;
//     Poco::Dynamic::Var result = parser.parse(msg->data);
//     Poco::JSON::Object::Ptr message = result.extract<Poco::JSON::Object::Ptr>();
//
//     message->set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
//     message->set("in_progress", false);
//
//     if (message->has("errors")) {
//         Poco::JSON::Array::Ptr errorMessageArry = message->getArray("errors");
//         for (int i = 0; i < errorMessageArry->size(); i++) {
//             Poco::JSON::Object::Ptr obj = errorMessageArry->getObject(i);
//             std::string strPcDeviceTag = obj->get("pc_device_tag").convert<std::string>();
//             std::string strPlcDeviceStatus = obj->get("plc_device_status").convert<std::string>();
//
//             Poco::JSON::Object plc_logs_create;
//             plc_logs_create.set("pc_device_tag", strPcDeviceTag);
//             plc_logs_create.set("plc_device_status", strPlcDeviceStatus);
//
//             std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res = apiClient_->post(
//                 "/plc-logs/create", plc_logs_create,
//                 InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken());
//
//             std::string is = std::get<0>(res);
//             Poco::Net::HTTPResponse::HTTPStatus status = std::get<1>(res);
//             std::string reason = std::get<2>(res);
//
//             if (status != Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
//                 LOG_INFO("/plc-logs/create Respone : %d", (int)status);
//             }
//         }
//     }
//
//     std::ostringstream oss;
//     message->stringify(oss);
//
//     DefaultPublisher::instance().publisher->publish(
//         NcBrainMessage::MESSAGE_ROBOT_STATUS_EXD + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
//         oss.str());
// }
// catch (std::exception ex) {
//     LOG_ERROR("%s", ex.what());
// }
//}

// void NcNaviCoreNoetic::onPlcServerRemoteCommand(const std_msgs::String::ConstPtr msg)
//{
// try {
//    Poco::JSON::Parser parser;
//    Poco::Dynamic::Var result = parser.parse(msg->data);
//    Poco::JSON::Object::Ptr message = result.extract<Poco::JSON::Object::Ptr>();

//    setControlMsg(0.0, 0.0);
//    std::string command = message->getValue<std::string>("plcCommand");

//    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

//    if (command == "plc_remote_Auto") {
//        isRobotManualMode_ = false;
//        robotInfo->setIsManual(isRobotManualMode_);
//    }
//    else if (command == "plc_remote_Manual") {
//        isRobotManualMode_ = true;
//        robotInfo->setIsManual(isRobotManualMode_);
//        cancelTask();
//    }
//    else if (command == "plc_remote_NextSW" && !isRobotManualMode_) {
//        // if (agentTaskManager_->isProcessing() == false) {
//        //    agentTaskManager_->process_event(Events::ResponseTaskAlarm{"manual", "done"});
//        //}
//    }
//    else if (command == "plc_remote_preset_done" && !isRobotManualMode_) {
//        NcTaskAlarm alarm(
//            "preset", "task_alarm", "task_manager", "done",
//            InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
//        DefaultPublisher::instance().publisher->publish(NcBrainMessage::MESSAGE_ACS_REQUEST, alarm.toString());
//        // agentTaskManager_->process_event(Events::ResponseTaskAlarm{"preset", "done"});
//    }
//    else if (command == "plc_remote_RepairSW" && !isRobotManualMode_) {
//        // if (agentTaskManager_->isProcessing() == false) {
//        //    agentTaskManager_->process_event(Events::ResponseTaskAlarm{"manual", "repair"});
//        //}
//    }

//    LOG_INFO("%s - Call", command.c_str());
//}
// catch (std::exception ex) {
//    LOG_ERROR("%s", ex.what());
//}
//}