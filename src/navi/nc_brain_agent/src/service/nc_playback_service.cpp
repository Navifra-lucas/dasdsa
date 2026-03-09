#include "nc_brain_agent/nc_brain_agent.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <Poco/Mutex.h>
#include <Poco/ScopedLock.h>
#include <Poco/UUIDGenerator.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/service/nc_playback_service.h>
#include <nc_brain_agent/utils/nc_agent_config.h>
#include <tf/transform_datatypes.h>

using namespace NaviFra;

NcPlaybackService::NcPlaybackService()
{
}

NcPlaybackService::~NcPlaybackService()
{
    isRunning_ = false;
    robotInfo_.reset();
    robotWheelInfoStore_.reset();
    robotCollisionInfo_.reset();
}

void NcPlaybackService::initialize(ros::NodeHandle& nh)
{
    robotInfo_.reset(new RobotInfo());

    /*
        Robot Info에서 사용 하는 Wheel 과 Collision의 경우 전역 Etity Manager에서 값을 복사 해서 써야 한다.
        전역데이터는 실시간 데이터고 리플레이도 보게 될 데이터는 이전 데이터의 저장 본이기 때문에 전역 데이터를 사용하는 robot_info의
       toString을 사용하지 않고 toString(collision, wheelinfo) 함수를 사용 하자
    */

    robotInfo_->setID(Config::instance().getString("robot_id", "5000"));

    robotWheelInfoStore_ =
        std::make_shared<RobotWheelInfoStore>(*InMemoryRepository::instance().get<RobotWheelInfoStore>(RobotWheelInfoStore::KEY));
    robotCollisionInfo_ = std::make_shared<RobotCollisionInfo>();

    robotPose_.reset(new RobotPose());

    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/front_cloud_global", 1, &NcPlaybackService::onFrontCloud, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/rear_cloud_global", 1, &NcPlaybackService::onRearCloud, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/left_cloud_global", 1, &NcPlaybackService::onLeftCloud, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/right_cloud_global", 1, &NcPlaybackService::onRightCloud, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/v2v_cloud_global", 1, &NcPlaybackService::onV2VCloud, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/camera_cloud_global", 1, &NcPlaybackService::onCameraCloud, this));
    // rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/obs_cloud_global", 1, &NcPlaybackService::onObsCloud, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/motor_info", 1, &NcPlaybackService::onMotorInfo, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/navifra/info", 1, &NcPlaybackService::onNavifrainfo, this));

    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/NaviFra/visualize/ui_local_path", 1, &NcPlaybackService::onLocalPath, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/NaviFra/visualize/ui_global_path", 1, &NcPlaybackService::onGlobalPath, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/play_time", 1, &NcPlaybackService::onTime, this));

    rosSubscriber_.push_back(
        nh.subscribe("/PLAYBACK/NaviFra/visualize/robot_collision_predict", 1, &NcPlaybackService::onPredictCollision, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/NaviFra/visualize/robot_collision", 1, &NcPlaybackService::onCollision, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/NaviFra/visualize/ui_obstacle_pos", 1, &NcPlaybackService::onObsPos, this));
    rosSubscriber_.push_back(nh.subscribe("/PLAYBACK/etc_custom_message", 1, &NcPlaybackService::onCustomMessgae, this));
}

void NcPlaybackService::run()
{
    while (isRunning_) {
        try {
            if (predict_polygon_.size() == 0) {
                std::string json(Poco::format(
                    "{\"id\":\"%s\",\"polygon_name\":\"predict_collision\",\"points\":\"[]\"}",
                    Config::instance().getString("robot_id", "5000")));
                predict_polygon_ = json;
            }
            if (obs_polygon_.size() == 0) {
                std::string json(Poco::format(
                    "{\"id\":\"%s\",\"polygon_name\":\"obstacle_pos\",\"points\":\"[]\"}",
                    Config::instance().getString("robot_id", "5000")));
                obs_polygon_ = json;
            }

            std::string info(Poco::format(
                "{\"id\":\"%s\",\"position\":\"%s\",\"info\":%s,\"polygon\":%s,\"message\":%s}",
                Config::instance().getString("robot_id", "5000"), playTime_,
                robotInfo_->toString(robotCollisionInfo_, robotWheelInfoStore_, robotPose_), predict_polygon_, etc_message_));

            MessageBroker::instance().publish(
                NcBrainMessage::MESSAGE_ROBOT_STATUS_PLAYBACK + Config::instance().getString("robot_id", "5000"), info);
            sendPointCloud();

        }
        catch (const Poco::Exception& ex) {
            NLOG(error) << ex.displayText();
        }
        Poco::Thread::sleep(100);

    };
}

void NcPlaybackService::onFrontCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    std::string val;
    val.append("[");
    size_t count = 0;
    sensor_msgs::PointCloud local_point;
    local_point.header = msg->header;
    local_point.points = msg->points;
    local_point.channels = msg->channels;

    // auto global_front_point = ToGlobal(local_point, current_pose_);
    // for (const auto& point : global_front_point.points) {
    for (const auto& point : local_point.points) {
        /// auto golobalPT = toGlobal(point, robo)
        if (count == 0) {
            std::string points;
            Poco::format(points, "[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        else {
            std::string points;
            Poco::format(points, ",[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        count++;
    }
    val.append("]");

    {
        Poco::ScopedLock lock(mutex_);
        frontCloud_ = val;
    }
    return;
}

void NcPlaybackService::onRearCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    std::string val;
    val.append("[");
    size_t count = 0;
    sensor_msgs::PointCloud local_point;
    local_point.header = msg->header;
    local_point.points = msg->points;
    local_point.channels = msg->channels;

    for (const auto& point : local_point.points) {
        if (count == 0) {
            std::string points;
            Poco::format(points, "[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        else {
            std::string points;
            Poco::format(points, ",[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        count++;
    }
    val.append("]");

    {
        Poco::ScopedLock lock(mutex_);
        rearCloud_ = val;
    }

    return;
}

void NcPlaybackService::onLeftCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    std::string val;
    val.append("[");
    size_t count = 0;
    sensor_msgs::PointCloud local_point;
    local_point.header = msg->header;
    local_point.points = msg->points;
    local_point.channels = msg->channels;

    for (const auto& point : local_point.points) {
        if (count == 0) {
            std::string points;
            Poco::format(points, "[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        else {
            std::string points;
            Poco::format(points, ",[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        count++;
    }
    val.append("]");

    {
        Poco::ScopedLock lock(mutex_);
        leftCloud_ = val;
    }

    return;
}

void NcPlaybackService::onRightCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    std::string val;
    val.append("[");
    size_t count = 0;
    sensor_msgs::PointCloud local_point;
    local_point.header = msg->header;
    local_point.points = msg->points;
    local_point.channels = msg->channels;

    for (const auto& point : local_point.points) {
        if (count == 0) {
            std::string points;
            Poco::format(points, "[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        else {
            std::string points;
            Poco::format(points, ",[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        count++;
    }
    val.append("]");

    {
        Poco::ScopedLock lock(mutex_);
        rightCloud_ = val;
    }

    return;
}

void NcPlaybackService::onV2VCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    std::string val;
    val.append("[");
    size_t count = 0;
    sensor_msgs::PointCloud local_point;
    local_point.header = msg->header;
    local_point.points = msg->points;
    local_point.channels = msg->channels;

    // auto global_right_point = ToGlobal(local_point, current_pose_);
    // for (const auto& point : global_right_point.points) {
    for (const auto& point : local_point.points) {
        if (count == 0) {
            std::string points;
            Poco::format(points, "[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        else {
            std::string points;
            Poco::format(points, ",[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        count++;
    }
    val.append("]");

    {
        Poco::ScopedLock lock(mutex_);
        v2vCloud_ = val;
    }

    return;
}

void NcPlaybackService::onCameraCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    std::string val;
    val.append("[");
    size_t count = 0;
    sensor_msgs::PointCloud local_point;
    local_point.header = msg->header;
    local_point.points = msg->points;
    local_point.channels = msg->channels;

    for (const auto& point : local_point.points) {
        if (count == 0) {
            std::string points;
            Poco::format(points, "[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        else {
            std::string points;
            Poco::format(points, ",[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        count++;
    }
    val.append("]");

    {
        Poco::ScopedLock lock(mutex_);
        cameraCloud_ = val;
    }

    return;
}

void NcPlaybackService::onObsCloud(const sensor_msgs::PointCloud::ConstPtr msg)
{
    std::string val;
    val.append("[");
    size_t count = 0;
    sensor_msgs::PointCloud local_point;
    local_point.header = msg->header;
    local_point.points = msg->points;
    local_point.channels = msg->channels;

    for (const auto& point : local_point.points) {
        if (count == 0) {
            std::string points;
            Poco::format(points, "[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        else {
            std::string points;
            Poco::format(points, ",[%.3hf,%.3hf,0]", point.x, point.y);
            val.append(points);
        }
        count++;
    }
    val.append("]");

    {
        Poco::ScopedLock lock(mutex_);
        obsCloud_ = val;
    }

    return;
}

sensor_msgs::PointCloud NcPlaybackService::ToGlobal(sensor_msgs::PointCloud& vec_cloud, NaviFra::Pos current_pose)
{
    sensor_msgs::PointCloud msg_result = vec_cloud;

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

void NcPlaybackService::sendPointCloud()
{
    try {
        Poco::ScopedLock lock(mutex_);
        if (frontCloud_.empty()) {
            frontCloud_ = "[]";
        }

        if (rearCloud_.empty()) {
            rearCloud_ = "[]";
        }

        if (leftCloud_.empty()) {
            leftCloud_ = "[]";
        }

        if (rightCloud_.empty()) {
            rightCloud_ = "[]";
        }

        if (v2vCloud_.empty()) {
            v2vCloud_ = "[]";
        }

        if (cameraCloud_.empty()) {
            cameraCloud_ = "[]";
        }

        std::string val;
        val.append("[");
        val.append(frontCloud_);
        val.append(",");
        val.append(rearCloud_);
        val.append(",");
        val.append(leftCloud_);
        val.append(",");
        val.append(rightCloud_);
        val.append(",");
        val.append(v2vCloud_);
        val.append(",");
        val.append(cameraCloud_);
        val.append("]");

        std::string json(Poco::format("{\"id\":\"%s\",\"scan\":{\"points\":%s}}", Config::instance().getString("robot_id", "5000"), val));

        MessageBroker::instance().publish(
            NcBrainMessage::MESSAGE_ROBOT_STATUS_PLAYBACK + Config::instance().getString("robot_id", "5000"), json);
    }
    catch (std::exception ex) {
        NLOG(error) << "updatePointCloud Exception";
    }
}

void NcPlaybackService::onNavifrainfo(const core_msgs::NavicoreStatus msg)
{
    robotInfo_->updateRobotInfo(
        {msg.f_robot_current_linear_vel_x, msg.f_robot_current_linear_vel_y, msg.f_robot_current_angular_vel_w},
        {msg.s_current_node_id, msg.s_next_node_id, msg.s_goal_node_id, msg.f_path_progress}, msg.n_confidence);

    robotInfo_->updateFootprint(msg.f_robot_pos_x_m, msg.f_robot_pos_y_m);
    robotInfo_->updateErrorPose(msg.f_error_pos_x_m, msg.f_error_pos_y_m, msg.f_error_pos_deg);
    robotInfo_->setStatus(msg.s_status);

    float robotOrientationDeg = msg.f_robot_pos_deg;
    tf::Quaternion q;
    q.setRPY(0, 0, robotOrientationDeg * 3.141592 / 180);
    std::vector<float> robotOrientationQuat = {0, 0, 0, 0};

    Position position;
    Orientation orientation;

    position.x = isnan(msg.f_robot_pos_x_m) ? 0 : msg.f_robot_pos_x_m;
    position.y = isnan(msg.f_robot_pos_y_m) ? 0 : msg.f_robot_pos_y_m;
    orientation.x = isnan(q.getX()) ? 0 : q.getX();
    orientation.y = isnan(q.getY()) ? 0 : q.getY();
    orientation.z = isnan(q.getZ()) ? 0 : q.getZ();
    orientation.w = isnan(q.getW()) ? 0 : q.getW();

    robotPose_->update(position, orientation);
}

void NcPlaybackService::onMotorInfo(const core_msgs::MotorInfo::ConstPtr msg)
{
    robotWheelInfoStore_->updateWheelInfo(msg);
}

void NcPlaybackService::onRobotPos(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    // robotInfo_->updateRobotStatus(msg);
    // current_pose_.SetXm(robotInfo_->getPose().position.x);
    // current_pose_.SetYm(robotInfo_->getPose().position.y);
    // tf2::Quaternion q(
    //     robotInfo_->getPose().orientation.x, robotInfo_->getPose().orientation.y, robotInfo_->getPose().orientation.z,
    //     robotInfo_->getPose().orientation.w);
    // current_pose_.SetRad(tf2::getYaw(q));
}

void NcPlaybackService::onGlobalPath(const nav_msgs::Path msg)
{
    Poco::JSON::Object message, paths;
    Poco::JSON::Array global_path;
    for (const auto& pose : msg.poses) {
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

    message.set("id", Config::instance().getString("robot_id", "5000"));
    paths.set("global_path", global_path);
    message.set("path", paths);
    std::ostringstream oss;
    message.stringify(oss);
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_PLAYBACK + Config::instance().getString("robot_id", "5000"), oss.str());
    NLOG(trace) << std::endl << oss.str();
}

void NcPlaybackService::onLocalPath(const nav_msgs::Path msg)
{
    Poco::JSON::Object message, paths;
    Poco::JSON::Array local_path;
    for (const auto& pose : msg.poses) {
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

    message.set("id", Config::instance().getString("robot_id", "5000"));
    paths.set("local_path", local_path);
    message.set("path", paths);
    std::ostringstream oss;
    message.stringify(oss);
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_PLAYBACK + Config::instance().getString("robot_id", "5000"), oss.str());
    NLOG(trace) << std::endl << oss.str();
}

void NcPlaybackService::onTime(const std_msgs::String time)
{
    playTime_ = time.data;
}

void NcPlaybackService::onObsPos(const geometry_msgs::PolygonStamped msg)
{
    static Poco::Timestamp prePubtime;
    Poco::Timestamp now;

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

    
    std::string json(Poco::format(
        "{\"id\":\"%s\",\"polygon_name\":\"obstacle_pos\",\"points\":%s}", Config::instance().getString("robot_id", "5000"), points));

    obs_polygon_ = json;
    
    
    std::string info(Poco::format(
        "{\"id\":\"%s\",\"position\":\"%s\",\"polygon\":%s}",
        Config::instance().getString("robot_id", "5000"), playTime_, obs_polygon_));


    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_PLAYBACK + Config::instance().getString("robot_id", "5000"), info);

    prePubtime.update();
}

void NcPlaybackService::onPredictCollision(const geometry_msgs::PolygonStamped msg)
{
    static Poco::Timestamp prePubtime;
    Poco::Timestamp now;
    if ((now - prePubtime) / 1000 < 50) {
        return;
    }

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

    std::string json(Poco::format(
        "{\"id\":\"%s\",\"polygon_name\":\"predict_collision\",\"points\":%s}", Config::instance().getString("robot_id", "5000"), points));

    predict_polygon_ = json;
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_PLAYBACK + Config::instance().getString("robot_id", "5000"), json);

    prePubtime.update();
}

void NcPlaybackService::onCollision(const geometry_msgs::PolygonStamped polygon)
{
    std::vector<Position> position;
    for (auto point : polygon.polygon.points) {
        position.push_back({point.x, point.y, point.z});
    }
    robotCollisionInfo_->updateCollision(position);
}

void NcPlaybackService::onCustomMessgae(const std_msgs::String::ConstPtr msg)
{
    static std::string strMessage;

    if (strMessage != msg->data) {
        strMessage = msg->data;
        Poco::JSON::Object message;

        strMessage = msg->data;
        message.set("id", Config::instance().getString("robot_id", "5000"));

        message.set("uuid", Poco::UUIDGenerator::defaultGenerator().create().toString());
        message.set("type", "info");
        message.set("message", strMessage);
        std::ostringstream oss;
        message.stringify(oss);
        etc_message_ = oss.str();
    }
}