#include "core_agent/core_agent.h"

#include <Poco/JSON/Object.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_collision_info.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_pose.h>
#include <core_agent/data/robot_wheel_info.h>
#include <tf/transform_datatypes.h>

using namespace NaviFra;

using Poco::JSON::Array;
using Poco::JSON::Object;

const std::string RobotInfo::KEY = "RobotInfo";

RobotInfo::RobotInfo()
{
    id_ = "";
    robot_status_ = "";
    robot_alarm_ = "";

    //추가
    robot_pose_x_ = 0.0;
    robot_pose_y_ = 0.0;
    robot_pose_deg_ = 0.0;

    f_max_linear_vel_of_path_ = 0.0;
    reset_cmd_ = false;
    batterystartcmd_ = 0;

    liftcmd_ = 0;
    piocmd_ = 0;

    alarm_ = "0";

    battery_ = 100;

    robot_linear_velocity_x_ = 0.0;
    robot_linear_velocity_y_ = 0.0;
    robot_linear_velocity_z_ = 0.0;
    robot_angular_velocity_ = 0;
    confidence_ = 0.0;

    now_node_ = "0";
    next_node_ = "0";
    goal_node_ = "0";
    path_progress_ = 0;

    now_task_ = "";
    is_load_ = "";
    robot_size_.width = 20;
    robot_size_.height = 20;
    setting_.map = 0;
    setting_.totalNodes = 0;
    setting_.teachedNodes = 0;

    robot_wheelpos = 0.f;
    robot_wheeloffset = 0.f;
    error_dist = 0.f;
    ossd_field_ = 0;
    light_ = false;
    charging_state_ = false;

    jiginfo_.nRotation = 0;
    jiginfo_.nLift = 0;
    jiginfo_.bClamp = false;
    jiginfo_.bProduct = false;
    jiginfo_.error = false;
    jiginfo_.communication = false;

    tasks_.current_task_id = "";
    tasks_.previous_task_id = "";
    map_.current_area_id = "";
    map_.current_map_id = "";
    map_.area = {};

    isrobotManual_ = false;

    errortable_ = {"ERROR_LOCALIZATION_MATCHING", "emergency", "paused_by_user", "paused", "paused_by_obs", "paused_by_path"};
}

RobotInfo::~RobotInfo()
{
}

void RobotInfo::updateRobotInfo(const Velocity& velocity, const NodePath& path, double confidence)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);

    robot_linear_velocity_x_ = velocity.linear_x;
    robot_linear_velocity_y_ = velocity.linear_y;
    robot_angular_velocity_ = velocity.angular;

    now_node_ = path.now_node;
    next_node_ = path.next_node;
    path_progress_ = path.path_progress;
    goal_node_ = path.goal_node;

    confidence_ = confidence;
}

void RobotInfo::updateFootprint(const float& x, const float& y)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    if (robot_footprint_.size() == 0 && x != 0.0 && y != 0.0) {
        lastPosition_.x = x;
        lastPosition_.y = y;
        Poco::JSON::Array path;
        path.add(round(lastPosition_.x * 100) / 100);
        path.add(round(lastPosition_.y * 100) / 100);
        path.add(0.f);
        robot_footprint_.add(path);
    }

    if (robot_footprint_.size() >= 100) {
        robot_footprint_.remove(0);
    }

    double distance = hypot(x - lastPosition_.x, y - lastPosition_.y);
    if (distance >= 0.2) {
        lastPosition_.x = x;
        lastPosition_.y = y;
        Poco::JSON::Array path;
        path.add(round(lastPosition_.x * 100) / 100);
        path.add(round(lastPosition_.y * 100) / 100);
        path.add(0.f);
        robot_footprint_.add(path);
    }
}

void RobotInfo::updateErrorPose(const float& x, const float& y, const float& deg)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    errorPose_.x = x;
    errorPose_.y = y;
    errorPose_.deg = isnan(deg) ? 0 : deg;
}

void RobotInfo::updateJob(const std::string& current_task_id, const std::string& previous_task_id)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    tasks_.current_task_id = current_task_id;
    tasks_.previous_task_id = previous_task_id;
}

// void RobotInfo::updateMap(const float& x, const float& y, const float& deg)
// {
//     Poco::FastMutex::ScopedLock lock(fastMutex_);
//     errorPose_.x = x;
//     errorPose_.y = y;
//     errorPose_.deg = isnan(deg) ? 0 : deg;
// }

// void RobotInfo::updateRobotStatus(const geometry_msgs::PoseWithCovarianceStamped msg)
//{
//    Poco::FastMutex::ScopedLock lock(fastMutex_);
//    robot_pose_.position.x = msg.pose.pose.position.x;
//    robot_pose_.position.y = msg.pose.pose.position.y;
//    robot_pose_.orientation.x = msg.pose.pose.orientation.x;
//    robot_pose_.orientation.y = msg.pose.pose.orientation.y;
//    robot_pose_.orientation.z = msg.pose.pose.orientation.z;
//    robot_pose_.orientation.w = msg.pose.pose.orientation.w;
//}

std::string RobotInfo::toString()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    Object data, robot_size, setting, jiginfo, errors, errorPose, tasks, areas, maps, map;

    data.set("id", id_);
    data.set("robot_status", robot_status_);
    data.set("robot_alarm", "");
    data.set("battery", battery_);
    data.set("charging", charging_state_);

    errors.set("dist", error_dist);
    errorPose.set("x", errorPose_.x);
    errorPose.set("y", errorPose_.y);
    errorPose.set("deg", errorPose_.deg);
    errors.set("position", errorPose);
    data.set("errors", errors);

    data.set("robot_pose", InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY)->toObject());

    Poco::JSON::Array a_speed;
    a_speed.add(robot_linear_velocity_x_);
    a_speed.add(robot_linear_velocity_y_);

    data.set("robot_linear_velocity", a_speed);
    data.set("robot_angular_velocity", robot_angular_velocity_);
    data.set("confidence", confidence_);
    data.set("now_node", now_node_);
    data.set("next_node", next_node_);
    data.set("goal_node", goal_node_);
    data.set("path_progress", path_progress_);
    data.set("now_task", now_task_);
    data.set("is_load", is_load_);

    robot_size.set("width", robot_size_.width);
    robot_size.set("height", robot_size_.height);
    data.set("robot_size", robot_size);
    data.set("robot_base", robot_wheelpos);

    auto collision = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);

    data.set("robot_shape", collision->getShape());
    data.set("robot_collision", collision->getCollision());
    data.set("robot_footprint", robot_footprint_);
    data.set("robot_wheels", InMemoryRepository::instance().get<RobotWheelInfoStore>(RobotWheelInfoStore::KEY)->toObject());
    
    map.set("current_area_id", map_.current_area_id);
    map.set("current_map_id", map_.current_map_id);
    data.set("map", map);

    setting.set("map", setting_.map);
    setting.set("total_nodes", setting_.totalNodes);
    setting.set("teached_nodes", setting_.teachedNodes);
    data.set("setting", setting);

    jiginfo.set("rotation", jiginfo_.nRotation);
    jiginfo.set("lift", jiginfo_.nLift);
    jiginfo.set("clamp", jiginfo_.bClamp);
    jiginfo.set("hold", jiginfo_.bHold);
    jiginfo.set("product", jiginfo_.bProduct);
    jiginfo.set("error", jiginfo_.error);
    jiginfo.set("communication", jiginfo_.communication);
    data.set("jig_status", jiginfo);

    data.set("manual", isrobotManual_);
    data.set("barcode_reader", bardcode_reader_);

    tasks.set("current_task_id", tasks_.current_task_id);
    tasks.set("previous_task_id", tasks_.previous_task_id);
    data.set("tasks", tasks);

    // area 정보 추가
    for(int i = 0; i < vec_area_uuid_.size(); i++) {
        maps.set(vec_area_uuid_[i], vec_map_uuid_[i]);
    }
    areas.set("maps", maps);
    areas.set("id", s_current_area_id_);
    data.set("area", areas);
    std::ostringstream ostr;
    data.stringify(ostr);

    return ostr.str();
}

std::string RobotInfo::toString(
    std::shared_ptr<RobotCollisionInfo> collision, std::shared_ptr<RobotWheelInfoStore> wheelInfoStore, std::shared_ptr<RobotPose> pose)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    Object data, robot_size, setting, jiginfo, errors, errorPose, areas, maps;

    data.set("id", id_);
    data.set("robot_status", robot_status_);
    data.set("robot_alarm", "");
    data.set("battery", battery_);

    errors.set("dist", error_dist);
    errorPose.set("x", errorPose_.x);
    errorPose.set("y", errorPose_.y);
    errorPose.set("deg", errorPose_.deg);
    errors.set("position", errorPose);
    data.set("errors", errors);

    data.set("robot_pose", pose->toObject());

    Poco::JSON::Array a_speed;
    a_speed.add(robot_linear_velocity_x_);
    a_speed.add(robot_linear_velocity_y_);

    data.set("robot_linear_velocity", a_speed);
    data.set("robot_angular_velocity", robot_angular_velocity_);
    data.set("confidence", confidence_);
    data.set("now_node", now_node_);
    data.set("next_node", next_node_);
    data.set("goal_node", goal_node_);
    data.set("path_progress", path_progress_);
    data.set("now_task", now_task_);
    data.set("is_load", is_load_);

    robot_size.set("width", robot_size_.width);
    robot_size.set("height", robot_size_.height);

    data.set("robot_size", robot_size);
    data.set("robot_base", robot_wheelpos);
    data.set("robot_shape", collision->getShape());
    data.set("robot_collision", collision->getCollision());
    data.set("robot_footprint", robot_footprint_);
    data.set("robot_wheels", wheelInfoStore->toObject());

    setting.set("map", setting_.map);
    setting.set("total_nodes", setting_.totalNodes);
    setting.set("teached_nodes", setting_.teachedNodes);
    data.set("setting", setting);

    jiginfo.set("rotation", jiginfo_.nRotation);
    jiginfo.set("lift", jiginfo_.nLift);
    jiginfo.set("clamp", jiginfo_.bClamp);
    jiginfo.set("hold", jiginfo_.bHold);
    jiginfo.set("product", jiginfo_.bProduct);
    jiginfo.set("error", jiginfo_.error);
    jiginfo.set("communication", jiginfo_.communication);
    data.set("jig_status", jiginfo);

    data.set("manual", isrobotManual_);
    data.set("barcode_reader", bardcode_reader_);

    // area 정보 추가
    for(int i = 0; i < vec_area_uuid_.size(); i++) {
        maps.set(vec_area_uuid_[i], vec_map_uuid_[i]);
    }
    areas.set("maps", maps);
    areas.set("id", s_current_area_id_);
    data.set("area", areas);

    std::ostringstream ostr;
    data.stringify(ostr);

    return ostr.str();
}

void RobotInfo::setID(std::string id)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    id_ = id;
}

void RobotInfo::setSetting(int map, int totalNodes, int teachedNodes)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    setting_.map = map;
    setting_.totalNodes = totalNodes;
    setting_.teachedNodes = teachedNodes;
}

void RobotInfo::setArea(std::string s_current_area_id, std::vector<std::string> vec_area_uuid, std::vector<std::string> vec_map_uuid)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    s_current_area_id_ = s_current_area_id;
    vec_area_uuid_ = vec_area_uuid;
    vec_map_uuid_ = vec_map_uuid;
}

void RobotInfo::setTeachedNodes(int teachedNodes)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    setting_.teachedNodes = teachedNodes;
}

void RobotInfo::setStatus(std::string status)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    robot_status_ = status;
}

void RobotInfo::setErrordist(float dist)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    error_dist = dist;
}

void RobotInfo::updateRobotBase(const float wheel, const float offset)
{
    robot_wheelpos = wheel;
    robot_wheeloffset = offset;
}

void RobotInfo::updateVelocity(const double& linear_x, const double& linear_y, const double& angular)
{
    robot_linear_velocity_x_ = linear_x;
    robot_linear_velocity_y_ = linear_y;
    robot_angular_velocity_ = angular * 180 / 3.141592;
}

void RobotInfo::updateTargetVelocity(const double& linear_x, const double& linear_y, const double& angular)
{
    robot_target_linear_velocity_x_ = linear_x;
    robot_target_linear_velocity_y_ = linear_y;
    robot_target_angular_velocity_ = angular * 180 / 3.141592;
}

// void RobotInfo::updateJigInfo(const std_msgs::String::ConstPtr& msg)
//{
//    try {
//        Poco::FastMutex::ScopedLock lock(fastMutex_);//

//        Poco::JSON::Parser parser;
//        Poco::Dynamic::Var result = parser.parse(msg->data);
//        Poco::JSON::Object::Ptr message = result.extract<Poco::JSON::Object::Ptr>();
//        if (message->has("pose") == false)
//            return;//

//        Poco::JSON::Object::Ptr poseObject = message->getObject("pose");
//        jiginfo_.nRotation = poseObject->getValue<int>("rotation");
//        jiginfo_.nLift = poseObject->getValue<int>("lift");
//        jiginfo_.bClamp = poseObject->getValue<bool>("clamp1");
//        jiginfo_.bHold = poseObject->getValue<bool>("hold");
//        jiginfo_.bProduct = poseObject->getValue<bool>("product");
//        jiginfo_.error = poseObject->getValue<bool>("error");
//        jiginfo_.communication = poseObject->getValue<bool>("communication");
//    }
//    catch (std::exception ex) {
//        LOG_ERROR("%s", ex.what());
//    }
//}

void RobotInfo::setIsManual(bool isManual)
{
    isrobotManual_ = isManual;
}

void RobotInfo::setRobotDrivingInfo(bool isGoalStart, bool isGoalComplete, bool robotError, bool isMoving)
{
    robotDrivingInfo_.isGoalStart = isGoalStart;
    robotDrivingInfo_.isGoalComplete = isGoalComplete;
    robotDrivingInfo_.robotError = robotError;
    robotDrivingInfo_.isMoving = isMoving;
}

void RobotInfo::setMoveToGoal()
{
    robotDrivingInfo_.isGoalStart = true;
    robotDrivingInfo_.isGoalComplete = false;
}
void RobotInfo::setGoalArrived()
{
    robotDrivingInfo_.isGoalComplete = true;
    robotDrivingInfo_.isGoalStart = false;
}

std::string RobotInfo::getRobotDrivingInfoToString()
{
    Object objMainMessage, objRobotDrivingInfoMessage;
    objRobotDrivingInfoMessage.set("goalStart", robotDrivingInfo_.isGoalStart);
    objRobotDrivingInfoMessage.set("goalComplete", robotDrivingInfo_.isGoalComplete);

    robotDrivingInfo_.robotError = std::find(errortable_.begin(), errortable_.end(), robot_status_) != errortable_.end() ? true : false;
    // Status 보고
    objRobotDrivingInfoMessage.set("robotError", robotDrivingInfo_.robotError);

    // Motor Info
    // objRobotDrivingInfoMessage.set("isMoving",robotDrivingInfo_.isMoving);

    objMainMessage.set("robotDrivingInfo", objRobotDrivingInfoMessage);

    std::ostringstream ostr;
    objMainMessage.stringify(ostr);

    return ostr.str();
}

void RobotInfo::setRobotType(std::string type)
{
    strRobottype_ = type;
}

std::string RobotInfo::getRobotType()
{
    return strRobottype_;
}

void RobotInfo::setBarcodeOn(bool bOnOff)
{
    bardcode_reader_ = bOnOff;
}
void RobotInfo::setBattery(float battery)
{
    battery_ = battery;
}

void RobotInfo::setBatteryVol(float battery_vol)
{
    battery_vol_ = battery_vol;
}

// forklift 함수 추가

void RobotInfo::setRobotPoseX(float robot_pose_x)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    robot_pose_x_ = robot_pose_x * 1000;
}

void RobotInfo::setRobotPoseY(float robot_pose_y)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    robot_pose_y_ = robot_pose_y * 1000;
}

void RobotInfo::setRobotPoseDeg(float robot_pose_deg)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    robot_pose_deg_ = robot_pose_deg * 100;
}

void RobotInfo::setRobotLinearVelocityZ(float robot_linear_velocity_z)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    robot_linear_velocity_z_ = robot_linear_velocity_z;
}

void RobotInfo::setRobotPathSpeed(float f_max_linear_vel_of_path)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    f_max_linear_vel_of_path_ = f_max_linear_vel_of_path;
}

void RobotInfo::setMotorError(bool reset_cmd)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    reset_cmd_ = reset_cmd;
}

void RobotInfo::setAlarm(std::string alarm)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    alarm_ = alarm;
}

void RobotInfo::setBatteryCmd(int16_t batterystartcmd)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    batterystartcmd_ = batterystartcmd;
}

void RobotInfo::setForkUpDownCmd(int16_t fork_up_down_cmd)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    fork_up_down_cmd_ = fork_up_down_cmd;
}

void RobotInfo::setForkUpDownPosition(int16_t fork_up_down_position)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    fork_up_down_position_ = fork_up_down_position;
}

void RobotInfo::setTiltingUpDownCmd(int16_t tilting_up_down_cmd)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    tilting_up_down_cmd_ = tilting_up_down_cmd;
}

void RobotInfo::setForkWidthCmd(int16_t fork_width_cmd)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    fork_width_cmd_ = fork_width_cmd;
}

void RobotInfo::setLiftCmd(int16_t liftcmd)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    liftcmd_ = liftcmd;
}

void RobotInfo::setPioCmd(int64_t piocmd)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    piocmd_ = piocmd;
}
void RobotInfo::setChargingState(bool charging_state)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    charging_state_ = charging_state;
}
void RobotInfo::setAlarmDescription(std::string alarm)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    AlarmDescription_ = alarm;
}

void RobotInfo::setAlarmId(int16_t alarm_id)
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    Alarmid_ = alarm_id;
}

void RobotInfo::setNowTask(std::string now_task)
{
    now_task_ = now_task;
}

void RobotInfo::setWorkID(std::string work_id)
{
    work_id_ = work_id;
}

void RobotInfo::setLight(bool light)
{
    light_ = light;
}

void RobotInfo::setOSSDField(int16_t ossd_field)
{
    ossd_field_ = ossd_field;
}

void RobotInfo::setForkLiftMax(int lift_max_dist)
{
    lift_max_dist_= lift_max_dist;
}

void RobotInfo::setForkLiftMin(int lift_min_dist)
{
    lift_min_dist_= lift_min_dist;
}

void RobotInfo::setPalletID(std::string pallet_id)
{
    pallet_id_ = pallet_id;
}

// get 함수 추가
int16_t RobotInfo::getRobotPoseX()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(robot_pose_x_);
}

int16_t RobotInfo::getRobotPoseY()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(robot_pose_y_);
}

int16_t RobotInfo::getRobotPoseDeg()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(robot_pose_deg_);
}

int16_t RobotInfo::getConfidence()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(confidence_);
}

int16_t RobotInfo::getNowNode()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);

    std::regex number_regex("^[0-9]+$");
    if (std::regex_match(now_node_, number_regex)) {
        try {
            return static_cast<int16_t>(std::stoi(now_node_));
        }
        catch (const std::exception& e) {
            return -1;
        }
    }
    else {
        return -1;
    }
}

int16_t RobotInfo::getNextNode()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(std::stoi(next_node_));
}

int16_t RobotInfo::getGoalNode()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);

    std::regex number_regex("^[0-9]+$");
    if (std::regex_match(goal_node_, number_regex)) {
        try {
            return static_cast<int16_t>(std::stoi(goal_node_));
        }
        catch (const std::exception& e) {
            return -1;
        }
    }
    else {
        return -1;
    }
}

int16_t RobotInfo::getTwistLinearX()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(robot_linear_velocity_x_);
}

int16_t RobotInfo::getTwistLinearZ()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(robot_linear_velocity_z_);
}

int16_t RobotInfo::getRobotPathSpeed()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(f_max_linear_vel_of_path_);
}

int16_t RobotInfo::getAlarm()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(std::stoi(alarm_));
}

int16_t RobotInfo::getMotorError()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(reset_cmd_);
}

int16_t RobotInfo::getBatteryCmd()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(batterystartcmd_);
}

int16_t RobotInfo::getForkUpDownCmd()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(fork_up_down_cmd_);
}

int16_t RobotInfo::getForkUpDownPosition()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(fork_up_down_position_);
}
int16_t RobotInfo::getTiltingUpDownCmd()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(tilting_up_down_cmd_);
}
int16_t RobotInfo::getForkWidthCmd()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(fork_width_cmd_);
}

int16_t RobotInfo::getLiftCmd()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(liftcmd_);
}

int16_t RobotInfo::getPioCmd()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(piocmd_);
}

std::string RobotInfo::getStatus()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    // return static_cast<
    std::string status = robot_status_;
    return status;
    // robot_status_ = status;
}

bool RobotInfo::getIsManual()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return isrobotManual_;
}


std::string RobotInfo::getAlarmDescription()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    std::string alarmDescription = AlarmDescription_;
    return alarmDescription;
}

int16_t RobotInfo::getAlarmId()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(Alarmid_);
}

int16_t RobotInfo::getLight()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(light_);
}

int16_t RobotInfo::getOSSDField()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(ossd_field_);
}

int16_t RobotInfo::getForkLiftMax()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(lift_max_dist_);
}

int16_t RobotInfo::getForkLiftMin()
{
    Poco::FastMutex::ScopedLock lock(fastMutex_);
    return static_cast<int16_t>(lift_min_dist_);
}