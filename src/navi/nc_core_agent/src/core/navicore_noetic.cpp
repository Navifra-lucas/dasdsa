#include "core_agent/core_agent.h"

#include <Poco/Dynamic/Var.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Parser.h>
#include <core_agent/core/navicore_command.h>
#include <core_agent/core/navicore_message.h>
#include <core_agent/core/navicore_noetic.h>
#include <core_agent/data/lidar_merger.h>
#include <core_agent/data/memory_repository.h>
#include <core_msgs/Vehicle.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace fs = std::filesystem;  // alias 선언
using namespace NaviFra;

NaviCoreNoetic::NaviCoreNoetic()
    : isSLAM_(false)
    , isUseLidar(true)
{
    ros::NodeHandle nodeHandler_;
    rosPubs_[CoreCommand::ROBOT_INIT_POSE] = nodeHandler_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    rosPubs_[CoreCommand::ROBOT_CORRECTION_INIT_POSE] = nodeHandler_.advertise<std_msgs::String>("correction_init_pos", 1);
    // rosPubs_[CoreCommand::ROBOT_DEVICE_GOAL] = nodeHandler_.advertise<core_msgs::Goal>("navifra/goal_id", 1);
    rosPubs_[CoreCommand::ROBOT_DEVICE_COMMAND] = nodeHandler_.advertise<std_msgs::String>("nc_task_manager/task_cmd", 1);
    // rosPubs_[CoreCommand::ROBOT_MOVE] = nodeHandler_.advertise<core_msgs::Goal>("navifra/goal_id", 1);
    rosPubs_[CoreCommand::ROBOT_STOP] = nodeHandler_.advertise<std_msgs::String>("nc_task_manager/task_cmd", 1);
    rosPubs_[CoreCommand::TASK_COMMAND] = nodeHandler_.advertise<std_msgs::String>("nc_task_manager/task_cmd", 1);
    rosPubs_[CoreCommand::ROBOT_SAVE_SLAM] = nodeHandler_.advertise<std_msgs::Int16>("answer/map_save", 1);
    rosPubs_[CoreCommand::ROBOT_CONTROL] = nodeHandler_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    rosPubs_[CoreCommand::ROBOT_CALIBRATION] = nodeHandler_.advertise<std_msgs::String>("cali_cmd", 1);
    rosPubs_[CoreCommand::ROBOT_EXD_UPDATEROBOT_INFO] = nodeHandler_.advertise<std_msgs::String>("plcserver/update_robotinfo", 1);
    rosPubs_[CoreCommand::ROBOT_START_EXD_STATUS] = nodeHandler_.advertise<std_msgs::String>("plcserver/start_exd_status", 1);
    rosPubs_[CoreCommand::ROBOT_CALIBRATION_DOCKING_SAVE] = nodeHandler_.advertise<std_msgs::String>("vmarker/laser_calculate", 1);
    rosPubs_[CoreCommand::MAP_UPDATE] = nodeHandler_.advertise<std_msgs::String>("map_request", 10, true);

    rosPubs_[CoreCommand::ROBOT_UPDATE_PARAMETERS] = nodeHandler_.advertise<std_msgs::String>("navifra/param_update", 1);

    rosPubs_[CoreCommand::ROBOT_PLAYBACK] = nodeHandler_.advertise<std_msgs::String>("playback_cmd", 1);
    rosPubs_[CoreCommand::ROBOT_REPEAT_DRIVE] = nodeHandler_.advertise<core_msgs::RepeatTestMsg>("repeat_test", 1);
    rosPubs_[CoreCommand::ROBOT_REPEAT_DRIVE_CMD] = nodeHandler_.advertise<std_msgs::String>("repeat_test/cmd", 1);
    rosPubs_[CoreCommand::ROBOT_BARCODE_READER] = nodeHandler_.advertise<std_msgs::Bool>("navifra/bcr_onoff", 1);
    rosPubs_[CoreCommand::ROBOT_CALIBRATION_STEER_WRITE] = nodeHandler_.advertise<std_msgs::String>("/quad_cmd", 1);
    rosPubs_[CoreCommand::ROBOT_CALIBRATION_STEER_ZEROSET] = nodeHandler_.advertise<std_msgs::String>("encoder_zero_offset", 1);
    rosPubs_[CoreCommand::ROBOT_FAB_COLOR] = nodeHandler_.advertise<std_msgs::String>("fab_color", 1);
    rosPubs_[CoreCommand::ROBOT_FAB_SOUND] = nodeHandler_.advertise<core_msgs::Sound>("fab_sound", 1);

    rosPubs_[CoreCommand::ROBOT_RESPONSE_AVOIDANCE] = nodeHandler_.advertise<std_msgs::Bool>("response_avoidance", 1);
    rosPubs_[CoreCommand::ROBOT_RSR_INFO] = nodeHandler_.advertise<core_msgs::Vehicle>("rsr_info", 10);

    rosPubs_[CoreCommand::ROBOT_CHEONIL_READ_REGISTER] = nodeHandler_.advertise<core_msgs::CheonilReadRegister>("cheonil/read_register", 1);
    rosPubs_[CoreCommand::ROBOT_CHEONIL_READ_COIL] = nodeHandler_.advertise<core_msgs::CheonilReadCoil>("cheonil/read_coil", 1);
    rosPubs_[CoreCommand::ROBOT_SPEED_LIMIT] = nodeHandler_.advertise<std_msgs::Float64>("navifra/speed_limit", 1);
    rosPubs_[CoreCommand::NAVIFRA_COMMAND] = nodeHandler_.advertise<std_msgs::String>("navifra/cmd", 1);
    rosPubs_[CoreCommand::ROBOT_LOADED] = nodeHandler_.advertise<std_msgs::Bool>("navifra/loaded", 1);
    rosPubs_[CoreCommand::ROBOT_CHARGING_SUCCESS] = nodeHandler_.advertise<std_msgs::Bool>("charging_success", 1);
    rosPubs_[CoreCommand::ROBOT_FORK_POSITION_STATE] = nodeHandler_.advertise<std_msgs::Int16>("fork_position_state", 1);

    rosSubs_.push_back(nodeHandler_.subscribe("navifra/info", 1, &NaviCoreNoetic::onNavifrainfo, this));
    rosSubs_.push_back(nodeHandler_.subscribe("front_cloud_global", 1, &NaviCoreNoetic::onFrontCloud, this));
    rosSubs_.push_back(nodeHandler_.subscribe("rear_cloud_global", 1, &NaviCoreNoetic::onRearCloud, this));
    rosSubs_.push_back(nodeHandler_.subscribe("left_cloud_global", 1, &NaviCoreNoetic::onLeftCloud, this));
    rosSubs_.push_back(nodeHandler_.subscribe("right_cloud_global", 1, &NaviCoreNoetic::onRightCloud, this));
    rosSubs_.push_back(nodeHandler_.subscribe("v2v_cloud_global", 1, &NaviCoreNoetic::onV2VCloud, this));
    rosSubs_.push_back(nodeHandler_.subscribe("camera_cloud_global", 1, &NaviCoreNoetic::onCameraCloud, this));
    // rosSubs_.push_back(nodeHandler_.subscribe("obs_cloud_global", 1, &NaviCoreNoetic::onObsCloud, this));
    rosSubs_.push_back(nodeHandler_.subscribe("nc_task_manager/task_info", 20, &NaviCoreNoetic::onTaskInfo, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_TASK_ALAM, 20, &NaviCoreNoetic::onTaskAlarm, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_TASK_RESPONSE, 20, &NaviCoreNoetic::onTaskResponse, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_MAPING_PROGRESS, 1, &NaviCoreNoetic::onMappingProgress, this));
    rosSubs_.push_back(nodeHandler_.subscribe("localization/robot_pos", 1, &NaviCoreNoetic::onRobotPos, this));

    rosSubs_.push_back(nodeHandler_.subscribe("cali/calresult", 1, &NaviCoreNoetic::onCaliResult, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_CALI_PROGRESS, 1, &NaviCoreNoetic::onCaliProgressValue, this));

    rosSubs_.push_back(nodeHandler_.subscribe("motor_driverToAgent/serve_on", 1, &NaviCoreNoetic::onMotorDriveServeOn, this));
    rosSubs_.push_back(nodeHandler_.subscribe("NaviFra/error_dist", 1, &NaviCoreNoetic::onErrorDist, this));
    rosSubs_.push_back(nodeHandler_.subscribe("SeverityMin/brain_agent", 1, &NaviCoreNoetic::onSetSeverityMin, this));
    rosSubs_.push_back(nodeHandler_.subscribe("SeverityMin", 1, &NaviCoreNoetic::onSetSeverityMin, this));

    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_LOCAL_PATH, 1, &NaviCoreNoetic::onLocalPath, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_GLOBAL_PATH, 1, &NaviCoreNoetic::onGlobalPath, this));

    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_PREDICT_COLLISION, 1, &NaviCoreNoetic::onPredictCollision, this));
    rosSubs_.push_back(nodeHandler_.subscribe("NaviFra/visualize/robot_collision", 1, &NaviCoreNoetic::onCollision, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESASGE_MOTOR_INFO, 1, &NaviCoreNoetic::onMotorInfo, this));
    rosSubs_.push_back(nodeHandler_.subscribe("odom", 1, &NaviCoreNoetic::onOdometry, this));
    rosSubs_.push_back(nodeHandler_.subscribe("/rosout_agg", 1, &NaviCoreNoetic::onChatterCallback, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_OBSTACLE, 2, &NaviCoreNoetic::onObstacle, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_CUSTOM, 2, &NaviCoreNoetic::onCustomMessgae, this));

    rosSubs_.push_back(nodeHandler_.subscribe("navifra/param_update", 2, &NaviCoreNoetic::onParamUpdate, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_SLAM_GRAPH, 1, &NaviCoreNoetic::onSLAMGraph, this));
    // rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_SLAM_GRAPH, 1, &NaviCoreNoetic::onSLAMGraph, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_HARDWARE_INFO, 1, &NaviCoreNoetic::onHardwareInfo, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_NAVI_ALARM, 1, &NaviCoreNoetic::onRecvAlarm, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_CHARGING_CMD, 1, &NaviCoreNoetic::onRecvChargingCmd, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_LIGHTING_CMD, 1, &NaviCoreNoetic::onRecvLightingCmd, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_OSSD_FIELD, 1, &NaviCoreNoetic::onRecvOssdFieldCmd, this));
    rosSubs_.push_back(nodeHandler_.subscribe("navifra/charge_on", 1, &NaviCoreNoetic::onRecvCharge, this));

    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_SET_MARKER, 1, &NaviCoreNoetic::onSetMarker, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_SET_REFLECTORS, 1, &NaviCoreNoetic::onSetReflectors, this));
    rosSubs_.push_back(nodeHandler_.subscribe("battery_info", 1, &NaviCoreNoetic::onBatteryInfo, this));
    rosSubs_.push_back(nodeHandler_.subscribe("change_current_area", 1, &NaviCoreNoetic::onChangeCurrentID, this));
    rosSubs_.push_back(nodeHandler_.subscribe(CoreMessage::CORE_MESSAGE_FORK_CMD, 1, &NaviCoreNoetic::onForkCmd, this));
    rosSubs_.push_back(nodeHandler_.subscribe("pallet_id", 1, &NaviCoreNoetic::onPallectID, this));
    rosSubs_.push_back(nodeHandler_.subscribe("wia_agent/now_task", 1, &NaviCoreNoetic::onWiaNowTask, this));

    auto lidarMerger = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
    // PointCloud1
    sensor_msgs::PointCloud::Ptr cloud1(new sensor_msgs::PointCloud);
    sensor_msgs::PointCloud::Ptr cloud2(new sensor_msgs::PointCloud);
    sensor_msgs::PointCloud::Ptr cloud3(new sensor_msgs::PointCloud);
    sensor_msgs::PointCloud::Ptr cloud4(new sensor_msgs::PointCloud);
    sensor_msgs::PointCloud::Ptr cloud5(new sensor_msgs::PointCloud);
    sensor_msgs::PointCloud::Ptr cloud6(new sensor_msgs::PointCloud);
    // sensor_msgs::PointCloud::Ptr cloud7(new sensor_msgs::PointCloud);
    lidarMerger->update(LIDAR::LIDAR_CAMERA, cloud1);
    lidarMerger->update(LIDAR::LIDAR_LEFT, cloud2);
    lidarMerger->update(LIDAR::LIDAR_RIGHT, cloud3);
    lidarMerger->update(LIDAR::LIDAR_V2V, cloud4);
    lidarMerger->update(LIDAR::LIDAR_FRONT, cloud5);
    lidarMerger->update(LIDAR::LIDAR_REAR, cloud6);
    // lidarMerger->update(LIDAR::LIDAR_OBS, cloud7);
}

NaviCoreNoetic::~NaviCoreNoetic()
{
    for (auto pub : rosPubs_) {
        pub.second.shutdown();
    }

    for (auto sub : rosSubs_) {
        sub.shutdown();
    }
}

void NaviCoreNoetic::stopMapping()
{
    NLOG(info) << "stopMapping";
#if 0
    if (system("rosnode kill /slam_node"))
        NLOG(error) << "Can't kill slam_node";
    if (system("rosnode kill /map_generator_node"))
        NLOG(error) << "Can't kill /map_generator_node";
    if (system("rosnode kill /slam_data_logger"))
        NLOG(error) << "Can't kill slam_data_logger";
#else
    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
    obj->set("command", "terminate_slam");
    std::ostringstream oss;
    obj->stringify(oss);
    requestStringSrv(CoreCommand::ANSWER_CONTROL, oss.str());
#endif
    isSLAM_ = false;
}

void NaviCoreNoetic::startMapping()
{
#if 0
    NLOG(info) << "startMapping";
    if (system("roslaunch core_launch slam.launch &"))
        NLOG(error) << "Can't start slam";
#else

    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
    obj->set("command", "start_slam");
    std::ostringstream oss;
    obj->stringify(oss);
    requestStringSrv(CoreCommand::ANSWER_CONTROL, oss.str());

#endif
    isSLAM_ = true;
}

void NaviCoreNoetic::stopNavigation()
{
    NLOG(info) << "stopNavigation";
#if 0
    if (system("rosnode kill /nc_localizer_ros"))
        NLOG(error) << "Can't kill nc_localizer_ros";
#else

    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
    obj->set("command", "terminate_localization");
    std::ostringstream oss;
    obj->stringify(oss);
    requestStringSrv(CoreCommand::ANSWER_CONTROL, oss.str());

#endif
}

void NaviCoreNoetic::startNavigation()
{
    NLOG(info) << "startNavigation";
#if 0
    if (system("roslaunch core_launch localizer.launch &"))
        NLOG(error) << "Can't start localizer";

#else

    Poco::JSON::Object::Ptr obj = new Poco::JSON::Object();
    obj->set("command", "start_localization");
    std::ostringstream oss;
    obj->stringify(oss);
    requestStringSrv(CoreCommand::ANSWER_CONTROL, oss.str());

#endif
}

void NaviCoreNoetic::startSCAN()
{
    // scanStartTime_.restart();
    // auto scanStatus = InMemoryRepository::instance().get<RobotScanStatus>(RobotScanStatus::KEY);
    // scanStatus->On();
}

bool NaviCoreNoetic::isSLAM()
{
    return isSLAM_;
}

void NaviCoreNoetic::cancelTask()
{
    std_msgs::String commandMsg;
    commandMsg.data = "cancel";
    if (sendROSMessage(CoreCommand::TASK_COMMAND, commandMsg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::TASK_COMMAND.c_str());
    }
}

void NaviCoreNoetic::onChangeCurrentID(const std_msgs::String::ConstPtr msg)
{
    try {
        if (msg->data.empty())
            return;
        changeArea(msg->data);
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

static bool ReadAll(const std::string& path, std::string& out)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
        return false;
    std::ostringstream oss;
    oss << ifs.rdbuf();
    out = oss.str();
    return true;
}

// map.json → "id"(=map_id) 읽기
static std::optional<std::string> ReadMapJsonId(const std::string& p)
{
    std::string body;
    if (!ReadAll(p, body))
        return std::nullopt;
    try {
        Poco::JSON::Parser parser;
        auto var = parser.parse(body);
        auto root = var.extract<Poco::JSON::Object::Ptr>();
        if (!root || !root->has("id"))
            return std::nullopt;
        return root->get("id").convert<std::string>();
    }
    catch (...) {
        return std::nullopt;
    }
}

// ---- 헬퍼: 파일에서 Array 로드 ----
static Poco::JSON::Array::Ptr LoadAreasArray(const std::string& areas_path)
{
    std::ifstream ifs(areas_path);
    if (!ifs.is_open())
        return new Poco::JSON::Array();

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    Poco::JSON::Parser parser;
    auto var = parser.parse(buffer.str());
    auto root = var.extract<Poco::JSON::Object::Ptr>();
    return root->getArray("areas");
}

static bool WriteAll(const std::string& path, const std::string& content)
{
    fs::create_directories(fs::path(path).parent_path());
    std::ofstream ofs(path, std::ios::trunc);
    if (!ofs.is_open())
        return false;
    ofs << content;
    return true;
}

static bool WriteCurrentAreaId(const std::string& p, const std::string& area_id)
{
    return WriteAll(p, area_id + "\n");
}

void NaviCoreNoetic::changeArea(std::string s_area_id)
{
    NLOG(info) << "Changing current area to: " << s_area_id;
    std::string s_path_ = std::string(std::getenv("HOME")) + "/navifra_solution/navicore/configs/map/latest/";
    std::string srcFile = s_path_ + "map_" + s_area_id + ".json";
    std::string dstFile = s_path_ + "map.json";
    std::string curFile = s_path_ + "current_area.txt";

    try {
        NLOG(info) << "Changing current area to: " << s_area_id;

        if (!fs::exists(srcFile)) {
            NLOG(error) << "Source file does not exist: " << srcFile;
            return;
        }

        fs::copy_file(srcFile, dstFile, fs::copy_options::overwrite_existing);
        NLOG(info) << "Copied " << srcFile << " -> " << dstFile;
        auto haveMapId = ReadMapJsonId(dstFile);
        NLOG(info) << "[MAP] Replaced map.json with mapid " << *haveMapId;
        WriteCurrentAreaId(curFile, s_area_id);

        updateBrainMap("");
        std::string current_area_id = s_area_id;

        if (current_area_id.empty()) {
            NLOG(error) << "[MAP] No current area_id found. update map";
        }
        // robot info 업데이트
        std::string areas_path = s_path_ + "areas.json";

        auto storedArr = LoadAreasArray(areas_path);

        std::vector<std::string> vec_area_uuid;
        std::vector<std::string> vec_map_uuid;

        if (storedArr) {
            for (size_t i = 0; i < storedArr->size(); ++i) {
                auto obj = storedArr->getObject(i);
                if (!obj)
                    continue;

                if (obj->has("id")) {
                    vec_area_uuid.push_back(obj->get("id").convert<std::string>());
                }
                if (obj->has("acs_map_id")) {
                    vec_map_uuid.push_back(obj->get("acs_map_id").convert<std::string>());
                }
            }
        }
        // InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setArea(current_area_id, vec_area_uuid, vec_map_uuid);
        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        robotInfo->setArea(current_area_id, vec_area_uuid, vec_map_uuid);
    }
    catch (const fs::filesystem_error& e) {
        NLOG(error) << "Filesystem error: " << e.what();
    }
}

void NaviCoreNoetic::initialPose(Position position, Orientation orientation, bool correct_position)
{
    geometry_msgs::PoseWithCovarianceStamped initPoseMsg;
    initPoseMsg.pose.pose.position.x = position.x;
    initPoseMsg.pose.pose.position.y = position.y;
    initPoseMsg.pose.pose.position.z = position.z;

    initPoseMsg.pose.pose.orientation.x = orientation.x;
    initPoseMsg.pose.pose.orientation.y = orientation.y;
    initPoseMsg.pose.pose.orientation.z = orientation.z;
    initPoseMsg.pose.pose.orientation.w = orientation.w;

    if (sendROSMessage(CoreCommand::ROBOT_INIT_POSE, initPoseMsg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_INIT_POSE.c_str());
    }
    else {
        if (correct_position)
            sendROSMessage(CoreCommand::ROBOT_CORRECTION_INIT_POSE, std_msgs::String());
    }
}

void NaviCoreNoetic::deviceGoal(std::string id, std::string name, std::string type)
{
    core_msgs::Goal goalMsg;
    goalMsg.id = id;
    goalMsg.name = name;
    goalMsg.type = type;

    if (sendROSMessage(CoreCommand::ROBOT_DEVICE_GOAL, goalMsg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_DEVICE_GOAL.c_str());
    }
}

void NaviCoreNoetic::robotStop()
{
    std_msgs::String stopMsg;
    stopMsg.data = "cancel";

    if (sendROSMessage(CoreCommand::ROBOT_STOP, stopMsg) != true) {
        LOG_ERROR("ros publiser not found action command %s", CoreCommand::ROBOT_STOP.c_str());
    }

    if (sendROSMessage(CoreCommand::TASK_COMMAND, stopMsg) != true) {
        LOG_ERROR("ros publiser not found action command %s", CoreCommand::TASK_COMMAND.c_str());
    }
}

void NaviCoreNoetic::deviceCommand(std::string command)
{
    std_msgs::String commandMsg;
    commandMsg.data = command;

    if (sendROSMessage(CoreCommand::ROBOT_DEVICE_COMMAND, commandMsg) != true) {
        LOG_ERROR("ros publiser not found action command %s", CoreCommand::ROBOT_DEVICE_COMMAND.c_str());
    }
}

void NaviCoreNoetic::navifraCommand(std::string navifra_command)
{
    std_msgs::String commandMsg;
    commandMsg.data = navifra_command;

    if (sendROSMessage(CoreCommand::NAVIFRA_COMMAND, commandMsg) != true) {
        LOG_ERROR("ros publiser not found action command %s", CoreCommand::NAVIFRA_COMMAND.c_str());
    }
}

void NaviCoreNoetic::move(std::string id, std::string type)
{
    // core_msgs::Goal goalMsg;
    // goalMsg.id = id;
    // goalMsg.type = type;
    std::string json_data = R"({
        "action": "add_task",
        "data": [
            {
                "type": "move",
                "uuid": "",
                "target_node": ""
            }
        ],
        "uuid": "76f6ce59-9bc6-467e-9e13-e69f149b3c8f"
    })";

    // JSON 파싱
    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(json_data);
    Poco::JSON::Object::Ptr jsonObject = result.extract<Poco::JSON::Object::Ptr>();

    // target_node 수정
    Poco::JSON::Array::Ptr dataArray = jsonObject->getArray("data");
    Poco::JSON::Object::Ptr dataObject = dataArray->getObject(0);
    dataObject->set("target_node", id);  // 새로운 값으로 변경

    std::string task_uuid = generateUUID();  // UUID 생성 함수 호출
    dataObject->set("uuid", task_uuid);

    // 수정된 JSON을 문자열로 변환
    std::ostringstream oss;
    jsonObject->stringify(oss);
    std::string modified_json = oss.str();

    ProcessResult process_result = requestStringSrv("add_task", modified_json);
    if (process_result.success) {
        NLOG(info) << "move request success";
    }
    else {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_MOVE.c_str());
    }

    // if (sendROSMessage(CoreCommand::ROBOT_MOVE, goalMsg) != true) {
    //     LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_MOVE.c_str());
    // }
}

void NaviCoreNoetic::move(const std::vector<Pose>& paths)
{
    if (paths.size() != 2) {
        LOG_ERROR("Error: Incorrect path input, size is not 2.");
        return;
    }

    core_msgs::HacsNodeList hacsnodelist;
    core_msgs::HacsNode start, end;
    tf::Quaternion q1, q2;

    start.drive_type = 0;
    start.name = "1";  // 임시
    start.speed = 0.5;
    start.drive_type = 0;
    start.x_m = paths[0].position.x;
    start.y_m = paths[0].position.y;
    q1.setX(paths[0].orientation.x);
    q1.setY(paths[0].orientation.y);
    q1.setZ(paths[0].orientation.z);
    q1.setW(paths[0].orientation.w);

    tf::Matrix3x3 m1(q1);
    double roll1, pitch1, yaw1;
    m1.getRPY(roll1, pitch1, yaw1);
    start.angle_deg = yaw1 * (180.0 / M_PI);

    end.drive_type = 9;
    end.name = "2";
    end.x_m = paths[1].position.x;
    end.y_m = paths[1].position.y;
    q2.setX(paths[1].orientation.x);
    q2.setY(paths[1].orientation.y);
    q2.setZ(paths[1].orientation.z);
    q2.setW(paths[1].orientation.w);

    tf::Matrix3x3 m2(q2);
    double roll2, pitch2, yaw2;
    m2.getRPY(roll2, pitch2, yaw2);
    end.angle_deg = yaw2 * (180.0 / M_PI);

    hacsnodelist.b_docking_flag = false;
    hacsnodelist.b_docking_in = false;
    hacsnodelist.b_docking_state = false;
    hacsnodelist.b_start_pause = false;

    hacsnodelist.nodes.push_back(start);
    hacsnodelist.nodes.push_back(end);

    LOG_INFO("Robot Pose Move Started");

    if (sendROSMessage(CoreCommand::ROBOT_DOCKING, hacsnodelist) != true) {
        LOG_ERROR("ros publiser not found action command %s", CoreCommand::ROBOT_DOCKING.c_str());
    }
}

void NaviCoreNoetic::taskCommand(std::string command)
{
    std_msgs::String commandMsg;
    commandMsg.data = command;

    if (sendROSMessage(CoreCommand::TASK_COMMAND, commandMsg) != true) {
        LOG_ERROR("ros publiser not found action command %s", CoreCommand::TASK_COMMAND.c_str());
    }
}

void NaviCoreNoetic::updateParameters()
{
    // navifra_solution/navicore/configs param set후에 업데이트하면됨
    std::string yamlFile = std::string(getenv("HOME")) + "/navifra_solution/navicore/install/share/core_launch/launch/param.yaml";
    std::string yamlFile2 = std::string(getenv("HOME")) + "/navifra_solution/navicore/configs/param.yaml";

    try {
        // 최상위 노드가 map 형태라고 가정하고 재귀적으로 파라미터 설정
        YAML::Node config = YAML::LoadFile(yamlFile);
        setParamFromYAML("", config);
        YAML::Node config2 = YAML::LoadFile(yamlFile2);
        setParamFromYAML("", config2);
    }
    catch (const YAML::Exception& e) {
        LOG_ERROR("YAML 파싱 오류: %s", e.what());
    }
    if (sendROSMessage(CoreCommand::ROBOT_UPDATE_PARAMETERS, std_msgs::String()) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_UPDATE_PARAMETERS.c_str());
    }
}

// 재귀적으로 YAML 노드를 순회하면서 ROS 파라미터 서버에 설정하는 함수
void NaviCoreNoetic::setParamFromYAML(const std::string& baseKey, const YAML::Node& node)
{
    if (node.IsScalar()) {
        // 스칼라인 경우, 문자열로 변환하여 설정 (타입에 따라 as<int>(), as<double>() 등으로 변경 가능)
        if (typeName(node.as<std::string>()) == "bool") {
            ros::param::set(baseKey, node.as<bool>());
        }
        else if (typeName(node.as<std::string>()) == "int") {
            ros::param::set(baseKey, node.as<int>());
        }
        else if (typeName(node.as<std::string>()) == "float") {
            ros::param::set(baseKey, node.as<float>());
        }
        else {
            ros::param::set(baseKey, node.as<std::string>());
        }
    }
    else if (node.IsSequence()) {
        // 시퀀스(배열)인 경우, 문자열 벡터로 변환 (필요 시 다른 타입의 벡터로 변경)
        if (node.size() > 0) {
            if (typeName(node[0].as<std::string>()) == "int" || typeName(node[0].as<std::string>()) == "float") {
                std::vector<float> vec;
                for (std::size_t i = 0; i < node.size(); ++i) {
                    vec.push_back(node[i].as<float>());
                }
                ros::param::set(baseKey, vec);
            }
            else {
                std::vector<std::string> vec;
                for (std::size_t i = 0; i < node.size(); ++i) {
                    vec.push_back(node[i].as<std::string>());
                }
                ros::param::set(baseKey, vec);
            }
        }
    }
    else if (node.IsMap()) {
        // map인 경우, 각 하위 키에 대해 재귀 호출
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            std::string key = it->first.as<std::string>();
            // ROS 파라미터 서버에서는 "/"로 네임스페이스를 구분하므로 baseKey와 결합
            std::string fullKey = baseKey.empty() ? key : baseKey + "/" + key;
            setParamFromYAML(fullKey, it->second);
        }
    }
}

std::string NaviCoreNoetic::typeName(std::string node)
{
    std::regex integerPattern("^-?\\d+$");  // 정수
    std::regex floatPattern("-?\\d+(\\.\\d+)?");  // 실수
    std::regex pattern("(true|false)$");  // bool

    if (std::regex_match(node, integerPattern)) {
        return "int";
    }
    else if (std::regex_match(node, floatPattern)) {
        return "float";
    }
    else if (std::regex_match(node, pattern)) {
        return "bool";
    }

    return "string";
}

void NaviCoreNoetic::updateBrainMap(std::string type)
{
    std_msgs::String msg;
    msg.data = type;
    if (sendROSMessage(CoreCommand::MAP_UPDATE, msg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::MAP_UPDATE.c_str());
    }
}

void NaviCoreNoetic::cmdVel(float linearX, float linearY, float angularZ)
{
    geometry_msgs::Twist controlMsg;
    controlMsg.linear.x = linearX;
    controlMsg.linear.y = linearY;
    controlMsg.angular.z = angularZ;

    if (sendROSMessage(CoreCommand::ROBOT_CONTROL, controlMsg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_CONTROL.c_str());
    }
}

void NaviCoreNoetic::calibration(std::string calibrationType)
{
    std_msgs::String calilMsg;
    calilMsg.data = calibrationType;

    if (sendROSMessage(CoreCommand::ROBOT_CALIBRATION, calilMsg) != true) {
        LOG_ERROR("ros publiser not found action command %s", CoreCommand::ROBOT_CALIBRATION.c_str());
    }
}

void NaviCoreNoetic::updateRobotInfoPLC(std::string info)
{
    std_msgs::String robotDrivingInfoMsg;
    robotDrivingInfoMsg.data = info;
    if (sendROSMessage(CoreCommand::ROBOT_EXD_UPDATEROBOT_INFO, robotDrivingInfoMsg) != true) {
        LOG_ERROR("ros publiser not found action command %s", CoreCommand::ROBOT_EXD_UPDATEROBOT_INFO.c_str());
    }
}

void NaviCoreNoetic::plcStartStatus(std::string msg)
{
    std_msgs::String plcStatusStartMsg;
    plcStatusStartMsg.data = "plc_status_monitoring";
    if (sendROSMessage(CoreCommand::ROBOT_START_EXD_STATUS, plcStatusStartMsg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_START_EXD_STATUS.c_str());
        return;
    }
}

void NaviCoreNoetic::saveSLAM(int16_t isSave)
{
    std_msgs::Int16 saveSlamMsg;
    saveSlamMsg.data = 1;

    if (sendROSMessage(CoreCommand::ROBOT_SAVE_SLAM, saveSlamMsg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_SAVE_SLAM.c_str());
    }
}

void NaviCoreNoetic::node_action()
{
    std::string json_data = R"({
        "action": "add_task",
        "data": [
            {
                "type": "action",
                "uuid": "12345678-d67a-4a79-955c-d87d469f2bc9"
            }
        ],
        "uuid": "98765432-9bc6-467e-9e13-e69f149b3c8f"
    })";

    // JSON 파싱
    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(json_data);
    Poco::JSON::Object::Ptr jsonObject = result.extract<Poco::JSON::Object::Ptr>();

    // 수정된 JSON을 문자열로 변환
    std::ostringstream oss;
    jsonObject->stringify(oss);
    std::string modified_json = oss.str();

    ProcessResult process_result = requestStringSrv("add_task", modified_json);
    if (process_result.success) {
        NLOG(info) << "move request success";
    }
    else {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_MOVE.c_str());
    }
}

void NaviCoreNoetic::docking(DockingNode start, DockingNode end, DockingType type)
{
    core_msgs::HacsNodeList hacsnodelist = core_msgs::HacsNodeList();
    core_msgs::HacsNode start_node = core_msgs::HacsNode();
    core_msgs::HacsNode goal_node = core_msgs::HacsNode();

    start_node.name = "1";
    start_node.x_m = start.f_x;
    start_node.y_m = start.f_y;
    start_node.angle_deg = start.f_angle_deg;
    start_node.speed = start.f_linear_speed;
    start_node.drive_type = start.n_drive_type;

    goal_node.name = "2";
    goal_node.x_m = end.f_x;
    goal_node.y_m = end.f_y;
    goal_node.angle_deg = end.f_angle_deg;
    goal_node.drive_type = 10;

    hacsnodelist.nodes.push_back(start_node);
    hacsnodelist.nodes.push_back(goal_node);

    hacsnodelist.b_docking_flag = type.b_flag;
    hacsnodelist.b_docking_in = type.b_in;
    hacsnodelist.b_docking_state = type.b_state;
    hacsnodelist.b_start_pause = type.b_start_pause;

    if (sendROSMessage(CoreCommand::ROBOT_DOCKING, hacsnodelist) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_DOCKING.c_str());
    }
}

void NaviCoreNoetic::playBackCMD(std::string cmd)
{
    std_msgs::String commnad;
    commnad.data = cmd;
    if (sendROSMessage(CoreCommand::ROBOT_PLAYBACK, commnad) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_PLAYBACK.c_str());
    }
}

void NaviCoreNoetic::repeatTestCMD(std::string cmd)
{
    Poco::JSON::Parser parser;
    Poco::Dynamic::Var result = parser.parse(cmd);
    Poco::JSON::Object::Ptr obj = result.extract<Poco::JSON::Object::Ptr>();
    std::string strCommand = obj->get("command");

    if (strCommand == "start") {
        if (obj->has("nodes") && obj->has("iteration") && obj->has("stop")) {
            core_msgs::RepeatTestMsg command;
            Poco::JSON::Array::Ptr nodes = obj->getArray("nodes");
            core_msgs::RepeatTestMsg::_recordNodeIdArray_type node_array;

            for (const auto node : *nodes) {
                node_array.push_back(node);
            }

            command.recordNodeIdArray = node_array;
            command.repeatDrivingNodeIdArray = node_array;
            command.numberOfIterations = obj->get("iteration");
            command.stopSeconds = obj->get("stop");

            if (sendROSMessage(CoreCommand::ROBOT_REPEAT_DRIVE, command) != true) {
                LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_REPEAT_DRIVE.c_str());
            }
        }
    }
    else {
        std_msgs::String commnad;
        commnad.data = strCommand;
        if (sendROSMessage(CoreCommand::ROBOT_REPEAT_DRIVE_CMD, commnad) != true) {
            LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_REPEAT_DRIVE_CMD.c_str());
        }
    }
}

bool NaviCoreNoetic::getIsPLCManual()
{
    return isPlcRemoteManual_;
}

ProcessResult NaviCoreNoetic::requestStringSrv(std::string serviceName, std::string data)
{
    ProcessResult result;
    try {
        // auto& serviceClient = service_->at(serviceName);
        ros::NodeHandle nodeHandler_;

        ros::ServiceClient serviceClient;
        if (CoreCommand::TASK_ADD == serviceName) {
            serviceClient = nodeHandler_.serviceClient<core_msgs::CommonString>("nc_task_manager_srv/task_add");
        }
        else if (CoreCommand::ROBOT_START_SCAN == serviceName) {
            serviceClient = nodeHandler_.serviceClient<core_msgs::CommonString>("scan_quality_service");
        }
        else if (CoreCommand::ANSWER_CONTROL == serviceName) {
            serviceClient = nodeHandler_.serviceClient<core_msgs::CommonString>(serviceName);
        }

        core_msgs::CommonString srv;
        srv.request.data = data;

        if (serviceClient.call(srv)) {
            if (srv.response.success) {
                NLOG(info) << Poco::format("%s Service call successful!", serviceName);
                result.success = true;
                result.message = "success";
                return result;
            }
            else {
                NLOG(error) << Poco::format("%s Service call failed! reason [ %s ]", serviceName, srv.response.reason);
                result.success = false;
                result.message = srv.response.reason;
                return result;
            }
        }
        else {
            NLOG(error) << "Failed to call service";
            result.success = false;
            result.message = "Service Timeout";
            return result;
        }
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << Poco::format("requestStringSrv exception reason %s, service name %s", ex.displayText(), serviceName);
        result.success = false;
        result.message = ex.displayText();
        return result;
    }
}

void NaviCoreNoetic::calibrationDockingSave(std::string msg)
{
    std_msgs::String message;
    message.data = msg;
    if (sendROSMessage(CoreCommand::ROBOT_CALIBRATION_DOCKING_SAVE, message) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_CALIBRATION_DOCKING_SAVE.c_str());
    }
}

void NaviCoreNoetic::barcodeReader(bool onOff)
{
    std_msgs::Bool message;
    message.data = onOff;
    if (sendROSMessage(CoreCommand::ROBOT_BARCODE_READER, message) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_BARCODE_READER.c_str());
    }
}

void NaviCoreNoetic::calibrationSteerWrite(std::string msg)
{
    std_msgs::String message;
    message.data = msg;
    if (sendROSMessage(CoreCommand::ROBOT_CALIBRATION_STEER_WRITE, message) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_CALIBRATION_STEER_WRITE.c_str());
    }
}

void NaviCoreNoetic::calibrationSteerZeroSet(bool bZero)
{
    std_msgs::String message;
    // message.data = bZero;
    if (sendROSMessage(CoreCommand::ROBOT_CALIBRATION_STEER_ZEROSET, message) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_CALIBRATION_STEER_ZEROSET.c_str());
    }
}

void NaviCoreNoetic::fabcolor(std::string color)
{
    std_msgs::String message;
    message.data = color;

    if (sendROSMessage(CoreCommand::ROBOT_FAB_COLOR, message) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_FAB_COLOR.c_str());
    }
}
void NaviCoreNoetic::fabsound(std::string id, int repeat_num)
{
    core_msgs::Sound message;
    ;
    message.id = id;
    message.repeat_num = repeat_num;

    if (sendROSMessage(CoreCommand::ROBOT_FAB_SOUND, message) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_FAB_SOUND.c_str());
    }
}

void NaviCoreNoetic::responseAvoidance(bool bPermission)
{
    std_msgs::Bool message;
    message.data = bPermission;

    if (sendROSMessage(CoreCommand::ROBOT_RESPONSE_AVOIDANCE, message) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_RESPONSE_AVOIDANCE.c_str());
    }
}

void NaviCoreNoetic::disableLidar()
{
    isUseLidar = false;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    InMemoryRepository::instance().remove<LidarMerger>(LidarMerger::KEY);
}

void NaviCoreNoetic::SendNearestRobotPose(std::string pose)
{
    try {
        core_msgs::Vehicle vehcle;
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(pose);
        Poco::JSON::Object::Ptr obj = result.extract<Poco::JSON::Object::Ptr>();
        auto poseObj = obj->getObject("pose");
        auto positionObj = poseObj->getObject("position");
        auto velocityObj = obj->getArray("velocity");
        auto collisionObj = obj->getArray("collision");

        vehcle.id = obj->getValue<std::string>("id");
        vehcle.x_m = positionObj->getValue<float>("x");
        vehcle.y_m = positionObj->getValue<float>("y");
        vehcle.f_linear_speed_x_ms = velocityObj->get(0).convert<float>();
        vehcle.f_linear_speed_y_ms = velocityObj->get(1).convert<float>();
        vehcle.f_angular_speed_z_degs = velocityObj->get(2).convert<float>();

        float top = 0.0f, bottom = 0.0f, left = 0.0f, right = 0.0f;

        if (collisionObj->size() == 5) {
            Poco::JSON::Array::Ptr topright = collisionObj->getArray(1);
            Poco::JSON::Array::Ptr bottomright = collisionObj->getArray(2);
            Poco::JSON::Array::Ptr bottomleft = collisionObj->getArray(3);
            Poco::JSON::Array::Ptr topleft = collisionObj->getArray(4);

            float x1 = static_cast<float>(bottomleft->getElement<double>(0));
            float x2 = static_cast<float>(topleft->getElement<double>(0));
            float x3 = static_cast<float>(topright->getElement<double>(0));
            float x4 = static_cast<float>(bottomright->getElement<double>(0));

            float y1 = static_cast<float>(bottomleft->getElement<double>(1));
            float y2 = static_cast<float>(topleft->getElement<double>(1));
            float y3 = static_cast<float>(topright->getElement<double>(1));
            float y4 = static_cast<float>(bottomright->getElement<double>(1));

            top = std::max(y2, y3);  // topleft.y, topright.y
            bottom = std::min(y1, y4);  // bottomleft.y, bottomright.y
            left = std::min(x1, x2);  // bottomleft.x, topleft.x
            right = std::max(x3, x4);  // topright.x, bottomright.x
        }

        vehcle.f_robot_size_front_m = top;
        vehcle.f_robot_size_rear_m = bottom;
        vehcle.f_robot_size_right_m = right;
        vehcle.f_robot_size_left_m = left;

        if (sendROSMessage(CoreCommand::ROBOT_RSR_INFO, vehcle) != true) {
            LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_RSR_INFO.c_str());
        }
    }
    catch (const Poco::Exception& ex) {
    }
}

void NaviCoreNoetic::CheonilReadRegister(ReadRegister read_register)
{
    core_msgs::CheonilReadRegister msg;

    msg.command_num                     = read_register.command_num;
    msg.command                         = read_register.command;
    msg.target_node                     = read_register.target_node;
    msg.x_position                      = read_register.x_position;
    msg.y_position                      = read_register.y_position;
    msg.angle_position                  = read_register.angle_position;
    msg.speed_limit                     = read_register.speed_limit;
    msg.jog_enable                      = read_register.jog_enable;
    msg.jog_speed                       = read_register.jog_speed;
    msg.jog_angle_speed                 = read_register.jog_angle_speed;
    msg.jog_steer_deg                 = read_register.jog_steer_deg;
    msg.speed_type                      = read_register.speed_type;
    msg.none_speed                      = read_register.none_speed;
    msg.empty_speed                     = read_register.empty_speed;
    msg.load_state                      = read_register.load_state;
    msg.auto_on                         = read_register.auto_on;
    msg.battery                         = read_register.battery;
    msg.fork_up_down_position           = read_register.fork_up_down_position;
    msg.fork_up_down_complete           = read_register.fork_up_down_complete;
    msg.tilting_up_down                 = read_register.tilting_up_down;
    msg.fork_width                      = read_register.fork_width;
    msg.lsc_1                           = read_register.lsc_1;
    msg.lsc_2                           = read_register.lsc_2;
    msg.lsc_3                           = read_register.lsc_3;
    msg.lsc_4                           = read_register.lsc_4;
    msg.pallet_touch                           = read_register.pallet_touch;
    msg.charge_state                    = read_register.charge_state;
    msg.job_cancel                      = read_register.job_cancel;
    msg.pallet_id_remove                = read_register.pallet_id_remove;
    msg.plc_alarm                       = read_register.plc_alarm;
    
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    int n_fork_lift_cmd = static_cast<int>(robotInfo->getForkUpDownCmd());
    int n_fork_tilting_cmd = static_cast<int>(robotInfo->getTiltingUpDownCmd());
    int n_fork_width_cmd = static_cast<int>(robotInfo->getForkWidthCmd());
    // NLOG(info)<<"[FORK] Lift : "<<n_fork_lift_cmd<<", "<<read_register.fork_up_down_complete
    //         <<", Tilt : "<<n_fork_tilting_cmd<<", "<<read_register.tilting_up_down
    //         <<", Width : "<<n_fork_width_cmd<<", "<<read_register.fork_width;
    if(n_fork_lift_cmd != 0 && read_register.fork_up_down_complete) {
        robotInfo->setForkUpDownCmd(0);
    }
    if(n_fork_tilting_cmd == read_register.tilting_up_down)
    {
        robotInfo->setTiltingUpDownCmd(0);
    }
    if(n_fork_width_cmd == read_register.fork_width)
    {
        robotInfo->setForkWidthCmd(0);
    }

    int n_fork_position = 1;
    std_msgs::Int16 n_msg;
    if(msg.fork_up_down_position < 500) {
        n_fork_position = 0;
    }
    n_msg.data = n_fork_position;
    if (sendROSMessage(CoreCommand::ROBOT_FORK_POSITION_STATE, n_msg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_FORK_POSITION_STATE.c_str());
    }
    
    bool b_loaded = false;
    if(msg.load_state != 0 || msg.pallet_touch == 1)
    {
        b_loaded = true;
    }
    std_msgs::Bool b_msg;
    b_msg.data = b_loaded;
    if (sendROSMessage(CoreCommand::ROBOT_LOADED, b_msg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_LOADED.c_str());
    }    

    if(msg.job_cancel == 1) {
        std_msgs::String s_msg;
        s_msg.data = "cancel";
        if (sendROSMessage(CoreCommand::TASK_COMMAND, s_msg) != true) {
            LOG_ERROR("ros publisher not found action command %s", CoreCommand::TASK_COMMAND.c_str());
        }
    }

    if (sendROSMessage(CoreCommand::ROBOT_CHEONIL_READ_REGISTER, msg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_CHEONIL_READ_REGISTER.c_str());
    }
}

void NaviCoreNoetic::CheonilReadCoil(ReadCoil read_coil)
{
    core_msgs::CheonilReadCoil msg;

    msg.read_front_safety_scanner = read_coil.read_front_safety_scanner;
    msg.read_left_safety_scanner = read_coil.read_left_safety_scanner;
    msg.read_right_safety_scanner = read_coil.read_right_safety_scanner;

    if (sendROSMessage(CoreCommand::ROBOT_CHEONIL_READ_COIL, msg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_CHEONIL_READ_COIL.c_str());
    }
}

void NaviCoreNoetic::NavifraSpeedLimit(float speed_limit)
{
    std_msgs::Float64 message;
    message.data = speed_limit;

    if (sendROSMessage(CoreCommand::ROBOT_SPEED_LIMIT, message) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::ROBOT_SPEED_LIMIT.c_str());
    }
}

void NaviCoreNoetic::alarmclear()
{
    std_msgs::String commandMsg;
    commandMsg.data = "cancel";
    if (sendROSMessage(CoreCommand::NAVIFRA_COMMAND, commandMsg) != true) {
        LOG_ERROR("ros publisher not found action command %s", CoreCommand::NAVIFRA_COMMAND.c_str());
    }
}
