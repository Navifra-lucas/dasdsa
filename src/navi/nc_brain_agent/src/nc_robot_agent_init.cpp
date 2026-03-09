#include "nc_brain_agent/nc_brain_agent.h"

#include <Poco/Delegate.h>
#include <Poco/Net/HTTPResponse.h>
#include <core_agent/core/navicore.h>
#include <core_agent/core/navicore_message.h>
#include <core_agent/data/lidar_merger.h>
#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_collision_info.h>
#include <core_agent/data/robot_info.h>
#include <core_agent/data/robot_status.h>
#include <core_agent/manager/action_manager.h>
#include <core_agent/manager/alarm_manager.h>
#include <core_agent/manager/message_handler_manager.h>
#include <core_agent/manager/publish_manager.h>
#include <core_agent/message/message_broker.h>
#include <core_agent/message/message_publisher.h>
#include <core_agent/message/message_subscriber.h>
#include <core_agent/mqtt/mqtt_subscriber.h>
#include <core_agent/redis/redis_publisher.h>
#include <core_agent/redis/redis_subscriber.h>
#include <core_agent/util/config.h>
#include <nc_brain_agent/data/nc_agent_parameters.h>
#include <nc_brain_agent/data/nc_brain_map.h>
#include <nc_brain_agent/data/nc_status_channel.h>
#include <nc_brain_agent/message/nc_agent_heartbeat.h>
#include <nc_brain_agent/message/nc_brain_command.h>
#include <nc_brain_agent/message/nc_brain_message.h>
#include <nc_brain_agent/message/nc_message_alarm.h>
#include <nc_brain_agent/message/nc_message_cali_progress.h>
#include <nc_brain_agent/message/nc_message_custom.h>
#include <nc_brain_agent/message/nc_message_global_path.h>
#include <nc_brain_agent/message/nc_message_hardware_info.h>
#include <nc_brain_agent/message/nc_message_local_path.h>
#include <nc_brain_agent/message/nc_message_mapping.h>
#include <nc_brain_agent/message/nc_message_motor_info.h>
#include <nc_brain_agent/message/nc_message_polygon.h>
#include <nc_brain_agent/message/nc_message_reflectors.h>
#include <nc_brain_agent/message/nc_message_slam_node.h>
#include <nc_brain_agent/message/nc_message_task_alarm.h>
#include <nc_brain_agent/message/nc_message_task_response.h>
#include <nc_brain_agent/nc_robot_agent.h>
#include <nc_brain_agent/net/nc_rest_api_utils.h>
#include <nc_brain_agent/utils/nc_agent_config.h>
#include <nc_brain_agent/utils/nc_agent_utils.h>

using namespace NaviFra;

using Poco::Net::HTTPResponse;
using namespace NaviFra;

void NcRobotAgent::initRoboCollision()
{
    try {
        // Get the parameter "obstacle/collision" with a default value of an empty vector
        std::vector<float> robot_outline;
        std::vector<float> robot_collision;
        bool b_use_detection_mode_flag = false;
        ros::param::get("obstacle/outline", robot_outline);
        ros::param::get("obstacle/collision", robot_collision);
        ros::param::get("obstacle/b_use_detection_mode_flag", b_use_detection_mode_flag);

        float collision_offestX = 0.f;
        float collision_offestY = 0.f;
        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        auto robotCollisionInfo = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);
        if (robot_outline.size() == 4) {
            LOG_INFO("Read Robot Outline successfully.");
            robotCollisionInfo->initShape(
                robot_outline[0], robot_outline[1], robot_outline[3], robot_outline[2], collision_offestX, collision_offestY);
        }
        if (robot_collision.size() == 4 && b_use_detection_mode_flag) {
            LOG_INFO("Read Robot collision successfully.");
            robotCollisionInfo->initCollision(
                robot_collision[0], robot_collision[1], robot_collision[3], robot_collision[2], collision_offestX, collision_offestY);
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

        robotInfo->setRobotBase(f_base);
    }
    catch (Poco::Exception& ex) {
        LOG_ERROR("%s", ex.displayText().c_str());
    }
}
void NcRobotAgent::initActionManager()
{
    std::string agent_type = Config::instance().getString("agent_type", "base");

    ActionManager::instance().initialize(agent_type == "volvo" ? ActionType::VOLVO : ActionType::DEFAULT);
    ActionManager::instance().printActions();
}
void NcRobotAgent::initRosLaunchManager()
{
    std::string agent_type = Config::instance().getString("agent_type", "base");
}
void NcRobotAgent::initRobotGroup()
{
#ifndef CPP_UNIT_TEST
    std::string strDrivingType = NcAgentConfig::get().getDrivingType_cd();

    std::string subDrivingType = strDrivingType.substr(0, strDrivingType.size() - 2);
    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res_robottype =
        Net::HTTP::get("/group-codes/" + subDrivingType);
    std::string is_robottype = std::get<0>(res_robottype);
    HTTPResponse::HTTPStatus status = std::get<1>(res_robottype);
    std::string reason = std::get<2>(res_robottype);

    if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
        Poco::JSON::Object::Ptr dataRobottype = JSONParse(is_robottype);
        Poco::JSON::Array::Ptr listRobottype = dataRobottype.get()->getArray("list");
        for (int i = 0; i < listRobottype->size(); i++) {
            Poco::JSON::Object::Ptr obj = listRobottype->getObject(i);
            std::string strCodeid = obj->get("code_id").convert<std::string>();
            std::string strGroupCode_id = obj->get("group_code_id").convert<std::string>();

            if ((strCodeid == strDrivingType) && (subDrivingType == strGroupCode_id)) {
                InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->setRobotType(obj->get("code_kname").convert<std::string>());
            }
        }
    }
    else {
        // error
        LOG_ERROR("backend /group-codes/%s status error : Status %d Reason %s", subDrivingType.c_str(), (int)status, reason.c_str());
    }

    LOG_TRACE("Agent - BackEnd Get   ->  /group-codes/  - Passed");
#endif
}

bool NcRobotAgent::initMapInfo()
{
    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res_map =
        Net::HTTP::get("/scan-maps/main-info", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken());
    std::string is_map = std::get<0>(res_map);
    HTTPResponse::HTTPStatus status = std::get<1>(res_map);
    std::string reason = std::get<2>(res_map);

    if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
        Poco::JSON::Object::Ptr dataMap = JSONParse(is_map);

        InMemoryRepository::instance()
            .get<NcBrainMap>(NcBrainMap::KEY)
            ->setMainRevision(dataMap->getObject("item")->get("revision").convert<std::string>());
        if (!InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->initialize()) {
            res_map = Net::HTTP::get("/scan-maps/main", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken());
            is_map = std::get<0>(res_map);
            status = std::get<1>(res_map);
            reason = std::get<2>(res_map);

            if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
                dataMap = JSONParse(is_map);
                InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->localMapUpdate(dataMap->getObject("item"));
                InMemoryRepository::instance()
                    .get<NcBrainMap>(NcBrainMap::KEY)
                    ->teachingJsonInit(InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getMainRevision());

#ifndef CPP_UNIT_TEST
                updateBrainMap();
#endif

                InMemoryRepository::instance()
                    .get<RobotInfo>(RobotInfo::KEY)
                    ->setSetting(
                        std::atoi(InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getLocalRevision().c_str()),
                        InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getKeyNodesSize(),
                        InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getTeachedNodesSize());
            }
            else {
                // error
                LOG_ERROR("backend /scan-maps/main-info status error : Status %d Reason %s", (int)status, reason.c_str());
                return false;
            }
        }
        else {
            if (InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getLocalRevision() !=
                    InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getMainRevision() &&
                InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->isJobActive()) {
                res_map = Net::HTTP::get("/scan-maps/main", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken());
                is_map = std::get<0>(res_map);
                status = std::get<1>(res_map);
                reason = std::get<2>(res_map);
                if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
                    dataMap = JSONParse(is_map);
                    InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->localMapUpdate(dataMap->getObject("item"));
                    InMemoryRepository::instance()
                        .get<NcBrainMap>(NcBrainMap::KEY)
                        ->teachingJsonInit(InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getMainRevision());
#ifndef CPP_UNIT_TEST
                    updateBrainMap();
#endif
                }
                else {
                    LOG_ERROR("backend /scan-maps/main status error : Status %d Reason %s", (int)status, reason.c_str());
                    return false;
                }
            }

            InMemoryRepository::instance()
                .get<RobotInfo>(RobotInfo::KEY)
                ->setSetting(
                    std::atoi(InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getLocalRevision().c_str()),
                    InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getKeyNodesSize(),
                    InMemoryRepository::instance().get<NcBrainMap>(NcBrainMap::KEY)->getTeachedNodesSize());
        }
    }
    else {
        // error
        LOG_ERROR("backend /scan-maps/main-info status error : Status %d Reason %s", (int)status, reason.c_str());
    }
    LOG_TRACE("Agent - BackEnd Get   ->  /scan-maps/main-info  - Passed");

    return true;
}

void NcRobotAgent::updateRobotInfo()
{
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
    robotStatus->setID(NcAgentConfig::get().getRobotID());
    robotStatus->setToken(NcAgentConfig::get().getToken());
    robotInfo->setID(NcAgentConfig::get().getRobotID());

    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res = Net::HTTP::get(
        "/robots/" + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
        InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken());

    std::string is = std::get<0>(res);
    HTTPResponse::HTTPStatus status = std::get<1>(res);
    std::string reason = std::get<2>(res);

    if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
        Poco::JSON::Object::Ptr dataRobot = JSONParse(is);
        robotStatus->setID(dataRobot->getObject("item")->get("id").convert<std::string>());
        robotStatus->setToken(dataRobot->getObject("item")->get("token").convert<std::string>());
        robotStatus->setEnable(dataRobot->getObject("item")->get("is_enabled").convert<bool>());
        robotStatus->setActive(dataRobot->getObject("item")->get("job_is_active").convert<bool>());
        robotInfo->setID(dataRobot->getObject("item")->get("id").convert<std::string>());

        NcAgentConfig::get().setRobotID(robotStatus->getID());
        NcAgentConfig::get().setToken(robotStatus->getToken());
    }
    else {
        LOG_ERROR("Robot register error status error : Status %d Reason %s", (int)status, reason.c_str());
    }
}

bool NcRobotAgent::initVerifyRobot()
{
    robot_acs_reg_code_ = redisReader_->GET("robot_acs_reg_code");

    Object robot_create_data;
    std::string strModelType = Config::instance().getString("model_type", "D10001");
    std::string strDrivingType = Config::instance().getString("drive_type", "D30001");
    robot_create_data.set("model_type_cd", strModelType);
    robot_create_data.set("driving_type_cd", strDrivingType);
    robot_create_data.set("ip_addr", ip_address_);
    robot_create_data.set("robot_acs_reg_code", robot_acs_reg_code_);

    LOG_TRACE("Agent - robot_create_data - Passed");

    std::tuple<std::string, Poco::Net::HTTPResponse::HTTPStatus, std::string> res = Net::HTTP::post("/robots/create", robot_create_data);

    std::string is = std::get<0>(res);
    HTTPResponse::HTTPStatus status = std::get<1>(res);
    std::string reason = std::get<2>(res);

    /*
        로봇을 등록 해보고
    */
    if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_CREATED || status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
        Poco::JSON::Object::Ptr data = JSONParse(is);

        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->setID(data->getObject("item")->get("id").convert<std::string>());
        InMemoryRepository::instance()
            .get<RobotStatus>(RobotStatus::KEY)
            ->setToken(data->getObject("item")->get("token").convert<std::string>());
        robotInfo->setID(data->getObject("item")->get("id").convert<std::string>());

        NcAgentConfig::get().initializeRobot();

        NcAgentConfig::get().setRobotID(InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
        NcAgentConfig::get().setToken(InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getToken());
    }
    else if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_CONFLICT || 
             status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_BAD_REQUEST) {
        if (NcAgentConfig::get().getToken().empty() != true) {
            updateRobotInfo();
        }
        else {
            NLOG(error) << "Robot information has been deleted.";
            res = Net::HTTP::get(Poco::format("/robots/find-amr/%s", Config::instance().getString("robot_ip", "localhost")));
            is = std::get<0>(res);
            status = std::get<1>(res);
            reason = std::get<2>(res);

            auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
            auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);

            if (status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_OK) {
                Poco::JSON::Object::Ptr savedRobot = JSONParse(is);
                robotStatus->setID(savedRobot->getObject("item")->get("id").convert<std::string>());
                robotStatus->setToken(savedRobot->getObject("item")->get("token").convert<std::string>());
                robotStatus->setEnable(savedRobot->getObject("item")->get("is_enabled").convert<bool>());
                robotStatus->setActive(savedRobot->getObject("item")->get("job_is_active").convert<bool>());
                robotInfo->setID(savedRobot->getObject("item")->get("id").convert<std::string>());

                NcAgentConfig::get().setRobotID(robotStatus->getID());
                NcAgentConfig::get().setToken(robotStatus->getToken());
            }
            else {
                LOG_ERROR("Robot register error status error : Status %d Reason %s", (int)status, reason.c_str());
                return false;
            }
        }
    }
    else if (
        status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_BAD_REQUEST ||
        status == Poco::Net::HTTPResponse::HTTPStatus::HTTP_METHOD_NOT_ALLOWED) {
        LOG_ERROR("backend /robots/create status error : Status %d Reason %s", (int)status, reason.c_str());
        return false;
    }
    else {
        LOG_ERROR("backend /robots/create status error : Status %d Reason %s", (int)status, reason.c_str());
        return false;
    }

    Config::instance().setString("robot_id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());

    return true;
}

void NcRobotAgent::initROS()
{
    /*
        아래는 ROS의 Subscriber 콜백 함수에서 들어 오는 데이터를 재 가공 하여 UI로 보내 거나 아님 ACS로 보내 줘야 할 경우에 사용 하는
       Message Dispatcher 같은 개념이다. 에이전트의 기능 일부를 API로 만들 경우 콜백 정보가 필요 하지 않을 수 있는데 이를 호환성을 유지하며
       다른 곳에서 쓸 수 있도록 수정 하였다. 필요 한곳에서만 핸들러를 만들어 사용 할 수 있도록 하면 된다.
    */
    std::shared_ptr<NcMessageLocalPath> local_path = std::make_shared<NcMessageLocalPath>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_LOCAL_PATH, std::move(local_path));

    std::shared_ptr<NcMessageGlobalPath> global_path = std::make_shared<NcMessageGlobalPath>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_GLOBAL_PATH, std::move(global_path));

    std::shared_ptr<NcMessagePolygon> polygon_obstacle = std::make_shared<NcMessagePolygon>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_OBSTACLE, std::move(polygon_obstacle));

    std::shared_ptr<NcMessagePolygon> polygon_collision = std::make_shared<NcMessagePolygon>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_PREDICT_COLLISION, std::move(polygon_collision));

    std::shared_ptr<NcMessageCustom> custom_message = std::make_shared<NcMessageCustom>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_CUSTOM, std::move(custom_message));

    // std::shared_ptr<NcMessageTaskAlarm> task_alarm = std::make_shared<NcMessageTaskAlarm>();
    // MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_TASK_ALAM, std::move(task_alarm));

    std::shared_ptr<NcMessageTaskResponse> task_response = std::make_shared<NcMessageTaskResponse>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_TASK_RESPONSE, std::move(task_response));

    std::shared_ptr<NcMessageMapping> mapping_progress = std::make_shared<NcMessageMapping>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_MAPING_PROGRESS, std::move(mapping_progress));

    std::shared_ptr<NcMessageMotorInfo> motor_info = std::make_shared<NcMessageMotorInfo>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESASGE_MOTOR_INFO, std::move(motor_info));

    std::shared_ptr<NcMessageSLAMNode> slam_node = std::make_shared<NcMessageSLAMNode>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_SLAM_GRAPH, std::move(slam_node));

    std::shared_ptr<NcMessageCaliProgress> cali_progress = std::make_shared<NcMessageCaliProgress>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_CALI_PROGRESS, std::move(cali_progress));

    std::shared_ptr<NcMessageHardwareInfo> hardware_info = std::make_shared<NcMessageHardwareInfo>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_HARDWARE_INFO, std::move(hardware_info));

    std::shared_ptr<NcMessageAlarm> navi_alarm = std::make_shared<NcMessageAlarm>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_NAVI_ALARM, std::move(navi_alarm));

    std::shared_ptr<NcMessageReflectors> reflectors = std::make_shared<NcMessageReflectors>();
    MessageHandlerManager::instance().registerHandler(CoreMessage::CORE_MESSAGE_SET_REFLECTORS, std::move(reflectors));

    /*
        Status publish 해주는 모듈 초기화
        UI에 특정 주기로 보내 줘야 하는 데이터들이 있는데 그데이터를 주기적으로 보내 주기 위한 Publish Manager를 만들어 관리 한다.
        아래 와 같이 설정 하면 원하는 주기만큼 Publish 해준다 참고해서 작성 할것
    */
    std::string id = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID();
    NcAgentHearbeat heartbeat;
    heartbeat.setID(id);

    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_HEARBEAT,
        [heartbeat, id]() mutable {
            Poco::Timestamp now;
            heartbeat.setTimestamp(now);
            auto robotStatus = InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY);
            if (robotStatus->isSLAM()) {
                heartbeat.setIsSlam(true);
            }
            else {
                heartbeat.setIsSlam(false);
            }
            heartbeat.setVersion(robotStatus->getVersion());

            MessageBroker::instance().publish(NcBrainMessage::MESSAGE_ROBOT_HEARTBEAT_STATUS + id, heartbeat.toString());
        },
        500, 0);

    PublisherManager::instance().addChannel(
       (int)PUBLISH_CHANNEL::CHANNEL_INFO_STATUS,
       []() mutable {
           auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
           MessageBroker::instance().publish(NcBrainMessage::MESSAGE_ROBOT_STATUS_INFO + robotInfo->getID(), robotInfo->toString());

           updateRobotInfoPLC(robotInfo->getRobotDrivingInfoToString());
       },
       100, 0);

    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_LIDAR,
        []() mutable {
            auto lidar = InMemoryRepository::instance().get<LidarMerger>(LidarMerger::KEY);
            std::string json(Poco::format(
                "{\"id\":\"%s\",\"points\":%s}", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                lidar->toString()));

            MessageBroker::instance().publish(
                NcBrainMessage::MESSAGE_ROBOT_STATUS_SCAN + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
                json);
        },
        1000, 120000);

    ///채널 알람은 로봇 스테이터스를 감시해 running 이나, idle 일 경우 Alarm 을 클리어 하는 역할 만 한다.
    // 차후에 알람 클리어 인터렉션을 GUI에서 하게 되면 아래 코드는 삭제 되어야 한다.
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_ALARM,
        []() mutable {
            std::string status = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY)->getStatus();

            if ((status == "running" || status == "idle") && AlarmManager::instance().size() > 0) {
                AlarmManager::instance().clearAllAlarms();
            }
        },
        200, 0);

    // 자신의 위치와 속도 정보만 전달 함
    PublisherManager::instance().addChannel(
        (int)PUBLISH_CHANNEL::CHANNEL_STATUS_POSE,
        []() mutable {
            auto pose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
            auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);

            Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
            obj->set("id", robotInfo->getID());
            obj->set("pose", pose->toObject());
            obj->set("velocity", robotInfo->getVelocity());
            obj->set("collision", InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY)->getCollision());

            std::ostringstream oss;
            obj->stringify(oss);

            MessageBroker::instance().publish(
                "robot.status.pose:" + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(), oss.str());
        },
        1000, 0);

    PublisherManager::instance().deactivate((int)PUBLISH_CHANNEL::CHANNEL_LIDAR);
    PublisherManager::instance().start();
}

void NcRobotAgent::initRobotStartUp()
{
    Object data;
    data.set("id", InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID());
    data.set("data", "RobotAgent");
    std::ostringstream ostr;
    data.stringify(ostr);
    MessageBroker::instance().publish(
        NcBrainMessage::MESSAGE_ROBOT_STATUS_STARTUP + InMemoryRepository::instance().get<RobotStatus>(RobotStatus::KEY)->getID(),
        ostr.str());
}