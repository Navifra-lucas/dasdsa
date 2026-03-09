#include "nc_wia_agent/initializer/publisher_initializer.h"

#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/data/status_channel.h"
#include "util/logger.hpp"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_pose.h>
#include <core_agent/data/types.h>
#include <core_agent/manager/publish_manager.h>
#include <core_agent/message/message_broker.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace NaviFra;

double getYawFromOrientation(const Orientation& o)
{
    tf2::Quaternion quat(o.x, o.y, o.z, o.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

Poco::JSON::Object::Ptr buildBasicStatusResponse()
{
    auto robot_basic_status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY)->toObject();

    auto robotPose = InMemoryRepository::instance().get<RobotPose>(RobotPose::KEY);
    auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
    auto robotStatus = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
    auto robotCollisionInfo = InMemoryRepository::instance().get<RobotCollisionInfo>(RobotCollisionInfo::KEY);
    Poco::JSON::Array robotShape = robotCollisionInfo->getShape();

    std::string TaskID = robotStatus->getTaskID();

    Poco::Timestamp now;
    // 상태 정보는 RobotInfo 기준으로 간단히 추정 (예시)
    int n_action_index = robotStatus->getWiaTask();
    bool b_acs_pause = robotStatus->getACSPause();
    int n_robot_state = b_acs_pause ? 3 : robotStatus->getWiaStatus();
    
    double d_map_confidence = static_cast<double>(robotInfo->getConfidence());

    Poco::JSON::Object::Ptr pose = new Poco::JSON::Object;
    float f_yaw_deg = getYawFromOrientation(robotPose->getOrientation()) * 180.0 / M_PI;

    f_yaw_deg -= 180.0;
    while(f_yaw_deg > 180.0f){
        f_yaw_deg -= 360.0f;
    }
    while(f_yaw_deg <= -180.0f){
        f_yaw_deg += 360.0f;
    }
    pose->set("x", robotPose->getPosition().x);
    pose->set("y", robotPose->getPosition().y);
    pose->set("theta", f_yaw_deg * M_PI / 180.0);

    auto feed_vel = robotInfo->getVelocity();
    auto target_vel = robotInfo->getTargetVelocity();

    Poco::JSON::Object::Ptr feedVelLinear = new Poco::JSON::Object;
    feedVelLinear->set("x", feed_vel[0]);
    feedVelLinear->set("y", feed_vel[1]);
    feedVelLinear->set("z", 0.0);

    Poco::JSON::Object::Ptr feedVelAngular = new Poco::JSON::Object;
    feedVelAngular->set("x", 0.0);
    feedVelAngular->set("y", 0.0);
    feedVelAngular->set("z", feed_vel[2]);

    Poco::JSON::Object::Ptr feedVel = new Poco::JSON::Object;
    feedVel->set("linear", feedVelLinear);
    feedVel->set("angular", feedVelAngular);

    Poco::JSON::Object::Ptr targetVelLinear = new Poco::JSON::Object;
    targetVelLinear->set("x", target_vel[0]);
    targetVelLinear->set("y", target_vel[1]);
    targetVelLinear->set("z", 0.0);

    Poco::JSON::Object::Ptr targetVelAngular = new Poco::JSON::Object;
    targetVelAngular->set("x", 0.0);
    targetVelAngular->set("y", 0.0);
    targetVelAngular->set("z", target_vel[2]);

    Poco::JSON::Object::Ptr targetVel = new Poco::JSON::Object;
    targetVel->set("linear", targetVelLinear);
    targetVel->set("angular", targetVelAngular);

    // robot_basic_status->set("type", robotStatus->getRobotType().c_str());
    std::string typesdf = "test";
    robot_basic_status->set("type", typesdf);
    // NLOG(info) <<
    float test = 0.6;
    robot_basic_status->set("radius", test);  // 예시값
    robot_basic_status->set("pose", pose);

    robot_basic_status->set("workstate", n_robot_state);  // obj->set("mode", robotStatus->getMode());
    // obj->set("alarm", robotStatus->getAlarm());
    robot_basic_status->set("lccsstate", 0);  // TODO

    auto ems = robotStatus->getEmergency();
    robot_basic_status->set("emergency", ems);

    auto mode = robotStatus->getMode();
    robot_basic_status->set("mode", mode);
    
    bool standby = robotStatus->getStandby();
    robot_basic_status->set("stand_by", standby);
    int n_lift_status = robotStatus->getLoaded();
    int n_loaded = (n_lift_status == 1)? 1 : 0; 
    robot_basic_status->set("lift_status", n_lift_status);
    robot_basic_status->set("loaded", n_loaded);

    std::string s_pallet_id = robotStatus->getRFID();
    robot_basic_status->set("RFID", s_pallet_id);
    
    robot_basic_status->set("turntable_status", 0);
    robot_basic_status->set("conv_status", 0);
    robot_basic_status->set("task_id", TaskID);
    robot_basic_status->set("action_indx", n_action_index);
    robot_basic_status->set("acceleration", targetVel);
    robot_basic_status->set("loop_count", 0);
    robot_basic_status->set("feed_vel", feedVel);
    robot_basic_status->set("Cmd", "basic");
    robot_basic_status->set("msg_id", 0);
    robot_basic_status->set("map_correction", d_map_confidence);
    robot_basic_status->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
    robot_basic_status->set("pause_reason", 1);
    robot_basic_status->set("avoidance_state", 0);
    robot_basic_status->set("Result", "S");
    return robot_basic_status;
}

namespace NaviFra {

void PublisherInitializer::initialize()
{
    std::string robotId = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY)->getRobotID();

    PublisherManager::instance().addChannel(
        static_cast<int>(PUBLISH_CHANNEL::CHANNEL_BASIC),
        [robotId]() {
            try {
                auto obj = buildBasicStatusResponse();
                if (!obj) {
                    NLOG(warning) << "buildBasicStatusResponse returned nullptr";
                    return;
                }

                std::ostringstream oss;
                obj->stringify(oss);

                MessageBroker::instance().publish(robotId + ".ACS", oss.str());
            }
            catch (const Poco::Exception& ex) {
                NLOG(error) << ex.displayText();
            }
        },
        500, 0);

    // 기본은 비활성화 후, 외부에서 조건에 따라 start()
    PublisherManager::instance().deactivate(static_cast<int>(PUBLISH_CHANNEL::CHANNEL_BASIC));
    PublisherManager::instance().start();
}

}  // namespace NaviFra
