#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_init_pose.h>

using namespace NaviFra;

NcActionInitPose::NcActionInitPose()
{
}

NcActionInitPose::~NcActionInitPose()
{
}

std::string NcActionInitPose::implName()
{
    return "NcActionInitPose";
}

void NcActionInitPose::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        float x = obj->getValue<double>("x");
        float y = obj->getValue<double>("y");
        float theta = obj->getValue<double>("theta");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << "[InitPose] RID: " << rid << ", x: " << x << ", y: " << y << ", theta: " << theta;

        Position position{x, y, 0.0f};

        // yaw → quaternion 변환
        float half_theta = theta * 0.5f;
        Orientation orientation{0.0f, 0.0f, sin(half_theta), cos(half_theta)};

        bool correct_position = true;  // 기본값 또는 필요시 obj->get("correct_position");

        initialPose(position, orientation, correct_position);

        // 응답 메시지 전송
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "initposition");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "[InitPose] Exception: " << ex.displayText();
    }
}
