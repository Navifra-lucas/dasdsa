#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_lift_status.h>
#include <task_msgs/Loading.h>

using namespace NaviFra;
NcActionLiftStatus::NcActionLiftStatus()
{
}

NcActionLiftStatus::~NcActionLiftStatus()
{
}

std::string NcActionLiftStatus::implName()
{
    return "NcActionLiftStatus";
}

void NcActionLiftStatus::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");
        int height, lift_status;

        lift_status = status->getLiftStatus();

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "lift_status");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("height", 0);
        response->set("lift_status", lift_status);
        response->set("RID", rid);
        response->set("AmrId", rid);
        response->set("Result", "S");

        std::ostringstream oss;
        response->stringify(oss);

        NLOG(info) << "lift_status is : " << lift_status;

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "LiftCmd Exception: " << ex.displayText();
    }
}
