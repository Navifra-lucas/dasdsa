#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_cancel.h>

using namespace NaviFra;

NcActionCancel::NcActionCancel()
{
}

NcActionCancel::~NcActionCancel()
{
}

std::string NcActionCancel::implName()
{
    return "NcActionCancel";
}

void NcActionCancel::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Cancel command received from RID: %s", rid);

        robotStop();
        status->setACSCancelCmd(true);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "cancel");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
        // TaskResultPublisher::instance().publish("PREEMPTED");
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Cancel Exception: " << ex.displayText();
    }
}
