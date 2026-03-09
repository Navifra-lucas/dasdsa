#include "nc_wia_agent/data/robot_basic_status.h"
#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_result.h>

using namespace NaviFra;
NcActionResult::NcActionResult()
{
}

NcActionResult::~NcActionResult()
{
}

std::string NcActionResult::implName()
{
    return "NcActionResult";
}

void NcActionResult::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        auto status = InMemoryRepository::instance().get<RobotBasicStatus>(RobotBasicStatus::KEY);

        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "result");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("AmrId", rid);
        response->set("Result", "S");

        std::ostringstream oss;
        response->stringify(oss);

        status->setResultStopTrigger(true);
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Result Exception: " << ex.displayText();
    }
}
