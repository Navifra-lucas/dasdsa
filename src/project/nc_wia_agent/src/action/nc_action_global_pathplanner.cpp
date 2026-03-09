#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_global_pathplanner.h>

using namespace NaviFra;

NcActionGlobalPathplanner::NcActionGlobalPathplanner()
{
}

NcActionGlobalPathplanner::~NcActionGlobalPathplanner()
{
}

std::string NcActionGlobalPathplanner::implName()
{
    return "NcActionGlobalPathplanner";
}

void NcActionGlobalPathplanner::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("GlobalPlanner request received. RID: %s", rid);

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("header", Poco::Dynamic::Var());  // 현재는 null
        response->set("poses", Poco::Dynamic::Var());  // 현재는 null
        response->set("Cmd", "globalplanner");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("ErrorCode", 0);  // 향후 실제 실패 시 오류 코드 지정 가능

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "GlobalPlanner Exception: " << ex.displayText();
    }
}
