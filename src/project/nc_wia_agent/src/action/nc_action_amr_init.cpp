#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_amr_init.h>

using namespace NaviFra;

NcActionAMRInit::NcActionAMRInit()
{
}

NcActionAMRInit::~NcActionAMRInit()
{
}

std::string NcActionAMRInit::implName()
{
    return "NcActionAMRInit";
}

void NcActionAMRInit::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        int msg_id = obj->getValue<int>("msg_id");

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        // response->set("version_info", versions);
        response->set("obstacle_state", 0);
        response->set("map_version", "1.0");
        response->set("Cmd", "amr_init");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", 0);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);
        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "amr_ver exception: " << ex.displayText();
    }
}
