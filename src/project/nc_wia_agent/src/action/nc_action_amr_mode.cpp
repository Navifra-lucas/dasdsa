#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/data/memory_repository.h>
#include <nc_wia_agent/action/nc_action_amr_mode.h>

using namespace NaviFra;

NcActionAmrMode::NcActionAmrMode()
{
}

NcActionAmrMode::~NcActionAmrMode()
{
}

std::string NcActionAmrMode::implName()
{
    return "NcActionAmrMode";
}

void NcActionAmrMode::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        int msg_id = obj->getValue<int>("msg_id");
        // 응답 메시지 구성
        std::string data = obj->getValue<std::string>("data");

        // data 처리 채우기

        // rep 비어있는데 rep 필요한지..
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "mode");
        response->set("data", "");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        // response->set("Result", "S");
        // response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "AmrMode Exception: " << ex.displayText();
    }
}
