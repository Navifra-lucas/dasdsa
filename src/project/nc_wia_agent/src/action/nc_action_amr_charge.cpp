#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/data/memory_repository.h>
#include <nc_wia_agent/action/nc_action_amr_charge.h>

using namespace NaviFra;

NcActionAmrCharge::NcActionAmrCharge()
{
}

NcActionAmrCharge::~NcActionAmrCharge()
{
}

std::string NcActionAmrCharge::implName()
{
    return "NcActionAmrCharge";
}

void NcActionAmrCharge::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        int msg_id = obj->getValue<int>("msg_id");
        // 응답 메시지 구성
        std::string data = obj->getValue<std::string>("data");

        // data 처리 채우기

        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "charge");
        response->set("data", "");
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
        NLOG(error) << "AmrMode Exception: " << ex.displayText();
    }
}
