#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_time_sync.h>

using namespace NaviFra;

NcActionTimeSync::NcActionTimeSync()
{
}

NcActionTimeSync::~NcActionTimeSync()
{
}

std::string NcActionTimeSync::implName()
{
    return "NcActionTimeSync";
}

void NcActionTimeSync::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string datetime = obj->getValue<std::string>("datetime");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << "RID: " << rid << ", datetime: " << datetime;

        // 필요시 시스템 시간 설정 로직
        // std::string cmd = "date -s \"" + datetime + "\"";
        // std::system(cmd.c_str());

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "timesync");
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
        NLOG(error) << "Exception: " << ex.displayText();
    }
}
