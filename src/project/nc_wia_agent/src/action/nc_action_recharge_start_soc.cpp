#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_recharge_start_soc.h>

using namespace NaviFra;

NcActionRechargeStartSOC::NcActionRechargeStartSOC()
{
}

NcActionRechargeStartSOC::~NcActionRechargeStartSOC()
{
}

std::string NcActionRechargeStartSOC::implName()
{
    return "NcActionRechargeStartSOC";
}

void NcActionRechargeStartSOC::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int soc_threshold = obj->getValue<int>("data");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Recharge Start SOC set request. RID: %s, Value: %d", rid, soc_threshold);

        // 실제 SOC 설정 적용 로직 (예: Config 등에 저장)
        // Config::instance().setInt("recharge.start_soc", soc_threshold);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "recharge_start_soc");
        response->set("success", true);
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
        NLOG(error) << "RechargeStartSOC Exception: " << ex.displayText();
    }
}