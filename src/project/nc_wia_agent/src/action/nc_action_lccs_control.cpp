#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_lccs_control.h>

using namespace NaviFra;

NcActionLCCSControl::NcActionLCCSControl()
{
}

NcActionLCCSControl::~NcActionLCCSControl()
{
}

std::string NcActionLCCSControl::implName()
{
    return "NcActionLCCSControl";
}

void NcActionLCCSControl::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int mode = obj->getValue<int>("mode");
        int data = obj->getValue<int>("data");
        int msg_id = obj->getValue<int>("msg_id");
        NLOG(info) << Poco::format("LCCS OnOff command received. RID: %s, mode: %d, data: %d", rid, mode, data);

        // 실제 제어 로직 위치
        // 예: lccsController.setOnOff(mode, data);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("message", Poco::Dynamic::Var());  // null
        response->set("success", false);  // 향후 로직 반영하여 true로 변경 가능
        response->set("Cmd", "lccs_onoff");
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
        NLOG(error) << "LccsOnOff Exception: " << ex.displayText();
    }
}
