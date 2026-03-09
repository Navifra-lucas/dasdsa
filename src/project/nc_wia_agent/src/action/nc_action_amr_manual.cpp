#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/core/navicore.h>
#include <nc_wia_agent/action/nc_action_amr_manual.h>

using namespace NaviFra;

NcActionAMRManual::NcActionAMRManual()
{
}

NcActionAMRManual::~NcActionAMRManual()
{
}

std::string NcActionAMRManual::implName()
{
    return "NcActionAMRManual";
}

void NcActionAMRManual::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string msg_time = obj->getValue<std::string>("msg_time");
        int msg_id = obj->getValue<int>("msg_id");

        // 속도 파싱
        Poco::JSON::Object::Ptr linear = obj->getObject("linear");
        Poco::JSON::Object::Ptr angular = obj->getObject("angular");

        float lin_x = -linear->getValue<float>("x");
        float lin_y = -linear->getValue<float>("y");
        float lin_z = linear->getValue<float>("z");
        float ang_x = angular->getValue<float>("x");
        float ang_y = angular->getValue<float>("y");
        float ang_z = angular->getValue<float>("z");

        NLOG(info) << Poco::format(
            "Manual cmd: linear(%.2f, %.2f, %.2f), angular(%.2f, %.2f, %.2f)", lin_x, lin_y, lin_z, ang_x, ang_y, ang_z);

        cmdVel(lin_x, lin_y, ang_z);

        // 응답 전송
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "manual");
        response->set("msg_time", msg_time);
        response->set("msg_id", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("RID", rid);
        response->set("AmrId", rid);
        response->set("Result", "S");

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "ManualDrive Exception: " << ex.displayText();
    }
}
