#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_reset_mileage.h>

using namespace NaviFra;

NcActionResetMileage::NcActionResetMileage()
{
}

NcActionResetMileage::~NcActionResetMileage()
{
}

std::string NcActionResetMileage::implName()
{
    return "NcActionResetMileage";
}

void NcActionResetMileage::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string volumeStr = obj->getValue<std::string>("data");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Alarm volume set request. RID: %s, Volume: %s", rid, volumeStr);

        // 실제 볼륨 처리 로직 (예: 하드웨어 제어 등)
        // int volume = std::stoi(volumeStr);
        // AlarmController::instance().setVolume(volume);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "reset_mileage");
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
        NLOG(error) << "AlarmVolume Exception: " << ex.displayText();
    }
}
