#include "nc_wia_agent/nc_wia_agent.h"

#include <nc_wia_agent/action/nc_action_alarm_volume.h>

using namespace NaviFra;

NcActionAlarmVolume::NcActionAlarmVolume()
{
    send_volume_ = nh.advertise<std_msgs::Byte>("/navifra/sound_volume", 10);
}

NcActionAlarmVolume::~NcActionAlarmVolume()
{
}

std::string NcActionAlarmVolume::implName()
{
    return "NcActionAlarmVolume";
}

void NcActionAlarmVolume::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        std::string volumeStr = obj->getValue<std::string>("data");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Alarm volume set request. RID: %s, Volume: %s", rid, volumeStr);

        // 실제 볼륨 처리 로직 (예: 하드웨어 제어 등)
        uint8_t volume = std::stoi(volumeStr);
        std_msgs::Byte msg;
        msg.data = volume;
        send_volume_.publish(msg);
        NLOG(info) << "Wia Agent Publish Volume : " << volume;

        // AlarmController::instance().setVolume(volume);

        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "alarm_volume");
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
        NLOG(error) << "AlarmVolume Exception: " << ex.displayText();
    }
}
