#include "nc_wia_agent/nc_wia_agent.h"

#include <core_agent/data/memory_repository.h>
#include <core_agent/data/robot_info.h>
#include <nc_wia_agent/action/nc_action_error_reset.h>
#include <nc_wia_agent/data/alarm_status.h>

using namespace NaviFra;

NcActionErrorReset::NcActionErrorReset()
{
}

NcActionErrorReset::~NcActionErrorReset()
{
}

std::string NcActionErrorReset::implName()
{
    return "NcActionErrorReset";
}

void NcActionErrorReset::implonAction(std::string source, Poco::JSON::Object::Ptr obj)
{
    try {
        std::string rid = obj->getValue<std::string>("RID");
        int msg_id = obj->getValue<int>("msg_id");

        NLOG(info) << Poco::format("Alarm Reset command received from RID: %s", rid);

        auto robotInfo = InMemoryRepository::instance().get<RobotInfo>(RobotInfo::KEY);
        robotInfo->setAlarmDescription("");
        robotInfo->setAlarmId(0000);

        auto alarmInfo = InMemoryRepository::instance().get<AlarmStatus>(AlarmStatus::KEY);
        if (alarmInfo) {
            alarmInfo->clearAllAlarm();
            NLOG(info) << "All alarms cleared via error_reset command";
        }
        // 응답 메시지 구성
        Poco::JSON::Object::Ptr response = new Poco::JSON::Object;
        response->set("Cmd", "error_reset");
        response->set("msg_time", Poco::DateTimeFormatter::format(Poco::LocalDateTime(), "%Y-%m-%d %H:%M:%S.%i"));
        response->set("msg_id", msg_id);
        response->set("RID", rid);
        response->set("Result", "S");
        response->set("AmrId", rid);

        std::ostringstream oss;
        response->stringify(oss);

        MessageBroker::instance().publish(source, rid + ".ACS", oss.str());
        NLOG(info) << "Published Error Reset Message: " << oss.str();  // 테스트용 로그 출력
    }
    catch (const Poco::Exception& ex) {
        NLOG(error) << "Alarm Reset Exception: " << ex.displayText();
    }
}
